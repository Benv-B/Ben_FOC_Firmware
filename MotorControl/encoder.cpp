#include <stm32_system.h>
#include <bitset>
#include "encoder.hpp"

void Encoder::setup()
{
    HAL_TIM_Encoder_Start(timer_, TIM_CHANNEL_ALL);

    // mode_ = config_.mode;

    spi_task_.config = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_16BIT,
        .CLKPolarity = SPI_POLARITY_LOW,
        .CLKPhase = SPI_PHASE_2EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial = 10,
    };

    abs_spi_cs_pin_init();

    // if (axis_->controller_.config_.anticogging.pre_calibrated)
    // {
    //     axis_->controller_.anticogging_valid_ = true;
    // }
}

// bool Encoder::run_direction_find()
// {
//     int32_t init_enc_val = shadow_count_;

//     Axis::LockinConfig_t lockin_config = axis_->config_.calibration_lockin;
//     lockin_config.finish_distance = lockin_config.vel * 3.0f; // run for 3 seconds
//     lockin_config.finish_on_distance = true;
//     lockin_config.finish_on_enc_idx = false;
//     lockin_config.finish_on_vel = false;
//     bool success = axis_->run_lockin_spin(lockin_config, false);

//     if (success)
//     {
//         // Check response and direction
//         if (shadow_count_ > init_enc_val + 8)
//         {
//             // motor same dir as encoder
//             config_.direction = 1;
//         }
//         else if (shadow_count_ < init_enc_val - 8)
//         {
//             // motor opposite dir as encoder
//             config_.direction = -1;
//         }
//         else
//         {
//             config_.direction = 0;
//         }
//     }

//     return success;
// }

void Encoder::abs_spi_cs_pin_init()
{

    abs_spi_cs_gpio_ = {GPIOA, GPIO_PIN_15};
    // Write pin high
    abs_spi_cs_gpio_.write(true);
}

void Encoder::sample_now()
{
    abs_spi_start_transaction();
}

bool Encoder::abs_spi_start_transaction()
{
    if (Stm32SpiArbiter::acquire_task(&spi_task_))
    {
        spi_task_.ncs_gpio = abs_spi_cs_gpio_;
        spi_task_.tx_buf = (uint8_t)abs_spi_dma_tx_;
        spi_task_.rx_buf = (uint8_t)abs_spi_dma_rx_;
        spi_task_.length = 1;
        spi_task_.on_complete = [](void *ctx, bool sucess)
        { ((Encoder *)ctx)->abs_spi_cb(sucess) };
        spi_task_.on_complete_ctx = this;

        Stm32SpiArbiter::transfer_async(&spi_task_);
    }
    else
    {
        return false;
    }
    return true;
}

uint8_t ams_parity(uint16_t v)
{
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}

void Encoder::abs_spi_cb(bool success)
{
    uint16_t pos;

    if (!success)
    {
        goto done;
    }

    uint16_t rawVal = abs_spi_dma_rx_[0];
    // check if parity is correct (even) and error flag clear
    if (ams_parity(rawVal) || ((rawVal >> 14) & 1))
    {
        goto done;
    }
    pos = rawVal & 0x3fff;
    pos_abs_ = pos;
    abs_spi_pos_updated_ = true;
    if (config_.pre_calibrated)
    {
        is_ready_ = true;
    }

done:
    Stm32SpiArbiter::release_task(&spi_task_);
}

void Encoder::abs_spi_cs_pin_init()
{
    abs_spi_cs_gpio_ = get_gpio(config_.abs_spi_cs_gpio_pin);
    abs_spi_cs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

    // Write pin high
    abs_spi_cs_gpio_.write(true);
}

bool Encoder::update()
{
    // update internal encoder state.
    int32_t delta_enc = 0;
    int32_t pos_abs_latched = pos_abs_; // LATCH

    /** This data low-pass filter for spi error rate*/
    // if (abs_spi_pos_updated_ == false)
    // {
    //     // Low pass filter the error
    //     spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
    //     if (spi_error_rate_ > 0.05f)
    //     {
    //         set_error(ERROR_ABS_SPI_COM_FAIL);
    //         return false;
    //     }
    // }
    // else
    // {
    //     // Low pass filter the error
    //     spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
    // }

    abs_spi_pos_updated_ = false;
    delta_enc = pos_abs_latched - count_in_cpr_; // LATCH
    delta_enc = mod(delta_enc, config_.cpr);
    if (delta_enc > config_.cpr / 2)
    {
        delta_enc -= config_.cpr;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ = pos_abs_latched;

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    //// run pll (for now pll is in units of encoder counts)
    // Encoder model
    auto encoder_model = [this](float internal_pos) -> int32_t
    {
        return (int32_t)std::floor(internal_pos);
    };
    // Predict current pos
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_ += current_meas_period * vel_estimate_counts_;
    // discrete phase detector
    float delta_pos_counts = (float)(shadow_count_ - encoder_model(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - encoder_model(pos_cpr_counts_));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    // pll feedback
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (std::abs(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_)
    {
        vel_estimate_counts_ = 0.0f; // align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Outputs from Encoder for Controller
    pos_estimate_ = pos_estimate_counts_ / (float)config_.cpr;
    vel_estimate_ = vel_estimate_counts_ / (float)config_.cpr;

    // // TODO: we should strictly require that this value is from the previous iteration
    // // to avoid spinout scenarios. However that requires a proper way to reset
    // // the encoder from error states.
    // float pos_circular = pos_circular_.any().value_or(0.0f);
    // pos_circular += wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    // pos_circular = fmodf_pos(pos_circular, axis_->controller_.config_.circular_setpoint_range);
    // pos_circular_ = pos_circular;

    /** no need for interpolation for now */
    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.phase_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation)
    {
        interpolation_ = 0.5f;
        // reset interpolation if encoder edge comes
        // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    }
    else if (delta_enc > 0)
    {
        interpolation_ = 0.0f;
    }
    else if (delta_enc < 0)
    {
        interpolation_ = 1.0f;
    }
    else
    {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f)
            interpolation_ = 1.0f;
        if (interpolation_ < 0.0f)
            interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    // TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = motor.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.phase_offset_float);

    if (is_ready_)
    {
        phase_ = wrap_pm_pi(ph) * config_.direction;
        phase_vel_ = (2 * M_PI) * *vel_estimate_.present() * motor.config_.pole_pairs * config_.direction;
    }

    return true;
}
