#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#include "my_Drivers/stm32_spi_arbiter.hpp"

// Only Magnetic Encoder is used
class Encoder
{
public:
    struct Config_t
    {
        bool enable_phase_interpolation = true;
        int32_t phase_offset = 0;        // Offset between encoder count and rotor electrical phase
        float phase_offset_float = 0.0f; // Sub-count phase alignment offset
        int32_t direction = 0;           // direction with respect to motor
        int32_t cpr = (1024 * 16);       // resolution of AMS5047P sensor,
    };

    Encoder(Stm32SpiArbiter *spi_arbiter) : spi_arbiter_(spi_arbiter) {}
    void setup();
    void sample_now();
    void abs_spi_cs_pin_init();

    Axis *axis_ = nullptr; // set by Axis constructor

    float pll_kp_ = 0.0f; // [count/s / count]
    float pll_ki_ = 0.0f; // [(count/s^2) / count]
    int32_t count_in_cpr_ = 0;
    int32_t shadow_count_ = 0;
    float delta_pos_cpr_counts_ = 0.0f; // [count] phase detector result for debug
    float pos_cpr_counts_ = 0.0f;       // [count]
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float interpolation_ = 0.0f;

    OutputPort<float> phase_ = 0.0f;        // [rad]
    OutputPort<float> phase_vel_ = 0.0f;    // [rad/s]
    OutputPort<float> pos_estimate_ = 0.0f; // [turn]
    OutputPort<float> vel_estimate_ = 0.0f; // [turn/s]
    // OutputPort<float> pos_circular_ = 0.0f; // [turn]

    int32_t pos_abs_ = 0;
    bool abs_spi_start_transaction();
    void abs_spi_cb(bool success);
    void abs_spi_cs_pin_init();
    bool abs_spi_pos_updated_ = false;
    Stm32Gpio abs_spi_cs_gpio_;
    uint16_t abs_spi_dma_tx_[1] = {0xFFFF};
    uint16_t abs_spi_dma_rx_[1];
    Stm32SpiArbiter *spi_arbiter_;
    Stm32SpiArbiter::SpiTask spi_task_;

    Config_t config_;

    constexpr float getCoggingRatio()
    {
        return 1.0f / 3600.0f;
    }
};
#endif