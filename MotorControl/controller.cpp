#include "controller.hpp"
#include <algorithm>
#include <numeric>

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 *
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate)
{
    float pos_err = input_pos_ - pos_estimate;
    if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / (float)ams_encoder.config_.cpr &&
        std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / (float)ams_encoder.config_.cpr)
    {
        config_.anticogging.cogging_map[std::clamp<uint32_t>(config_.anticogging.index++, 0, 3600)] = vel_integrator_torque_;
    }
    if (config_.anticogging.index < 3600)
    {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * ams_encoder.getCoggingRatio();
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    }
    else
    {
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = 0.0f; // Send the motor home
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        anticogging_valid_ = true;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

bool Controller::control_mode_updated()
{
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL)
    {
        std::optional<float> estimate = pos_estimate_linear_src_.any();
        if (!estimate.has_value())
        {
            return false;
        }

        pos_setpoint_ = *estimate;
        set_input_pos_and_steps(*estimate);
    }
    return true;
}

void Controller::set_input_pos_and_steps(float const pos)
{
    input_pos_ = pos;
    axis_->steps_ = (int64_t)(pos * config_.steps_per_circular_range);
}

bool Controller::update()
{
    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.present();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.present();
    // std::optional<float> pos_wrap = pos_wrap_src_.present();
    std::optional<float> vel_estimate = vel_estimate_src_.present();

    std::optional<float> anticogging_pos_estimate = ams_encoder.pos_estimate_.present();
    std::optional<float> anticogging_vel_estimate = ams_encoder.vel_estimate_.present();

    if (config_.anticogging.calib_anticogging)
    {
        if (!anticogging_pos_estimate.has_value() || !anticogging_vel_estimate.has_value())
        {
            // set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate);
    }

    // Update inputs
    switch (config_.input_mode)
    {
    case INPUT_MODE_INACTIVE:
    {
        // do nothing
    }
    break;
    case INPUT_MODE_PASSTHROUGH:
    {
        pos_setpoint_ = input_pos_;
        vel_setpoint_ = input_vel_;
        torque_setpoint_ = input_torque_;
    }
    break;
    case INPUT_MODE_VEL_RAMP:
    {
        float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
        float full_step = input_vel_ - vel_setpoint_;
        float step = std::clamp(full_step, -max_step_size, max_step_size);

        vel_setpoint_ += step;
        torque_setpoint_ = (step / current_meas_period) * config_.inertia;
    }
    break;
    case INPUT_MODE_TORQUE_RAMP:
    {
        float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
        float full_step = input_torque_ - torque_setpoint_;
        float step = std::clamp(full_step, -max_step_size, max_step_size);

        torque_setpoint_ += step;
    }
    break;
    case INPUT_MODE_POS_FILTER:
    {
        // 2nd order pos tracking filter
        float delta_pos = input_pos_ - pos_setpoint_;                              // Pos error
        float delta_vel = input_vel_ - vel_setpoint_;                              // Vel error
        float accel = input_filter_kp_ * delta_pos + input_filter_ki_ * delta_vel; // Feedback
        torque_setpoint_ = accel * config_.inertia;                                // Accel
        vel_setpoint_ += current_meas_period * accel;                              // delta vel
        pos_setpoint_ += current_meas_period * vel_setpoint_;                      // Delta pos
    }
    break;
    default:
    {
        // set_error(ERROR_INVALID_INPUT_MODE);
        return false;
    }
    }

    // Never command a setpoint beyond its limit
    if (config_.enable_vel_limit)
    {
        vel_setpoint_ = std::clamp(vel_setpoint_, -config_.vel_limit, config_.vel_limit);
    }
    const float Tlim = motor.max_available_torque();
    torque_setpoint_ = std::clamp(torque_setpoint_, -Tlim, Tlim);

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL)
    {
        float pos_err;

        if (!pos_estimate_linear.has_value())
        {
            // set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        pos_err = pos_setpoint_ - *pos_estimate_linear;

        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width)
        {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit)
    {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled)
    {
        if (!anticogging_pos_estimate.has_value())
        {
            // set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        float anticogging_pos = *anticogging_pos_estimate / ams_encoder.getCoggingRatio();
        torque += config_.anticogging.cogging_map[std::clamp(mod((int)anticogging_pos, 3600), 0, 3600)];
    }

    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL)
    {
        if (!vel_estimate.has_value())
        {
            // set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
    }

    // Torque limiting
    bool limited = false;
    if (torque > Tlim)
    {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim)
    {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL)
    {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    }
    else
    {
        if (limited)
        {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        }
        else
        {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
        // integrator limiting to prevent windup
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }

    float ideal_electrical_power = 0.0f;
    ideal_electrical_power = motor.current_control_.power_;
    mechanical_power_ += config_.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    electrical_power_ += config_.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

    // Spinout check
    // If mechanical power is negative (braking) and measured power is positive, something is wrong
    // This indicates that the controller is trying to stop, but torque is being produced.
    // Usually caused by an incorrect encoder offset
    if (mechanical_power_ < config_.spinout_mechanical_power_threshold && electrical_power_ > config_.spinout_electrical_power_threshold)
    {
        // set_error(ERROR_SPINOUT_DETECTED);
        return false;
    }

    torque_output_ = torque;

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    // error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}
