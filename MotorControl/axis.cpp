#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "axis.hpp"
#include "utils.hpp"
#include "ben_drive_main.h"
// #include "communication/interface_can.hpp"

Axis::Axis(osPriority thread_priority,
           Encoder &encoder,
           Controller &controller,
           Motor &motor)
    : thread_priority_(thread_priority),
      encoder_(encoder),
      controller_(controller),
      motor_(motor)
{
    motor_.axis_ = this;
    encoder_.axis_ = this;
    controller_.axis_ = this;
}

/**
 * @brief Blocks until at least one complete control loop has been executed.
 */
bool Axis::wait_for_control_iteration()
{
    osSignalWait(0x0001, osWaitForever); // this might return instantly
    osSignalWait(0x0001, osWaitForever); // this might be triggered at the
                                         // end of a control loop iteration
                                         // which was started before we entered
                                         // this function
    osSignalWait(0x0001, osWaitForever);
    return true;
}

bool Axis::start_closed_loop_control()
{
    // Hook up the data paths between the components
    CRITICAL_SECTION()
    {
        // TODO: modify this
        Axis *ax = &axis1;
        // controller_.pos_estimate_circular_src_.connect_to(&ax->encoder_.pos_circular_);
        // controller_.pos_wrap_src_.connect_to(&controller_.config_.circular_setpoint_range);
        controller_.pos_estimate_linear_src_.connect_to(&ax->encoder_.pos_estimate_);
        controller_.vel_estimate_src_.connect_to(&ax->encoder_.vel_estimate_);

        // To avoid any transient on startup, we intialize the setpoint to be the current position
        controller_.control_mode_updated();
        controller_.input_pos_updated();

        // Avoid integrator windup issues
        controller_.vel_integrator_torque_ = 0.0f;

        motor_.torque_setpoint_src_.connect_to(&controller_.torque_output_);
        motor_.direction_ = encoder_.config_.direction;

        motor_.current_control_.enable_current_control_src_ = true;
        motor_.current_control_.Idq_setpoint_src_.connect_to(&motor_.Idq_setpoint_);
        motor_.current_control_.Vdq_setpoint_src_.connect_to(&motor_.Vdq_setpoint_);

        bool is_acim = false;
        // phase
        OutputPort<float> *phase_src = &encoder_.phase_;
        // acim_estimator_.rotor_phase_src_.connect_to(phase_src);
        OutputPort<float> *stator_phase_src = phase_src;
        motor_.current_control_.phase_src_.connect_to(stator_phase_src);
        // phase vel
        OutputPort<float> *phase_vel_src = &encoder_.phase_vel_;
        // acim_estimator_.rotor_phase_vel_src_.connect_to(phase_vel_src);
        OutputPort<float> *stator_phase_vel_src = phase_vel_src;
        motor_.phase_vel_src_.connect_to(stator_phase_vel_src);
        motor_.current_control_.phase_vel_src_.connect_to(stator_phase_vel_src);
    }

    // In sensorless mode the motor is already armed.
    if (!motor_.is_armed_)
    {
        wait_for_control_iteration();
        motor_.arm(&motor_.current_control_);
    }

    return true;
}

bool Axis::stop_closed_loop_control()
{
    return motor_.disarm();
}

bool Axis::run_closed_loop_control_loop()
{
    start_closed_loop_control();
    // set_step_dir_active(config_.enable_step_dir);

    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_)
    {
        osDelay(1);
    }

    // set_step_dir_active(config_.enable_step_dir && config_.step_dir_always_on);
    stop_closed_loop_control();

    return check_for_errors();
}

bool Axis::run_idle_loop()
{
    last_drv_fault_ = motor_.gate_driver_.get_error();
    // TODO : check if this works
    axis1.motor_.disarm();
    // mechanical_brake_.engage();
    // set_step_dir_active(config_.enable_step_dir && config_.step_dir_always_on);
    while (requested_state_ == AXIS_STATE_UNDEFINED)
    {
        motor_.setup();
        osDelay(1);
    }
    return check_for_errors();
}

// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop()
{
    for (;;)
    {
        // Load the task chain if a specific request is pending
        if (requested_state_ != AXIS_STATE_UNDEFINED)
        {
            size_t pos = 0;
            if (requested_state_ == AXIS_STATE_STARTUP_SEQUENCE)
            {
                if (config_.startup_motor_calibration)
                    task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                if (config_.startup_encoder_offset_calibration)
                    task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                if (config_.startup_homing)
                    task_chain_[pos++] = AXIS_STATE_HOMING;
                if (config_.startup_closed_loop_control)
                    task_chain_[pos++] = AXIS_STATE_CLOSED_LOOP_CONTROL;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            }
            else if (requested_state_ == AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
            {
                task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            }
            else if (requested_state_ != AXIS_STATE_UNDEFINED)
            {
                task_chain_[pos++] = requested_state_;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            }
            task_chain_[pos++] = AXIS_STATE_UNDEFINED; // TODO: bounds checking
            requested_state_ = AXIS_STATE_UNDEFINED;
            // // Auto-clear any invalid state error
            // error_ &= ~ERROR_INVALID_STATE;
        }

        // Note that current_state is a reference to task_chain_[0]

        // Run the specified state
        // Handlers should exit if requested_state != AXIS_STATE_UNDEFINED
        bool status;
        switch (current_state_)
        {
        case AXIS_STATE_CLOSED_LOOP_CONTROL:
        {
            // if (odrv.any_error())
            //     goto invalid_state_label;
            if (!axis1.motor_.is_calibrated_ || axis1.encoder_.config_.direction == 0)
                goto invalid_state_label;
            // watchdog_feed();
            status = run_closed_loop_control_loop();
        }
        break;

        case AXIS_STATE_IDLE:
        {
            run_idle_loop();
            status = true;
        }
        break;

            // case AXIS_STATE_MOTOR_CALIBRATION:
            // {
            //     // These error checks are a hacky way to force legacy behavior
            //     // when an error is raised. TODO: remove this when we overhaul
            //     // the error architecture
            //     // (https://github.com/madcowswe/ODrive/issues/526).
            //     // if (odrv.any_error())
            //     //    goto invalid_state_label;
            //     status = motor.run_calibration();
            // }
            // break;

            // case AXIS_STATE_ENCODER_DIR_FIND:
            // {
            //     // if (odrv.any_error())
            //     //     goto invalid_state_label;
            //     // if (!motor_.is_calibrated_)
            //     //     goto invalid_state_label;

            //     status = encoder_.run_direction_find();
            //     // Help facilitate encoder.is_ready without reboot
            //     if (status)
            //         encoder_.apply_config(motor_.config_.motor_type);
            // }
            // break;

            // case AXIS_STATE_HOMING:
            // {
            //     Controller::ControlMode stored_control_mode = controller_.config_.control_mode;
            //     Controller::InputMode stored_input_mode = controller_.config_.input_mode;

            //     status = run_homing();

            //     controller_.config_.control_mode = stored_control_mode;
            //     controller_.config_.input_mode = stored_input_mode;
            // }
            // break;

            // case AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
            // {
            //     // if (odrv.any_error())
            //     //     goto invalid_state_label;
            //     // if (!motor_.is_calibrated_)
            //     //     goto invalid_state_label;
            //     status = encoder_.run_offset_calibration();
            // }
            // break;

        default:
        invalid_state_label:
            // error_ |= ERROR_INVALID_STATE;
            status = false; // this will set the state to idle
            break;
        }
        // If the state failed, go to idle, else advance task chain
        if (!status)
        {
            std::fill(task_chain_.begin(), task_chain_.end(), AXIS_STATE_UNDEFINED);
            current_state_ = AXIS_STATE_IDLE;
        }
        else
        {
            std::rotate(task_chain_.begin(), task_chain_.begin() + 1, task_chain_.end());
            task_chain_.back() = AXIS_STATE_UNDEFINED;
        }
    }
}

static void run_state_machine_loop_wrapper(void *ctx)
{
    reinterpret_cast<Axis *>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis *>(ctx)->thread_id_valid_ = false;
}

// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread()
{
    osThreadDef(thread_def, (os_pthread)run_state_machine_loop_wrapper, thread_priority_, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(thread_def), this);
    thread_id_valid_ = true;
}