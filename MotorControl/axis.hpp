#ifndef __AXIS_HPP
#define __AXIS_HPP

#include "encoder.hpp"
#include "interface.hpp"
#include "utils.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "controller.hpp"

#include <array>

class Axis : public BenDrive_Intf::Axis_Intf
{
public:
    enum AxisState
    {
        AXIS_STATE_UNDEFINED = 0,
        AXIS_STATE_IDLE = 1,
        AXIS_STATE_STARTUP_SEQUENCE = 2,
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
        AXIS_STATE_MOTOR_CALIBRATION = 4,
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 5,
        AXIS_STATE_CLOSED_LOOP_CONTROL = 6,
        AXIS_STATE_ENCODER_DIR_FIND = 7,
        AXIS_STATE_HOMING = 8,
    };

    struct Config_t
    {
        bool startup_motor_calibration = false;          //<! run motor calibration at startup, skip otherwise
        bool startup_encoder_offset_calibration = false; //<! run encoder offset calibration after startup, skip otherwise
        bool startup_closed_loop_control = false;        //<! enable closed loop control after calibration/startup
        bool startup_homing = false;                     //<! enable homing after calibration/startup

        // bool enable_step_dir = false;    //<! enable step/dir input after calibration
        //                                  //   For M0 this has no effect if enable_uart is true
        // bool step_dir_always_on = false; //<! Keep step/dir enabled while the motor is disabled.
        //                                  //<! This is ignored if enable_step_dir is false.
        //                                  //<! This setting only takes effect on a state transition
        //                                  //<! into idle or out of closed loop control.

        // bool enable_sensorless_mode = false;

        // float watchdog_timeout = 0.0f; // [s]
        // bool enable_watchdog = false;

        // // Defaults loaded from hw_config in load_configuration in main.cpp
        // uint16_t step_gpio_pin = 0;
        // uint16_t dir_gpio_pin = 0;

        // LockinConfig_t calibration_lockin = default_calibration();
        // LockinConfig_t sensorless_ramp = default_sensorless();
        // LockinConfig_t general_lockin;

        // CANConfig_t can;

        // // custom setters
        // Axis *parent = nullptr;
        // void set_step_gpio_pin(uint16_t value)
        // {
        //     step_gpio_pin = value;
        //     parent->decode_step_dir_pins();
        // }
        // void set_dir_gpio_pin(uint16_t value)
        // {
        //     dir_gpio_pin = value;
        //     parent->decode_step_dir_pins();
        // }
    };

    Axis(osPriority thread_priority,
         Encoder &encoder,
         Controller &controller,
         Motor &motor);

    bool wait_for_control_iteration();
    void start_thread();
    bool start_closed_loop_control();
    bool stop_closed_loop_control();
    // bool run_lockin_spin(const LockinConfig_t &lockin_config, bool remain_armed,
    //                      std::function<bool(bool)> loop_cb = {});
    bool run_closed_loop_control_loop();
    bool run_homing();
    bool run_idle_loop();

    void run_state_machine_loop();

    AxisState requested_state_ = AXIS_STATE_STARTUP_SEQUENCE;
    std::array<AxisState, 10> task_chain_ = {AXIS_STATE_UNDEFINED};
    AxisState &current_state_ = task_chain_.front();

    // hardware config
    Config_t config_;
    uint16_t default_step_gpio_pin_;
    uint16_t default_dir_gpio_pin_;
    osPriority thread_priority_;

    Encoder &encoder_;
    // AcimEstimator acim_estimator_;
    // SensorlessEstimator &sensorless_estimator_;
    Controller &controller_;
    Motor &motor_;

    int64_t steps_ = 0; // Steps counted at interface

    osThreadId thread_id_ = 0;
    const uint32_t stack_size_ = 2048; // Bytes
    volatile bool thread_id_valid_ = false;
    uint32_t last_drv_fault_ = 0;

    BenDrive_Intf::Axis_Intf::Error error_ = ERROR_NONE;
    // True if there are no errors
    bool inline check_for_errors()
    {
        return error_ == ERROR_NONE;
    }
};

#endif /* __AXIS_HPP */
