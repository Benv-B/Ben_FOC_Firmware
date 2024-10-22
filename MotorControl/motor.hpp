#ifndef __MOTOR_HPP
#define __MOTOR_HPP

class Axis; // declared in axis.hpp
class Motor;

#include "foc.hpp"

class Motor
{
public:
    enum Error
    {
        ERROR_NONE = 0x00000000,
        ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001,
        ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002,
        ERROR_DRV_FAULT = 0x00000008,
        ERROR_CONTROL_DEADLINE_MISSED = 0x00000010,
        ERROR_MODULATION_MAGNITUDE = 0x00000080,
        ERROR_CURRENT_SENSE_SATURATION = 0x00000400,
        ERROR_CURRENT_LIMIT_VIOLATION = 0x00001000,
        ERROR_MODULATION_IS_NAN = 0x00010000,
        ERROR_MOTOR_THERMISTOR_OVER_TEMP = 0x00020000,
        ERROR_FET_THERMISTOR_OVER_TEMP = 0x00040000,
        ERROR_TIMER_UPDATE_MISSED = 0x00080000,
        ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = 0x00100000,
        ERROR_CONTROLLER_FAILED = 0x00200000,
        ERROR_I_BUS_OUT_OF_RANGE = 0x00400000,
        ERROR_BRAKE_RESISTOR_DISARMED = 0x00800000,
        ERROR_SYSTEM_LEVEL = 0x01000000,
        ERROR_BAD_TIMING = 0x02000000,
        ERROR_UNKNOWN_PHASE_ESTIMATE = 0x04000000,
        ERROR_UNKNOWN_PHASE_VEL = 0x08000000,
        ERROR_UNKNOWN_TORQUE = 0x10000000,
        ERROR_UNKNOWN_CURRENT_COMMAND = 0x20000000,
        ERROR_UNKNOWN_CURRENT_MEASUREMENT = 0x40000000,
        ERROR_UNKNOWN_VBUS_VOLTAGE = 0x80000000,
        ERROR_UNKNOWN_VOLTAGE_COMMAND = 0x100000000,
        ERROR_UNKNOWN_GAINS = 0x200000000,
        ERROR_CONTROLLER_INITIALIZING = 0x400000000,
        ERROR_UNBALANCED_PHASES = 0x800000000,
    };

    // NOTE: for gimbal motors, all units of Nm are instead V.
    // example: vel_gain is [V/(turn/s)] instead of [Nm/(turn/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t
    {
        // bool pre_calibrated = false; // can be set to true to indicate that all values here are valid
        int32_t pole_pairs = 7;
        float calibration_current = 10.0f;         // [A]
        float resistance_calib_max_voltage = 2.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        float phase_inductance = 0.0f;             // to be set by measure_phase_inductance
        float phase_resistance = 0.0f;             // to be set by measure_phase_resistance
        float torque_constant = 0.04f;             // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
        // MotorType motor_type = MOTOR_TYPE_HIGH_CURRENT;
        // Read out max_allowed_current to see max supported value for current_lim.
        // float current_lim = 70.0f; //[A]
        // float current_lim = 10.0f;                                 //[A]
        // float current_lim_margin = 8.0f;                           // Maximum violation of current_lim
        // float torque_lim = std::numeric_limits<float>::infinity(); //[Nm].
        // Value used to compute shunt amplifier gains
        float requested_current_range = 30.0f; // [A]
        // float current_control_bandwidth = 1000.0f; // [rad/s]
        // float inverter_temp_limit_lower = 100;
        // float inverter_temp_limit_upper = 120;

        // float acim_gain_min_flux = 10;   // [A]
        // float acim_autoflux_min_Id = 10; // [A]
        // bool acim_autoflux_enable = false;
        // float acim_autoflux_attack_gain = 10.0f;
        // float acim_autoflux_decay_gain = 1.0f;

        bool R_wL_FF_enable = false; // Enable feedforwards for R*I and w*L*I terms
        bool bEMF_FF_enable = false; // Enable feedforward for bEMF

        float I_bus_hard_min = -INFINITY;
        float I_bus_hard_max = INFINITY;
        // float I_leak_max = 0.1f;

        float dc_calib_tau = 0.2f;

        // // custom property setters
        // Motor *parent = nullptr;
        // void set_pre_calibrated(bool value)
        // {
        //     pre_calibrated = value;
        //     parent->is_calibrated_ = parent->is_calibrated_ || parent->config_.pre_calibrated;
        // }
        // void set_phase_inductance(float value)
        // {
        //     phase_inductance = value;
        //     parent->update_current_controller_gains();
        // }
        // void set_phase_resistance(float value)
        // {
        //     phase_resistance = value;
        //     parent->update_current_controller_gains();
        // }
        // void set_current_control_bandwidth(float value)
        // {
        //     current_control_bandwidth = value;
        //     parent->update_current_controller_gains();
        // }
    };

    Motor(TIM_HandleTypeDef *timer,
          float shunt_conductance,
          TGateDriver &gate_driver,
          TOpAmp &opamp);

    bool setup();
    float max_available_torque();
    void update(uint32_t timestamp);
    void pwm_update_cb(uint32_t output_timestamp);
    void apply_pwm_timings(uint16_t timings[3], bool tentative);
    bool arm(PhaseControlLaw<3> *control_law);

    TIM_HandleTypeDef *const timer_;

    /** current limit */
    float effective_current_lim_ = 10.0f; // [A]
    float I_bus_ = 0.0f;                  // this motors contribution to the bus current

    /**  disarm motor */
    bool disarm(bool *p_was_armed);
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    bool is_armed_ = false;
    uint8_t armed_state_ = 0;

    /** gate driver DRV8323 */
    TGateDriver &gate_driver_;
    TOpAmp &opamp_;

    /** current sense */
    std::optional<float> phase_current_from_adcval(uint32_t ADCValue);
    void current_meas_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current);
    std::optional<Iph_ABC_t> current_meas_;
    const float shunt_conductance_;
    float phase_current_rev_gain_ = 0.0f; // Reverse gain for ADC to Amps (to be set by DRV8301_setup)
    uint32_t n_evt_current_measurement_ = 0;
    float max_allowed_current_ = 0.0f; // [A] set in setup()

    /** dc calib */
    bool is_calibrated_ = false; // Set in apply_config()
    void dc_calib_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current);
    Iph_ABC_t DC_calib_ = {0.0f, 0.0f, 0.0f};
    float dc_calib_running_since_ = 0.0f; // current sensor calibration needs some time to settle
    float max_dc_calib_ = 0.0f;           // [A] set in setup()

    InputPort<float> torque_setpoint_src_; // Usually points to the Controller object's output
    InputPort<float> phase_vel_src_;       // Usually points to the Encoder object's output

    float direction_ = 0.0f;                            // if -1 then positive torque is converted to negative Iq
    OutputPort<float2D> Vdq_setpoint_ = {{0.0f, 0.0f}}; // fed to the FOC
    OutputPort<float2D> Idq_setpoint_ = {{0.0f, 0.0f}}; // fed to the FOC

    FieldOrientedController current_control_;

    PhaseControlLaw<3> *control_law_;
    Config_t config_;
    Axis *axis_ = nullptr; // set by Axis constructor
};

#endif // __MOTOR_HPP
