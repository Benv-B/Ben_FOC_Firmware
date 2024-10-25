#ifndef __INTERFACES_HPP
#define __INTERFACES_HPP

#include <tuple>
#include <limits>
#include <functional>
#include <unordered_map>
#include <stdlib.h>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

using float2D = std::pair<float, float>;
struct Iph_ABC_t
{
    float phA;
    float phB;
    float phC;
};

class BenDrive_Intf
{
public:
    class Motor_Intf
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
    };
    class Axis_Intf
    {
    public:
        enum Error
        {
            ERROR_NONE = 0x00000000,
            ERROR_INVALID_STATE = 0x00000001,
            ERROR_MOTOR_FAILED = 0x00000040,
            ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x00000080,
            ERROR_ENCODER_FAILED = 0x00000100,
            ERROR_CONTROLLER_FAILED = 0x00000200,
            ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000800,
            ERROR_MIN_ENDSTOP_PRESSED = 0x00001000,
            ERROR_MAX_ENDSTOP_PRESSED = 0x00002000,
            ERROR_ESTOP_REQUESTED = 0x00004000,
            ERROR_HOMING_WITHOUT_ENDSTOP = 0x00020000,
            ERROR_OVER_TEMP = 0x00040000,
            ERROR_UNKNOWN_POSITION = 0x00080000,
        };
    };
};

// this is technically not thread-safe but practically it might be
inline BenDrive_Intf::Axis_Intf::Error operator|(BenDrive_Intf::Axis_Intf::Error a, BenDrive_Intf::Axis_Intf::Error b) { return static_cast<BenDrive_Intf::Axis_Intf::Error>(static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(a) | static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(b)); }
inline BenDrive_Intf::Axis_Intf::Error operator&(BenDrive_Intf::Axis_Intf::Error a, BenDrive_Intf::Axis_Intf::Error b) { return static_cast<BenDrive_Intf::Axis_Intf::Error>(static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(a) & static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(b)); }
inline BenDrive_Intf::Axis_Intf::Error operator^(BenDrive_Intf::Axis_Intf::Error a, BenDrive_Intf::Axis_Intf::Error b) { return static_cast<BenDrive_Intf::Axis_Intf::Error>(static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(a) ^ static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(b)); }
inline BenDrive_Intf::Axis_Intf::Error &operator|=(BenDrive_Intf::Axis_Intf::Error &a, BenDrive_Intf::Axis_Intf::Error b) { return reinterpret_cast<BenDrive_Intf::Axis_Intf::Error &>(reinterpret_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error> &>(a) |= static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(b)); }
inline BenDrive_Intf::Axis_Intf::Error &operator&=(BenDrive_Intf::Axis_Intf::Error &a, BenDrive_Intf::Axis_Intf::Error b) { return reinterpret_cast<BenDrive_Intf::Axis_Intf::Error &>(reinterpret_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error> &>(a) &= static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(b)); }
inline BenDrive_Intf::Axis_Intf::Error &operator^=(BenDrive_Intf::Axis_Intf::Error &a, BenDrive_Intf::Axis_Intf::Error b) { return reinterpret_cast<BenDrive_Intf::Axis_Intf::Error &>(reinterpret_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error> &>(a) ^= static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(b)); }
inline BenDrive_Intf::Axis_Intf::Error operator~(BenDrive_Intf::Axis_Intf::Error a) { return static_cast<BenDrive_Intf::Axis_Intf::Error>(~static_cast<std::underlying_type_t<BenDrive_Intf::Axis_Intf::Error>>(a)); }
// this is technically not thread-safe but practically it might be
inline BenDrive_Intf::Motor_Intf::Error operator|(BenDrive_Intf::Motor_Intf::Error a, BenDrive_Intf::Motor_Intf::Error b) { return static_cast<BenDrive_Intf::Motor_Intf::Error>(static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(a) | static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(b)); }
inline BenDrive_Intf::Motor_Intf::Error operator&(BenDrive_Intf::Motor_Intf::Error a, BenDrive_Intf::Motor_Intf::Error b) { return static_cast<BenDrive_Intf::Motor_Intf::Error>(static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(a) & static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(b)); }
inline BenDrive_Intf::Motor_Intf::Error operator^(BenDrive_Intf::Motor_Intf::Error a, BenDrive_Intf::Motor_Intf::Error b) { return static_cast<BenDrive_Intf::Motor_Intf::Error>(static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(a) ^ static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(b)); }
inline BenDrive_Intf::Motor_Intf::Error &operator|=(BenDrive_Intf::Motor_Intf::Error &a, BenDrive_Intf::Motor_Intf::Error b) { return reinterpret_cast<BenDrive_Intf::Motor_Intf::Error &>(reinterpret_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error> &>(a) |= static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(b)); }
inline BenDrive_Intf::Motor_Intf::Error &operator&=(BenDrive_Intf::Motor_Intf::Error &a, BenDrive_Intf::Motor_Intf::Error b) { return reinterpret_cast<BenDrive_Intf::Motor_Intf::Error &>(reinterpret_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error> &>(a) &= static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(b)); }
inline BenDrive_Intf::Motor_Intf::Error &operator^=(BenDrive_Intf::Motor_Intf::Error &a, BenDrive_Intf::Motor_Intf::Error b) { return reinterpret_cast<BenDrive_Intf::Motor_Intf::Error &>(reinterpret_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error> &>(a) ^= static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(b)); }
inline BenDrive_Intf::Motor_Intf::Error operator~(BenDrive_Intf::Motor_Intf::Error a) { return static_cast<BenDrive_Intf::Motor_Intf::Error>(~static_cast<std::underlying_type_t<BenDrive_Intf::Motor_Intf::Error>>(a)); }

#endif
