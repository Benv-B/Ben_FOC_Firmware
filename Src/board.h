#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "stm32_system.h"

 #ifdef __cplusplus
 #include "stm32_gpio.hpp"
 #include "stm32_spi_arbiter.hpp"
 #include "gate_driver.h"
 #include "drv8323.hpp"

 using TGateDriver = DRV8323;
 using TOpAmp = DRV8323;

 #include "motor.hpp"
 #include "encoder.hpp"
 #include "controller.hpp"
 #include "axis.hpp"

 extern Encoder ams_encoder;
 extern Motor motor;
 extern Controller foc_controller;
 extern Axis axis1;
 #endif

#define TIM_TIME_BASE TIM14

// The delta from the control loop timestamp to the current sense timestamp is
// exactly 0 for M0 and TIM1_INIT_COUNT for M1.
// #define MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA (TIM_1_8_PERIOD_CLOCKS / 2 + 1 * 128)
#define MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA (0)
 #define nCS_Pin GPIO_PIN_15
 #define nCS_GPIO_Port GPIOA
 #define nFAULT_Pin GPIO_PIN_2
 #define nFAULT_GPIO_Port GPIOD
 #define EN_GATE_Pin GPIO_PIN_12
 #define EN_GATE_GPIO_Port GPIOB

// Linear range of the DRV8301 opamp output: 0.3V...5.7V. We set the upper limit
// to 3.0V so that it's symmetric around the center point of 1.65V.
#define CURRENT_SENSE_MIN_VOLT 0.3f
#define CURRENT_SENSE_MAX_VOLT 3.0f

#define SHUNT_RESISTANCE (500e-6f)

// Period in [s]
#define CURRENT_MEAS_PERIOD ((float)2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1) / (float)TIM_1_8_CLOCK_HZ)
static const float current_meas_period = CURRENT_MEAS_PERIOD;
const float vbus_voltage = 12.0f;

 void system_init();
 bool board_init();

#endif