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

#ifdef __cplusplus
#include "my_Drivers/stm32_gpio.hpp"
#include "my_Drivers/stm32_spi_arbiter.hpp"
#include "MotorControl/DRV8323/gate_driver.h"

using TGateDriver = Drv8301;
using TOpAmp = Drv8301;

#include "MotorControl/motor.hpp"
#include "MotorControl/encoder.hpp"

extern Encoder ams_encoder;
extern Motor motor;
extern Controller foc_controller;
extern Axis axis1;
#endif

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