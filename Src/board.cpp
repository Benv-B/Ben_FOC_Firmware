//
// Created by benv on 24-10-12.
//
#include "board.h"
#include "axis.hpp"
#include "ben_drive_main.h"

#include <stm32f405xx.h>

#define ControlLoop_IRQHandler OTG_HS_IRQHandler
#define ControlLoop_IRQn OTG_HS_IRQn

extern "C" void SystemClock_Config(void);

Stm32SpiArbiter spi3_arbiter{&hspi3};

DRV8323 gate_driver{
    &spi3_arbiter,
    {nCS_GPIO_Port, nCS_Pin},         // nCS
    {EN_GATE_GPIO_Port, EN_GATE_Pin}, // EN pin
    {nFAULT_GPIO_Port, nFAULT_Pin}    // nFAULT pin
};

Encoder ams_encoder{&spi3_arbiter};

Motor motor{&htim1,
            1 / SHUNT_RESISTANCE,
            gate_driver,
            gate_driver};

Controller foc_controller;

Axis axis1{osPriorityHigh,
           ams_encoder,
           foc_controller,
           motor};

void system_init()
{
    HAL_Init();

    SystemClock_Config();
}

bool board_init()
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_SPI3_Init();
    MX_UART4_Init();
    MX_CAN1_Init();

    HAL_NVIC_SetPriority(ControlLoop_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ControlLoop_IRQn);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    Stm32Gpio drv_enable_gpio = {EN_GATE_GPIO_Port, EN_GATE_Pin};

    // Reset both DRV chips. The enable pin also controls the SPI interface, not
    // only the driver stages.
    drv_enable_gpio.write(false);
    delay_us(40); // mimumum pull-down time for full reset: 20us
    drv_enable_gpio.write(true);
    delay_us(20000); // mimumum pull-down time for full reset: 20us

    return true;
}

static bool
fetch_and_reset_adcs(std::optional<Iph_ABC_t> *current)
{
    bool all_adcs_done = ((ADC2->SR & ADC_SR_JEOC) == ADC_SR_JEOC) && ((ADC3->SR & ADC_SR_JEOC) == ADC_SR_JEOC);
    if (!all_adcs_done)
    {
        return false;
    }

    // vbus_sense_adc_cb(ADC1->JDR1);

    if (gate_driver.is_ready())
    {
        std::optional<float> phB = motor.phase_current_from_adcval(ADC2->JDR1);
        std::optional<float> phC = motor.phase_current_from_adcval(ADC3->JDR1);
        if (phB.has_value() && phC.has_value())
        {
            *current = {-*phB - *phC, *phB, *phC};
        }
    }

    ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);
    ADC3->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);

    return true;
}

extern "C"
{
    volatile uint32_t timestamp_ = 0;
    volatile bool counting_down_ = false;

    /**
     * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
     */
    void TIM1_UP_TIM10_IRQHandler(void)
    {
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

        // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
        // If we are counting down, we just sampled in SVM vector 7, with zero current
        bool counting_down = TIM1->CR1 & TIM_CR1_DIR;

        bool timer_update_missed = (counting_down == counting_down_);
        if (timer_update_missed)
        {
            // TODO: add action
        }

        counting_down_ = counting_down;
        timestamp_ += TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1);

        if (!counting_down)
        {
            bdrv.sampling_cb();
            NVIC->STIR = ControlLoop_IRQn;
        }
        else
        {
            TIM1->CCR1 =
                TIM1->CCR2 =
                    TIM2->CCR3 = TIM_1_8_PERIOD_CLOCKS / 2;
        }
    }

    void ControlLoop_IRQHandler()
    {
        uint32_t timestamp = timestamp_;

        std::optional<Iph_ABC_t> current;

        if (fetch_and_reset_adcs(&current))
        {
            motor.disarm();
        }

        if (!(TIM1->BDTR & TIM_BDTR_MOE_Msk))
        {
            current = {0.0f, 0.0f, 0.0f};
        }

        motor.current_meas_cb(timestamp, current);

        bdrv.control_loop_cb(timestamp);

        // By this time the ADCs for both M0 and M1 should have fired again. But
        // let's wait for them just to be sure.
        while (!(ADC2->SR & ADC_SR_EOC))
            ;

        if (fetch_and_reset_adcs(&current))
        {
            motor.disarm();
        }

        motor.dc_calib_cb(timestamp + TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1), current);

        motor.pwm_update_cb(timestamp + 3 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1));

        // If we did everything right, the TIM8 update handler should have been
        // called exactly once between the start of this function and now.

        if (timestamp_ != timestamp + TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1))
        {
            // motor.disarm_with_error(Motor::ERROR_CONTROL_DEADLINE_MISSED);
            motor.disarm();
        }
    }
}
