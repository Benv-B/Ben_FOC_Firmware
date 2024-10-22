#ifndef __STM32_GPIO_HPP
#define __STM32_GPIO_HPP

#include <gpio.h>

class Stm32Gpio
{
public:
    static const Stm32Gpio none;

    Stm32Gpio() : port_(nullptr), pin_mask_(0) {}
    Stm32Gpio(GPIO_TypeDef *port, uint16_t pin) : port_(port), pin_mask_(pin) {}

    operator bool() const { return port_ && pin_mask_; }

    bool config(uint32_t mode, uint32_t pull, uint32_t speed);

    void write(bool state)
    {
        if (port_)
        {
            HAL_GPIO_WritePin(port_, pin_mask_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }

    bool read()
    {
        return port_ && (port_->IDR & pin_mask_);
    }

    GPIO_TypeDef *port_;
    uint16_t pin_mask_; // TODO: store pin_number_ instead of pin_mask_
};

#endif // __STM32_GPIO_HPP
