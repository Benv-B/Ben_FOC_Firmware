#ifndef __ODRIVE_MAIN_H
#define __ODRIVE_MAIN_H

// Hardware configuration
#include "board.h"

// #define IB_Pin GPIO_PIN_0
// #define IB_GPIO_Port GPIOC
// #define IC_Pin GPIO_PIN_1
// #define IC_GPIO_Port GPIOC

// general system functions defined in main.cpp
class BenDrive
{
public:
    BenDrive(Encoder &encoder) : AMSEncoder(encoder) {}

    void sampling_cb();
    void control_loop_cb(uint32_t timestamp);

    Encoder &AMSEncoder;

    uint32_t last_update_timestamp_ = 0;
    uint32_t n_evt_control_loop_ = 0;
};

extern BenDrive bdrv; // defined in main.cpp

#endif // __cplusplus

#endif /* __ODRIVE_MAIN_H */
