#define __MAIN_CPP__
#include "ben_drive_main.h"

const uint32_t stack_size_default_task = 2048; // Bytes
extern osThreadId defaultTaskHandle;

BenDrive bdrv{};

void BenDrive::sampling_cb()
{
    axis1.encoder_.sample_now();
}

/**
 * @brief Runs the periodic control loop.
 *
 * This function is executed in a low priority interrupt context and is allowed
 * to call CMSIS functions.
 *
 * Yet it runs at a higher priority than communication workloads.
 *
 * @param update_cnt: The true count of update events (wrapping around at 16
 *        bits). This is used for timestamp calculation in the face of
 *        potentially missed timer update interrupts. Therefore this counter
 *        must not rely on any interrupts.
 */
void BenDrive::control_loop_cb(uint32_t timestamp)
{
    last_update_timestamp_ = timestamp;
    n_evt_control_loop_++;

    // TODO : set uart
    // uart_poll();

    ams_encoder.update();

    // Controller of either axis might use the encoder estimate of the other
    // axis so we process both encoders before we continue.
    foc_controller.update();

    motor.update(timestamp); // uses torque from controller and phase_vel from encoder

    motor.current_control_.update(timestamp); // uses the output of controller_ or open_loop_contoller_ and encoder_ or sensorless_estimator_ or acim_estimator_

    // Tell the axis threads that the control loop has finished
    // for (auto &axis : axes)
    // {
    //     if (axis.thread_id_)
    //     {
    //         osSignalSet(axis.thread_id_, 0x0001);
    //     }
    // }

    // get_gpio(odrv.config_.error_gpio_pin).write(odrv.any_error());
}

static void rtos_main(const void *)
{
    // Init USB device
    MX_USB_DEVICE_Init();

    // TODO: init communication

    // pull encoder spi high
    axis1.encoder_.abs_spi_cs_pin_init();

    axis1.motor_.setup();

    axis1.encoder_.setup();

    // Wait for up to 2s for motor to become ready to allow for error-free
    // startup. This delay gives the current sensor calibration time to
    // converge. If the DRV chip is unpowered, the motor will not become ready
    // but we still enter idle state.
    for (size_t i = 0; i < 2000; ++i)
    {
        bool motors_ready = axis1.motor_.current_meas_.has_value();
        if (motors_ready)
        {
            break;
        }
        osDelay(1);
    }

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    // TODO: generalize for AXIS_COUNT != 2
    axis1.start_thread();

    // odrv.system_stats_.fully_booted = true;

    // Main thread finished starting everything and can delete itself now (yes this is legal).
    vTaskDelete(defaultTaskHandle);
}

extern "C" int main()
{
    system_init();

    if (!board_init())
    {
        for (;;)
            ;
    }

    osThreadDef(defaultTask, rtos_main, osPriorityNormal, 0, stack_size_default_task / sizeof(StackType_t));
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // Start scheduler
    osKernelStart();

    for (;;)
        ;
}