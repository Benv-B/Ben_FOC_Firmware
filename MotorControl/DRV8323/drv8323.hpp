#ifndef __DRV8323_HPP
#define __DRV8323_HPP

#include <stdbool.h>
#include <stdint.h>

#include "gate_driver.h"
#include "stm32_gpio.hpp"
#include "stm32_spi_arbiter.hpp"

class DRV8323 : public GateDriverBase, public OpAmpBase
{
public:
    typedef enum : uint32_t
    {
        FaultType_NoFault = (0 << 0), // No fault

        // Status Register 1
        FaultType_VDS_LC = (1 << 0),  // VDS overcurrent fault on the C low-side MOSFET
        FaultType_VDS_HC = (1 << 1),  // VDS overcurrent fault on the C high-side MOSFET
        FaultType_VDS_LB = (1 << 2),  // VDS overcurrent fault on the B low-side MOSFET
        FaultType_VDS_HB = (1 << 3),  // VDS overcurrent fault on the B high-side MOSFET
        FaultType_VDS_LA = (1 << 4),  // VDS overcurrent fault on the A low-side MOSFET
        FaultType_VDS_HA = (1 << 5),  // VDS overcurrent fault on the A high-side MOSFET
        FaultType_OTSD = (1 << 6),    // overtemperature shutdown
        FaultType_UVLO = (1 << 7),    // undervoltage lockout fault condition
        FaultType_GDF = (1 << 8),     // gate drive fault condition
        FaultType_VDS_OCP = (1 << 9), // DRV8323 VDS monitor overcurrent

        FaultType_FAULT = (1 << 10),

        // Status Register 2
        FaultType_VGS_LC = (1 << 16), // overcurrent on phase A sense amplifier
        FaultType_VGS_HC = (1 << 17), // overcurrent on phase B sense amplifier
        FaultType_VGS_LB = (1 << 18), // overcurrent on phase C sense amplifier
        FaultType_VGS_HB = (1 << 19), // overtemperature warning
        FaultType_VGS_LA = (1 << 20), // charge pump undervoltage fault condition
        FaultType_VGS_HA = (1 << 21), // gate drive fault on the A high-side MOSFET
        FaultType_CPUV = (1 << 22),   // gate drive fault on the A low-side MOSFET
        FaultType_OTW = (1 << 23),    // gate drive fault on the B high-side MOSFET
        FaultType_SC_OC = (1 << 24),  // gate drive fault on the B low-side MOSFET
        FaultType_SB_OC = (1 << 25),  // gate drive fault on the C high-side MOSFET
        FaultType_SA_OC = (1 << 26)   // gate drive fault on the C low-side MOSFET
    } FaultType_e;

    DRV8323(Stm32SpiArbiter *spi_arbiter, Stm32Gpio ncs_gpio,
            Stm32Gpio enable_gpio, Stm32Gpio nfault_gpio)
        : spi_arbiter_(spi_arbiter), ncs_gpio_(ncs_gpio),
          enable_gpio_(enable_gpio), nfault_gpio_(nfault_gpio) {}

    /**
     * @brief Prepares the gate driver's configuration.
     *
     * If the gate driver was in ready state and the new configuration is
     * different from the old one then the gate driver will exit ready state.
     *
     * In any case changes to the configuration only take effect with a call to
     * init().
     */
    bool config(float requested_gain, float *actual_gain);

    /**
     * @brief Initializes the gate driver to the configuration prepared with
     * config().
     *
     * Returns true on success or false otherwise (e.g. if the gate driver is
     * not connected or not powered or if config() was not yet called).
     */
    bool init();

    /**
     * @brief Monitors the nFAULT pin.
     *
     * This must be run at an interval of <8ms from the moment the init()
     * functions starts to run, otherwise it's possible that a temporary power
     * loss is missed, leading to unwanted register values.
     * In case of power loss the nFAULT pin can be low for as little as 8ms.
     */
    void do_checks();

    /**
     * @brief Returns true if and only if the DRV8323 chip is in an initialized
     * state and ready to do switching and current sensor opamp operation.
     */
    bool is_ready() final;

    /**
     * @brief This has no effect on this driver chip because the drive stages are
     * always enabled while the chip is initialized
     */
    bool set_enabled(bool enabled) final { return true; }

    FaultType_e get_error();

    float get_midpoint() final
    {
        return 0.5f; // [V]
    }

    float get_max_output_swing() final
    {
        return 1.35f / 1.65f; // +-1.35V, normalized from a scale of +-1.65V to +-0.5
    }

private:
    enum CtrlMode_e
    {
        DRV8323_CtrlMode_Read = 1 << 15, //!< Read Mode
        DRV8323_CtrlMode_Write = 0 << 15 //!< Write Mode
    };

    // Registers addresses
    enum RegName_e
    {
        Fault_Status = 0 << 11,
        VGS_Status = 1 << 11,
        Driver_Control = 2 << 11,
        Gate_Driver_HS = 3 << 11,
        Gate_Driver_LS = 4 << 11,
        OCP_Control = 5 << 11,
        CSA_Control = 6 << 11
    };

    struct RegisterFile
    {
        uint16_t Fault_Status_register;
        uint16_t VGS_Status_register;
        uint16_t Driver_Control_register;
        uint16_t Gate_Driver_HS_register;
        uint16_t Gate_Driver_LS_register;
        uint16_t OCP_Control_register;
        uint16_t CSA_Control_register;
    };

    static inline uint16_t build_ctrl_word(const CtrlMode_e ctrlMode,
                                           const RegName_e regName,
                                           const uint16_t data)
    {
        return ctrlMode | regName | (data & 0x07FF);
    }

    RegisterFile regs_; //!< Current configuration. If is_ready_ is
                        //!< true then this can be considered consistent
                        //!< with the actual file on the DRV8323 chip.

    /** @brief Reads data from a DRV8323 register */
    bool read_reg(const RegName_e regName, uint16_t *data);

    /** @brief Writes data to a DRV8323 register. There is no check if the write succeeded. */
    bool write_reg(const RegName_e regName, const uint16_t data);

    static const SPI_InitTypeDef spi_config_;
    Stm32SpiArbiter *spi_arbiter_;
    Stm32Gpio ncs_gpio_;
    Stm32Gpio enable_gpio_;
    Stm32Gpio nfault_gpio_;

    // We don't put these buffers on the stack because we place the stack in
    // a RAM section which cannot be used by DMA.
    uint16_t tx_buf_, rx_buf_;

    enum
    {
        kStateUninitialized,
        kStateStartupChecks,
        kStateReady,
    } state_ = kStateUninitialized;
};

#endif