#include "drv8323.hpp"
#include "utils.hpp"
#include "cmsis_os.h"
#include "board.h"

const SPI_InitTypeDef DRV8323::spi_config_ = {
    .Mode = SPI_MODE_MASTER,
    .Direction = SPI_DIRECTION_2LINES,
    .DataSize = SPI_DATASIZE_16BIT,
    .CLKPolarity = SPI_POLARITY_LOW,
    .CLKPhase = SPI_PHASE_2EDGE,
    .NSS = SPI_NSS_SOFT,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
    .FirstBit = SPI_FIRSTBIT_MSB,
    .TIMode = SPI_TIMODE_DISABLE,
    .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .CRCPolynomial = 10,
};

bool DRV8323::config(float requested_gain, float *actual_gain)
{
    // Calculate gain setting: Snap down to have equal or larger range as
    // requested or largest possible range otherwise

    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    uint16_t gain_setting = 3;
    float gain_choices[] = {5.0f, 10.0f, 20.0f, 40.0f};
    while (gain_setting && (gain_choices[gain_setting] > requested_gain))
    {
        gain_setting--;
    }

    if (actual_gain)
    {
        *actual_gain = gain_choices[gain_setting];
    }

    RegisterFile new_config;

    new_config.Driver_Control_register =
        (0b0 << 10)   // Reserved
        | (0b0 << 9)  // Charge pump UVLO fault is enabled
        | (0b0 << 8)  // Gate drive fault is enabled
        | (0b0 << 7)  // OTW is not reported on nFAULT or the FAULT bit
        | (0b00 << 5) // 6x PWM mode
        | (0b0 << 4)  // 0b = 1x PWM mode uses synchronous rectification
        | (0b0 << 3)  // in 1x PWM mode
        | (0b0 << 2)  // Write a 1 to this bit to put all MOSFETs in the Hi-Z state
        | (0b0 << 1)  // in 1x PWM mode
        | (0b1 << 0); // Write a 1 to this bit to clear latched fault bits. This bit automatically resets after being written.

    new_config.Gate_Driver_HS_register =
        (0b011 << 8)     // unlock all registers
        | (0b1111 << 4)  // IDRIVEP_HS = 1000mA
        | (0b1111 << 0); // IDRIVEN_HS = 1000mA

    new_config.Gate_Driver_LS_register =
        (0b1) << 10      // Cycle-by cycle operation
        | (0b11 << 8)    // 4000-ns peak gate-current drive time
        | (0b1111 << 4)  // IDRIVEP_LS = 1000mA
        | (0b1111 << 0); // IDRIVEN_LS = 1000mA

    new_config.OCP_Control_register =
        (0b0 << 10)      // VDS_OCP and SEN_OCP retry time is 4 ms
        | (0b01 << 8)    // MOSFET 100-ns dead time
        | (0b01 << 6)    // Overcurrent causes an automatic retrying fault
        | (0b01 << 4)    // Overcurrent deglitch time of 4 µs
        | (0b1001 << 0); // VDS_LVL = 0.75 V

    new_config.CSA_Control_register =
        (0b0 << 10)           // Current sense amplifier positive input is SPx
        | (0b1 << 9)          // Current sense amplifier reference voltage is VREF divided by 2
        | (0b0 << 8)          // VDS_OCP for the low-side MOSFET is measured across SHx to SPx
        | (gain_setting << 6) // 20-V/V current sense amplifier gain
        | (0b0 << 5)          // Sense overcurrent fault is enabled
        | (0b0 << 4)          // Normal current sense amplifier A operation, 1 for calibration
        | (0b0 << 3)          // Normal current sense amplifier B operation, 1 for calibration
        | (0b0 << 2)          // Normal current sense amplifier C operation, 1 for calibration
        | (0b10 << 0);        // Sense OCP 0.75 V

    bool regs_equal = (regs_.Driver_Control_register == new_config.Driver_Control_register)    // Driver_Control_register
                      && (regs_.Gate_Driver_HS_register == new_config.Gate_Driver_HS_register) // Gate_Driver_HS_register
                      && (regs_.Gate_Driver_LS_register == new_config.Gate_Driver_LS_register) // Gate_Driver_LS_register
                      && (regs_.OCP_Control_register == new_config.OCP_Control_register)       // OCP_Control_register
                      && (regs_.CSA_Control_register == new_config.CSA_Control_register);      // CSA_Control_register

    if (!regs_equal)
    {
        regs_ = new_config;
        state_ = kStateUninitialized;
        enable_gpio_.write(false);
    }

    return true;
}

bool DRV8323::init()
{
    uint16_t val;

    if (state_ == kStateReady)
    {
        return true;
    }

    // Reset DRV chip. The enable pin also controls the SPI interface, not only
    // the driver stages.
    enable_gpio_.write(false);
    delay_us(40);                 // mimumum pull-down time for full reset: 20us
    state_ = kStateUninitialized; // make is_ready() ignore transient errors before registers are set up
    enable_gpio_.write(true);
    osDelay(20); // t_spi_ready, max = 10ms

    // Write current configuration
    bool wrote_regs = write_reg(Driver_Control, regs_.Driver_Control_register)    // the write operation
                      && write_reg(Driver_Control, regs_.Driver_Control_register) // the write operation
                      && write_reg(Driver_Control, regs_.Driver_Control_register) // the write operation
                      && write_reg(Driver_Control, regs_.Driver_Control_register) // the write operation
                      && write_reg(Driver_Control, regs_.Driver_Control_register) // the write operation
                      && write_reg(Gate_Driver_HS, regs_.Gate_Driver_HS_register) // the write operation
                      && write_reg(Gate_Driver_LS, regs_.Gate_Driver_LS_register) // the write operation
                      && write_reg(OCP_Control, regs_.OCP_Control_register)       // the write operation
                      && write_reg(CSA_Control, regs_.CSA_Control_register);      // the write operation tends to be ignored if only done once (not sure why)
    if (!wrote_regs)
    {
        return false;
    }

    // Wait for configuration to be applied
    delay_us(100);
    state_ = kStateStartupChecks;

    bool is_read_regs = read_reg(Driver_Control, &val) && (val == regs_.Driver_Control_register)    // Driver_Control
                        && read_reg(Gate_Driver_HS, &val) && (val == regs_.Gate_Driver_HS_register) // Driver_Control
                        && read_reg(Gate_Driver_LS, &val) && (val == regs_.Gate_Driver_LS_register) // Driver_Control
                        && read_reg(OCP_Control, &val) && (val == regs_.OCP_Control_register)       // Driver_Control
                        && read_reg(CSA_Control, &val) && (val == regs_.CSA_Control_register);
    if (!is_read_regs)
    {
        return false;
    }

    if (get_error() != FaultType_NoFault)
    {
        return false;
    }

    // There could have been an nFAULT edge meanwhile. In this case we shouldn't
    // consider the driver ready.
    CRITICAL_SECTION()
    {
        if (state_ == kStateStartupChecks)
        {
            state_ = kStateReady;
        }
    }

    return state_ == kStateReady;
}

void DRV8323::do_checks()
{
    if (state_ != kStateUninitialized && !nfault_gpio_.read())
    {
        state_ = kStateUninitialized;
    }
}

bool DRV8323::is_ready()
{
    return state_ == kStateReady;
}

DRV8323::FaultType_e DRV8323::get_error()
{
    uint16_t fault1, fault2;

    if (!read_reg(Fault_Status, &fault1) ||
        !read_reg(VGS_Status, &fault2))
    {
        return (FaultType_e)0xffffffff;
    }

    return (FaultType_e)((uint32_t)fault1 | ((uint32_t)fault2 << 16));
}

bool DRV8323::read_reg(const RegName_e regName, uint16_t *data)
{
    tx_buf_ = build_ctrl_word(DRV8323_CtrlMode_Read, regName, 0);
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000))
    {
        return false;
    }

    delay_us(1);

    tx_buf_ = build_ctrl_word(DRV8323_CtrlMode_Read, regName, 0);
    rx_buf_ = 0xffff;
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 1, 1000))
    {
        return false;
    }

    delay_us(1);

    if (rx_buf_ == 0xbeef)
    {
        return false;
    }

    if (data)
    {
        *data = rx_buf_ & 0x07FF;
    }

    return true;
}

bool DRV8323::write_reg(const RegName_e regName, const uint16_t data)
{
    // Do blocking write
    tx_buf_ = build_ctrl_word(DRV8323_CtrlMode_Write, regName, data);
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000))
    {
        return false;
    }
    delay_us(1);

    return true;
}
