
#include "AS5048A.hpp"
#if 0
void Drv8301::link_spi_class(SPIClass* spi)
{
    spi_class_ptr = spi;
}

bool Drv8301::init(GainSetting_e requested_gain) 
{
    /*********** SET CONTROL REGISTERS WITH THE REQUESTED GAIN VALUE **********/

    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    RegisterFile new_config;

    new_config.control_register_1 =
          (21 << 6) // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        | (0b01 << 4) // OCP_MODE: latch shut down
        | (0b0 << 3) // 6x PWM mode
        | (0b0 << 2) // don't reset latched faults
        | (0b00 << 0); // gate-drive peak current: 1.7A

    new_config.control_register_2 =
          (0b0 << 6) // OC_TOFF: cycle by cycle
        | (0b00 << 4) // calibration off (normal operation)
        | ((uint8_t)requested_gain << 2) // select gain
        | (0b00 << 0); // report both over temperature and over current on nOCTW pin


    /*********** WRITE CONTROL REGISTERS TO DRV8301 **********/

    uint16_t val;

    /** DRV8301 is MSB first.
     * 
     * Data is shifted out of the SDO pin on rising edge of clock,
     * and data is sampled in on falling edge of clock. Clock must be low
     * when CS goes from inactive high to active low. (CPOL 0, CPHA 1, SPI_MODE1)  */
    //spi_settings = SPISettings(SPI_CLOCK_SPEED_HZ_DRV, MSBFIRST, SPI_MODE1); // 
    spi_settings = SPISettings(SPI_CLOCK_SPEED_HZ_DRV, MSBFIRST, SPI_MODE1); // 

    // Reset DRV chip. The enable pin also controls the SPI interface, not only
    // the driver stages.

    /** THIS ASSUMES SIMPLE FOC DRIVER CLASS HAS BEEN INITIALIZED AND THE ENABLE
     * PIN IS PASSES AS A DRIVER CONSTRUCTOR PARAMETER!!!! */
    digitalWrite(EN_gpio_, LOW);
    delayMicroseconds(40); // mimumum pull-down time for full reset: 20us
    digitalWrite(EN_gpio_, HIGH);
    delayMicroseconds(40);

    // Write current configuration
    bool wrote_regs = write_reg(kRegNameControl1, new_config.control_register_1)
                       && write_reg(kRegNameControl1, new_config.control_register_1)
                       && write_reg(kRegNameControl1, new_config.control_register_1)
                       && write_reg(kRegNameControl1, new_config.control_register_1)
                       && write_reg(kRegNameControl1, new_config.control_register_1) // the write operation tends to be ignored if only done once (not sure why)
                       && write_reg(kRegNameControl2, new_config.control_register_2);
    if (!wrote_regs) {
        return false;
    }

    // Wait for configuration to be applied
    delayMicroseconds(100);
    
    bool is_read_regs = read_reg(kRegNameControl1, &val) && (val == new_config.control_register_1)
                      && read_reg(kRegNameControl2, &val) && (val == new_config.control_register_2);
    if (!is_read_regs) {
        return false;
    }

#if 0
    if (get_error() != FaultType_NoFault) {
        return false;
    }
#endif 

    return true;
}

void Drv8301::do_checks() 
{
    
}

bool Drv8301::is_ready() 
{
    return 0;
}

Drv8301::FaultType_e Drv8301::get_error() {
    uint16_t fault1, fault2;

    if (!read_reg(kRegNameStatus1, &fault1) ||
        !read_reg(kRegNameStatus2, &fault2)) {
        return (FaultType_e)0xffffffff;
    }

    return (FaultType_e)((uint32_t)fault1 | ((uint32_t)(fault2 & 0x0080) << 16));
}

bool Drv8301::read_reg(const RegName_e regName, uint16_t* data) 
{
    tx_buf_ = build_addr_word(DRV8301_CtrlMode_Read, regName, 0);

    spi_class_ptr->beginTransaction(spi_settings);

    digitalWrite(nCS_gpio_, LOW);
    (void)spi_class_ptr->transfer16(tx_buf_);
    digitalWrite(nCS_gpio_, HIGH);

    delayMicroseconds(100);

    tx_buf_ = build_addr_word(DRV8301_CtrlMode_Read, regName, 0);

    digitalWrite(nCS_gpio_, LOW);
    rx_buf_ = spi_class_ptr->transfer16(tx_buf_);
    digitalWrite(nCS_gpio_, HIGH);

    spi_class_ptr->endTransaction();

#if 0 /** this is from latest open source odrive firmware, what is this?  */
    if (rx_buf_ == 0xbeef) 
    {
        return false;
    }
#endif 

    if (data) 
        *data = rx_buf_ & 0x07FF;
    else
        return false;


    delayMicroseconds(400);
    
    return true;
}

bool Drv8301::write_reg(const RegName_e regName, const uint16_t data) 
{
    tx_buf_ = build_addr_word(DRV8301_CtrlMode_Write, regName, data);

    spi_class_ptr->beginTransaction(spi_settings);

    digitalWrite(nCS_gpio_, LOW);
    (void)spi_class_ptr->transfer16(tx_buf_);
    digitalWrite(nCS_gpio_,HIGH);

    spi_class_ptr->endTransaction();
    
    delayMicroseconds(400);

    return true;
}

#endif
