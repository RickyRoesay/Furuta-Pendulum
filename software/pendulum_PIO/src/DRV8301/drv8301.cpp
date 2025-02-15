
#include "drv8301.hpp"


/** NOTE: This file has been adapted from the DRV8301 driver found 
 * in the most recent version of the ODrive 3.5 open source software. */ 

/** The cheapo MOSFET's that are used on the ODESC 4.2 have a part 
 * number of NTMFS5C628NL.  These MOSFET's have an Rds ON of about 
 * 2 mOhms at a Gate-Source voltage of 10V, according to figure 3
 * of the datasheet.  (Gate drive power supply on the DRV8301 is ~11V, 
 * but figure 3 doesn't go up that high in Vgs.)
 * 
 * Assuming a 2 mOhms of Rds ON, and an overcurrent fault chosen arbitrarily
 * to be 100A, that gives a Vds of 0.2V.  
 * 
 * This gives an overcurrent value of 10 (0.197V)
 * (DRV Datasheet Section 7.6.3.3) */
#define OC_ADJ_SET_100A_25C 10

void Drv8301::link_spi_class(SPIClass* spi)
{
    spi_class_ptr = spi;
}

bool Drv8301::init(DRV8301_GainSetting_e requested_gain) 
{
    /*********** SET CONTROL REGISTERS WITH THE REQUESTED GAIN VALUE **********/

    // for reference:
    // 10V/V on 500uOhm gives a range of +/- 300A
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 80V/V on 500uOhm gives a range of +/- 37.5A
    // 10V/V on 10 mOhm gives a range of +/- 15A
    // 20V/V on 10 mOhm gives a range of +/- 7.5A

    RegisterFile new_config;

    /** Datasheet Section 7.6.3.2 */

    new_config.control_register_1 =
          (OC_ADJ_SET_100A_25C << 6) // See macro definition for more info 
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
    spi_settings = SPISettings(SPI_CLOCK_SPEED_HZ_DRV, MSBFIRST, SPI_MODE1); // 

    // Reset DRV chip. The enable pin also controls the SPI interface, not only
    // the driver stages.

    digitalWrite(EN_gpio_, LOW);
    delayMicroseconds(40); // mimumum pull-down time for full reset: 20us
    digitalWrite(EN_gpio_, HIGH);
    delayMicroseconds(40);

    // Write current configuration

    /** "the write operation tends to be ignored if only 
     * done once (not sure why)" -odrive devs 
     * 
     * This may have been specific to their 
     * code base/board so I'm going to test that out for myself. 
     * 
     * EDIT: They were totally right :) the initial write does NOT take effect
     * if it's only written once. I have not spent the time to dig any deeper on a 
     * root cause/explaination.  This may even just be quirk of this IC. */
    bool wrote_regs = write_reg(kRegAddrControl1, new_config.control_register_1)
                    #if 1
                    && write_reg(kRegAddrControl1, new_config.control_register_1)
                    && write_reg(kRegAddrControl1, new_config.control_register_1)
                    && write_reg(kRegAddrControl1, new_config.control_register_1)
                    && write_reg(kRegAddrControl1, new_config.control_register_1) 
                    #endif
                    && write_reg(kRegAddrControl2, new_config.control_register_2);
    if (!wrote_regs) {
        return false;
    }

    // Wait for configuration to be applied
    delayMicroseconds(100);
    
    bool is_read_regs = read_reg(kRegAddrControl1, &val) && (val == new_config.control_register_1)
                      && read_reg(kRegAddrControl2, &val) && (val == new_config.control_register_2);
    if (!is_read_regs) {
        return false;
    }

    if (get_error() != DRV8301_FaultType_NoFault) {
        return false;
    }

    return true;
}

DRV8301_FaultType_e Drv8301::get_error() {
    uint16_t fault1, fault2;

    if (!read_reg(kRegAddrStatus1, &fault1) ||
        !read_reg(kRegAddrStatus2, &fault2)) {
        return (DRV8301_FaultType_e)0xffffffff;
    }

    return (DRV8301_FaultType_e)((uint32_t)fault1 | ((uint32_t)(fault2 & 0x0080) << 16));
}

bool Drv8301::read_reg(const RegAddr_e regName, uint16_t* data) 
{
    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Read, regName, 0);

    spi_class_ptr->beginTransaction(spi_settings);

    digitalWrite(nCS_gpio_, LOW);
    (void)spi_class_ptr->transfer16(tx_buf_);
    digitalWrite(nCS_gpio_, HIGH);

    delayMicroseconds(100);

    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Read, regName, 0);

    digitalWrite(nCS_gpio_, LOW);
    rx_buf_ = spi_class_ptr->transfer16(tx_buf_);
    digitalWrite(nCS_gpio_, HIGH);

    spi_class_ptr->endTransaction();

    if (data) 
        *data = rx_buf_ & 0x07FF;
    else
        return false;


    delayMicroseconds(400);
    
    return true;
}

bool Drv8301::write_reg(const RegAddr_e regName, const uint16_t data) 
{
    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Write, regName, data);

    spi_class_ptr->beginTransaction(spi_settings);

    digitalWrite(nCS_gpio_, LOW);
    (void)spi_class_ptr->transfer16(tx_buf_);
    digitalWrite(nCS_gpio_,HIGH);

    spi_class_ptr->endTransaction();
    
    delayMicroseconds(400);

    return true;
}
