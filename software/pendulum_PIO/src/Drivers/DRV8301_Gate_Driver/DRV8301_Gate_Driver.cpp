
#include "DRV8301_Gate_Driver.hpp"
#include "Arduino.h"


/** NOTE: This file has been adapted from the DRV8301 driver found 
 * in the most recent version of the ODrive 3.5 open source software. */ 

/** The MOSFET's that are used on the ODESC 4.2 have a part 
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

/** The nFAULT pin is active low: */
#define nFAULT_PIN_FAULTED      0
#define nFAULT_PIN_UNFAULTED    1

void DRV8301_Gate_Driver::fault_pin_asserted_isr_callback(void)
{
    nFAULT_pin_level = digitalRead(nFAULT_gpio_);

    /** only check for faults if the device is enabled, 
     * otherwise the errors will all be falsely active
     * simply because the gate driver is off. */
    if(EN_pin_controlled_level == HIGH)
    {
        has_nFAULT_tripped = true;
        error_code_when_nFAULT_tripped = get_error();

        /** Set the enable pin LOW since we faulted  */
        digitalWrite(EN_gpio_, LOW);
        EN_pin_controlled_level = LOW;
    }
    else
    {
        /** do nothing, somewhere else in the code we disabled
         * the gate driver causing the nFAULT pin to go LOW/asserted. */
    }
}






bool DRV8301_Gate_Driver::init(DRV8301_GainSetting_e requested_gain, SPIClass* spi, void (*handle_nFAULT)()) 
{
    uint16_t val;

    /*********** LINK SPI CLASS **********/
    spi_class_ptr = spi;

    /*********** SET CONTROL REGISTERS WITH THE REQUESTED GAIN VALUE **********/
    RegisterFile tmp_new_control_reg_config;

    /** Datasheet Section 7.6.3.2 */

    tmp_new_control_reg_config.control_register_1 =
          (OC_ADJ_SET_100A_25C << 6) // See macro definition for more info 
        | (0b01 << 4) // OCP_MODE: latch shut down
        | (0b0 << 3) // 6x PWM mode
        | (0b0 << 2) // don't reset latched faults
        | (0b00 << 0); // gate-drive peak current: 1.7A

    tmp_new_control_reg_config.control_register_2 =
          (0b0 << 6) // OC_TOFF: cycle by cycle
        | (0b00 << 4) // calibration off (normal operation)
        | ((uint8_t)requested_gain << 2) // select gain
        | (0b00 << 0); // report both over temperature and over current on nOCTW pin

    pinMode(nCS_gpio_, OUTPUT);
    digitalWrite(nCS_gpio_, HIGH);

    pinMode(EN_gpio_, OUTPUT);
    digitalWrite(EN_gpio_, LOW);
    
    pinMode(nFAULT_gpio_, INPUT);

    /*********** WRITE CONTROL REGISTERS TO DRV8301 **********/

    /** DRV8301 is MSB first.
     * Data is shifted out of the SDO pin on rising edge of clock,
     * and data is sampled in on falling edge of clock. Clock must be low
     * when CS goes from inactive high to active low. (CPOL 0, CPHA 1, SPI_MODE1)  */
    spi_settings = SPISettings(SPI_CLOCK_SPEED_HZ_DRV, MSBFIRST, SPI_MODE1); // 

    /** Reset DRV chip. The enable pin also controls 
     * the SPI interface, not only the driver stages. */
    digitalWrite(EN_gpio_, LOW);

    /** Mimumum pull-down time for full reset: 20us (Datasheet Section 7.4.1)
     * Double that for a reasonable amount of margin.  */
    delayMicroseconds(40); 
    
    enable(); // enable the gate driver.

    /** As per datasheet section 6.8, the time it takes for SPI to be ready after EN_GATE
     * transitions to HIGH is 5-10ms.  When reading SPI too soon after EN_GATE is driven high,
     * the DRV8301's SDO output to the MCU will show that some of the faults have not cleared 
     * yet.  This is likely why the original ODrive devs thought that you had to write to the 
     * register multiple times to take effect. */
    delayMicroseconds(10000);

    // Write current configuration
    write_reg(kRegAddrControl1, tmp_new_control_reg_config.control_register_1);
    write_reg(kRegAddrControl2, tmp_new_control_reg_config.control_register_2);

    // Wait for configuration to be applied
    delayMicroseconds(100);
    
    bool is_read_regs = read_reg(kRegAddrControl1, &val) && (val == tmp_new_control_reg_config.control_register_1)
                      && read_reg(kRegAddrControl2, &val) && (val == tmp_new_control_reg_config.control_register_2);
    if (!is_read_regs) {
        return false;
    }

    nFAULT_pin_level = digitalRead(nFAULT_gpio_);

    if (get_error() != DRV8301_FaultType_NoFault
    || nFAULT_pin_level != nFAULT_PIN_UNFAULTED
    || handle_nFAULT == nullptr) 
    {
        if(nFAULT_pin_level != nFAULT_PIN_UNFAULTED)
            has_nFAULT_tripped = true;

        EN_pin_controlled_level = LOW;

        /** Disable the device and return 0 */    
        digitalWrite(EN_gpio_, LOW);

        return false;
    }
    else
    {
        // attach interrupt to nFAULT pin
        attachInterrupt(nFAULT_gpio_, handle_nFAULT, LOW);
    }

    return true;
}



void DRV8301_Gate_Driver::enable() 
{
    if(has_nFAULT_tripped == false)
    {

        EN_pin_controlled_level = HIGH;
        digitalWrite(EN_gpio_, HIGH);
        nFAULT_pin_level = digitalRead(nFAULT_gpio_);
    }
    else
    {
        /** Do nothing, don't enable the gate driver 
         * if we have previously faulted.
         */
    }
}



void DRV8301_Gate_Driver::disable() 
{
    /** Set the "EN_pin_controlled_level" variable
     * before controlling the pin in case the external interrupt
     * configured to trip on nFAULT sets immediately after disabling the
     * device.  This way, if that happens, the EN_pin_controlled_level
     * will already be set and we'll know if nFAULT is only asserted active
     * since the device is disabled. */
    EN_pin_controlled_level = LOW;
    digitalWrite(EN_gpio_, LOW);
    nFAULT_pin_level = digitalRead(nFAULT_gpio_);
}



DRV8301_FaultType_e DRV8301_Gate_Driver::get_error() 
{
    uint16_t fault1, fault2;

    if (!read_reg(kRegAddrStatus1, &fault1) ||
        !read_reg(kRegAddrStatus2, &fault2)) {
        return (DRV8301_FaultType_e)0xffffffff;
    }

    return (DRV8301_FaultType_e)((uint32_t)fault1 | ((uint32_t)(fault2 & 0x0080) << 16));
}



bool DRV8301_Gate_Driver::read_reg(const RegAddr_e regName, uint16_t* data) 
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
    
    // there's no real error management with the Arduino SPI library so just return true
    return true;
}




bool DRV8301_Gate_Driver::write_reg(const RegAddr_e regName, const uint16_t data) 
{
    tx_buf_ = build_ctrl_word(DRV8301_CtrlMode_Write, regName, data);

    spi_class_ptr->beginTransaction(spi_settings);

    digitalWrite(nCS_gpio_, LOW);
    (void)spi_class_ptr->transfer16(tx_buf_);
    digitalWrite(nCS_gpio_,HIGH);

    spi_class_ptr->endTransaction();
    
    delayMicroseconds(400);

    // there's no real error management with the Arduino SPI library so just return true
    return true;
}
