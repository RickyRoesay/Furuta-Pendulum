#ifndef __DRV8301_HPP
#define __DRV8301_HPP

#include "Arduino.h"
#include <SPI.h>

/** NOTE: This file has been adapted from the DRV8301 driver found 
 * in the most recent version of the ODrive 3.5 open source software. 
 */

/** Datasheet Section 7.6.3.2 */
typedef enum : uint16_t {
    DRV8301_GainSetting_10_V_over_V  = 0, 
    DRV8301_GainSetting_20_V_over_V  = 1, 
    DRV8301_GainSetting_40_V_over_V  = 2, 
    DRV8301_GainSetting_80_V_over_V  = 3, 
} DRV8301_GainSetting_e;

typedef enum : uint32_t {
    DRV8301_FaultType_NoFault  = (0 << 0),  //!< No fault

    // Status Register 1
    DRV8301_FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
    DRV8301_FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
    DRV8301_FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
    DRV8301_FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
    DRV8301_FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
    DRV8301_FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
    DRV8301_FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
    DRV8301_FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
    DRV8301_FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
    DRV8301_FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
    DRV8301_FaultType_FAULT    = (1 << 10),

    // Status Register 2
    DRV8301_FaultType_GVDD_OV  = (1 << 23)  //!< DRV8301 Vdd Over Voltage fault
} DRV8301_FaultType_e;

class Drv8301 {
public:

    Drv8301(int nCS_gpio, int EN_gpio, int nFAULT_gpio) : nCS_gpio_(nCS_gpio), EN_gpio_(EN_gpio), nFAULT_gpio_(nFAULT_gpio) {}

    void link_spi_class(SPIClass* spi);
    
    bool init(DRV8301_GainSetting_e requested_gain);

    DRV8301_FaultType_e get_error();

private:
    enum CtrlMode_e {
        DRV8301_CtrlMode_Read = 1 << 15,   //!< Read Mode
        DRV8301_CtrlMode_Write = 0 << 15   //!< Write Mode
    };

    enum RegAddr_e {
        kRegAddrStatus1  = 0 << 11,  //!< Status Register 1
        kRegAddrStatus2  = 1 << 11,  //!< Status Register 2
        kRegAddrControl1 = 2 << 11,  //!< Control Register 1
        kRegAddrControl2 = 3 << 11   //!< Control Register 2
    };


    struct RegisterFile {
        uint16_t control_register_1;
        uint16_t control_register_2;
    };

    static inline uint16_t build_ctrl_word(const CtrlMode_e ctrlMode,
                                           const RegAddr_e regName,
                                           const uint16_t data) {
        return ctrlMode | regName | (data & 0x07FF);
    }

    /** @brief Reads data from a DRV8301 register */
    bool read_reg(const RegAddr_e regName, uint16_t* data);

    /** @brief Writes data to a DRV8301 register. There is no check if the write succeeded. */
    bool write_reg(const RegAddr_e regName, const uint16_t data);

    int nCS_gpio_;
    int EN_gpio_;
    int nFAULT_gpio_;

    const int SPI_CLOCK_SPEED_HZ_DRV = 250000UL;

    SPIClass* spi_class_ptr;

    SPISettings spi_settings; //!< SPI settings variable


    RegisterFile regs_; //!< Current configuration. If is_ready_ is
                        //!< true then this can be considered consistent
                        //!< with the actual file on the DRV8301 chip.

    uint16_t tx_buf_, rx_buf_;

    enum {
        kStateUninitialized,
        kStateStartupChecks,
        kStateReady,
    } state_ = kStateUninitialized;
};


#endif // __DRV8301_HPP
