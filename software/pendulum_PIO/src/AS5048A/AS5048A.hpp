#ifndef __AS5048A_HPP
#define __AS5048A_HPP

#include "Arduino.h"
#include <SPI.h>

//AS5048A::



class AS5048A {
public:
    typedef enum : uint32_t {
        FaultType_NoFault  = (0 << 0),  //!< No fault

        // Status Register 1
        FaultType_FETLC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
        FaultType_FETHC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
        FaultType_FETLB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
        FaultType_FETHB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
        FaultType_FETLA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
        FaultType_FETHA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
        FaultType_OTW      = (1 << 6),  //!< Over Temperature Warning fault
        FaultType_OTSD     = (1 << 7),  //!< Over Temperature Shut Down fault
        FaultType_PVDD_UV  = (1 << 8),  //!< Power supply Vdd Under Voltage fault
        FaultType_GVDD_UV  = (1 << 9),  //!< DRV8301 Vdd Under Voltage fault
        FaultType_FAULT    = (1 << 10),

        // Status Register 2
        FaultType_GVDD_OV  = (1 << 23)  //!< DRV8301 Vdd Over Voltage fault
    } FaultType_e;


    AS5048A(int nCS_gpio) : nCS_gpio_(nCS_gpio) {}

    void link_spi_class(SPIClass* spi);

    bool is_ready();
    
    bool init();

    void do_checks();

    FaultType_e get_error();

private:
    enum CtrlMode_e {
        DRV8301_CtrlMode_Read = 1 << 15,   //!< Read Mode
        DRV8301_CtrlMode_Write = 0 << 15   //!< Write Mode
    };

    enum RegName_e {
        RegName_NOP = 0x0000U, // no operation dummy information
        RegName_ClearErrorFlags = 0x0001U, // read out the error flags, clear them on read
        RegName_ProgrammingControl = 0x0003U, // programming control
        RegName_ProgrammingControlHigh = 0x0016U, // OTP Reg Zero Pos top 8 bits
        RegName_ProgrammingControlLow = 0x0017U, // OTP Reg Zero Pos lower 6 bits
    };


    struct ErrFlags_s {
        uint16_t framing_error:1U;
        uint16_t command_inv:1U;
        uint16_t parity_error:1U;
        uint16_t ErrFlags_unused:13U;
    };

    union RegName_u {
        uint16_t all;
        ErrFlags_s bit; 
    };

    struct RegisterFile {
        uint16_t control_register_1;
        uint16_t control_register_2;
    };

    static inline uint16_t build_addr_word(const CtrlMode_e ctrlMode,
                                           const RegName_e regName,
                                           const uint16_t data) {
        return ctrlMode | regName | (data & 0x0FFF);
    }

    /** @brief Reads data from a DRV8301 register */
    bool read_reg(const RegName_e regName, uint16_t* data);

    /** @brief Writes data to a DRV8301 register. There is no check if the write succeeded. */
    bool write_reg(const RegName_e regName, const uint16_t data);

    int nCS_gpio_;

    const int SPI_CLOCK_SPEED_HZ_DRV = 1000000UL;

    SPIClass* spi_class_ptr;

    SPISettings spi_settings; //!< SPI settings variable

    RegisterFile regs_; 

    uint16_t tx_buf_, rx_buf_;

    enum {
        kStateUninitialized,
        kStateStartupChecks,
        kStateReady,
    } state_ = kStateUninitialized;
};



#endif // __AS5048A_HPP
