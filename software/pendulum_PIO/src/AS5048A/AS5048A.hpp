#ifndef __AS5048A_HPP
#define __AS5048A_HPP

#include "Arduino.h"
#include <SPI.h>
#include "../../lib/Arduino-FOC/src/SimpleFOC.h"



class AS5048A {
public:
   
    AS5048A(int nCS_gpio) : nCS_gpio_(nCS_gpio) {}

    void link_spi_class(SPIClass* spi);
    
    bool init();

private:
    struct ErrFlags_s {
        uint16_t framing_error:1U;
        uint16_t command_inv:1U;
        uint16_t parity_error:1U;
        uint16_t ErrFlags_unused:13U;
    };

    union ErrFlags_u {
        uint16_t all;
        ErrFlags_s bit; 
    };

#if 0
    /** @brief Reads data from a DRV8301 register */
    bool read_reg(const RegAddr_e regName, uint16_t* data);

    /** @brief Writes data to a DRV8301 register. There is no check if the write succeeded. */
    bool write_reg(const RegAddr_e regName, const uint16_t data);
#endif 
    int nCS_gpio_;

    const int SPI_CLOCK_SPEED_HZ_DRV = 1000000UL;

    SPIClass* spi_class_ptr;

    SPISettings spi_settings; //!< SPI settings variable

    uint16_t tx_buf_, rx_buf_;
};



#endif // __AS5048A_HPP
