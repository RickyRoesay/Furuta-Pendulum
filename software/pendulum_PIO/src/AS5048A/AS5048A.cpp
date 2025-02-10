#include "AS5048A.hpp"
#include "../../lib/Arduino-FOC/src/SimpleFOC.h"


void AS5048A::link_spi_class(SPIClass* spi)
{
    spi_class_ptr = spi;
}
