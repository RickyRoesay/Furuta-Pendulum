
#include "Arduino.h"
#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"
#include "RGB_Wrapper.hpp"




bool RGB_Wrapper_Pixel_Info_s::rgb_wrapper_pixel_update(void)
{
    bool ret_val = true;
    switch(pixel_type)
    {
        default:
        case RGB_Wrapper__Unused_Pixel:
            ret_val = false; // fallthrough:
        case RGB_Wrapper__Circle_Pixel: // circle pixels must be updated all as a group
            // do nothing
        break;


    }

    return ret_val;
}


