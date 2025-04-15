
#include "Arduino.h"
#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"
#include "RGB_Wrapper.hpp"
#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc



#define BLINK_ACCUMULATION_MAX_VALUE 1.0f




bool RGB_Wrapper_Pixel_Info_s::normal_pixel_update(void)
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





RGB_Wrapper::RGB_Wrapper(uint8_t num_of_leds_to_cmd_)
{
    if(num_of_leds_to_cmd_ > 0 \
    && num_of_leds_to_cmd_ <= WS2812B_MAX_NUM_OF_LEDS)
    {
        num_of_leds_to_cmd = num_of_leds_to_cmd_;
        status = RGB_Wrapper_Status_RGB_DRIVER_UNLINKED;
    }
    else
    {
        status = RGB_Wrapper_Status_INIT_FAIL;
    }
}


void RGB_Wrapper::link_ws2812b_rgb_led_driver_class(WS2812B_RGB_LED_Strip *led_drv_class_ptr)
{
    switch(status)
    {
        case RGB_Wrapper_Status_RGB_DRIVER_UNLINKED:
            if(led_drv_class_ptr)
            {
                led_strip_drv_ptr = led_drv_class_ptr;
                status = RGB_Wrapper_Status_INIT_PERIPH;
            }
            else
            {
                status = RGB_Wrapper_Status_INIT_FAIL;
            }
        break;

        default:
            // do nothing, don't change the class pointer!
        break;
    }
}

RGB_Wrapper_Status_e RGB_Wrapper::init_periphs_for_WS2812B(void)
{
    switch(status)
    {
        case RGB_Wrapper_Status_INIT_PERIPH:
            driver_status = led_strip_drv_ptr->init_dma_and_timer_peripherals(num_of_leds_to_cmd);
            switch(driver_status)
            {
                case WS2812B_WAITING_TO_PROCESS_PIXEL_BITSTREAM:
                    status = RGB_Wrapper_Status_INIT_PIXEL_TYPES;
                break;

                default:
                case WS2812B_INIT_FAIL:
                    status = RGB_Wrapper_Status_INIT_FAIL;
                break;
            }
        break;

        default:
            // do nothing
        break;
    }
    return status;
}



RGB_Wrapper_Status_e RGB_Wrapper::finish_initializing_pixel_types(void)
{
    uint8_t tmp_pixel_idx_range_itr = 0;
    uint8_t tmp_num_of_unconfigured_pixels = 0;

    /** return early since if there is an init fail, we don't know 
     * if num_of_leds_to_cmd is a valid value and theres no point in 
     * executing the function further. */
    if(status == RGB_Wrapper_Status_INIT_FAIL)
        return status;

    for(uint8_t tmp_pixel_idx = 0; tmp_pixel_idx < num_of_leds_to_cmd; tmp_pixel_idx++)
    {
        if(pixel_data[tmp_pixel_idx_range_itr].pixel_type == RGB_Wrapper__Unused_Pixel)
            tmp_num_of_unconfigured_pixels++;
    }

    switch(status)
    {
        case RGB_Wrapper_Status_INIT_PIXEL_TYPES:
        case RGB_Wrapper_Status_UPDATING_LED_BUF:
        case RGB_Wrapper_Status_PROCESSING_PIXEL_BITSTREAM:
        case RGB_Wrapper_Status_SEND_OUT_PIXEL_BITSTREAM:
            if(tmp_num_of_unconfigured_pixels == 0)
            {
                /** it will be assumed that there has been 
                 * an update to the settings, so don't bother
                 * continuing to update the previous configuration's
                 * pixel buffer.  Just start from scratch. */
                status = RGB_Wrapper_Status_UPDATING_LED_BUF;
            }
            else
            {
                status = RGB_Wrapper_Status_INIT_PIXEL_TYPES;
            }
        break;

        default:
        case RGB_Wrapper_Status_RGB_DRIVER_UNLINKED:
        case RGB_Wrapper_Status_INIT_PERIPH:
        case RGB_Wrapper_Status_INIT_FAIL:
            // do nothing, simply keep the status the same
        break; 
    }
    return status;
}




        
RGB_Wrapper_Status_e RGB_Wrapper::set_blink_pixel_settings(uint8_t blink_start_idx, \
                                                        uint8_t blink_end_idx, \
                                                        float increment_start_point, \
                                                        float increment_update_val, \
                                                        bool increment_direction_start, \
                                                        RGB_Wrapper_HV_Color_s color)
{
    if(blink_start_idx > blink_end_idx
    || blink_start_idx >= num_of_leds_to_cmd
    || blink_end_idx >= num_of_leds_to_cmd)
        return status;
    
    uint8_t tmp_pixel_idx_range_itr = blink_start_idx;
    
    while(tmp_pixel_idx_range_itr < blink_end_idx)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Blink_Pixel;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_accumulator = increment_start_point;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_or_rainbow_max_value = BLINK_ACCUMULATION_MAX_VALUE;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_value_to_increment_per_update = increment_update_val;
        pixel_data[tmp_pixel_idx_range_itr].is_blink_pulse_incrementing_up = increment_direction_start;
        pixel_data[tmp_pixel_idx_range_itr].colors[0] = color;
        
        pixel_data[tmp_pixel_idx_range_itr].last_adjacent_idx_of_similar_pixel_setting = blink_end_idx;
        
        tmp_pixel_idx_range_itr++;
    }
    
    return status;
}






