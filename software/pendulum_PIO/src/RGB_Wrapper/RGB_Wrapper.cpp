
#include "Arduino.h"
#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"
#include "RGB_Wrapper.hpp"
#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc
#include "RGB_Utils.hpp"
#include "gpio.h"


#define BLINK_ACCUMULATION_MAX_VALUE 1.0f

/** value_in_radians * RADIANS_TO_DEGREES_COEFF = value_in_degrees */
#define RADIANS_TO_DEGREES_COEFF        (360.0f / _2PI)

#define ONE_OVER_2PI_  (1.0f / _2PI)
#define ONE_OVER_PI_  (1.0f / _PI)





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
            switch(led_strip_drv_ptr->init_dma_and_timer_peripherals(num_of_leds_to_cmd))
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





        
RGB_Wrapper_Status_e RGB_Wrapper::set_theta_offset_in_radians(float pdm_theta_offset_rad,
                                                        RGB_Wrapper_Theta_Tracking_Direction tracking_direction)
{
    theta_offset_in_radians = pdm_theta_offset_rad;
    theta_tracking_direction = tracking_direction;
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
    
    while(tmp_pixel_idx_range_itr <= blink_end_idx)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Blink_Pixel;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_accumulator = increment_start_point;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_max_accumulator_val = BLINK_ACCUMULATION_MAX_VALUE;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_increment_val = increment_update_val;
        pixel_data[tmp_pixel_idx_range_itr].is_blink_pulse_incrementing_up = increment_direction_start;
        pixel_data[tmp_pixel_idx_range_itr].colors[0] = color;
        pixel_data[tmp_pixel_idx_range_itr].color_buf = color;
        
        pixel_data[tmp_pixel_idx_range_itr].last_adjacent_idx_of_similar_pixel_setting = blink_end_idx;
        
        tmp_pixel_idx_range_itr++;
    }

    status = RGB_Wrapper_Status_INIT_PIXEL_TYPES;
    
    return status;
}



RGB_Wrapper_Status_e RGB_Wrapper::set_pulse_pixel_settings(uint8_t pulse_start_idx, \
                                                uint8_t pulse_end_idx, \
                                                float val_increment_start_point, \
                                                float val_increment_update_val, \
                                                bool val_increment_direction_start, \
                                                RGB_Wrapper_HV_Color_s color_with_max_value)
{
    if(pulse_start_idx > pulse_end_idx
    || pulse_start_idx >= num_of_leds_to_cmd
    || pulse_end_idx >= num_of_leds_to_cmd)
        return status;
    
    uint8_t tmp_pixel_idx_range_itr = pulse_start_idx;
    
    while(tmp_pixel_idx_range_itr <= pulse_end_idx)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Pulse_Pixel;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_accumulator = val_increment_start_point;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_max_accumulator_val = (float)color_with_max_value.value;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_increment_val = val_increment_update_val;
        pixel_data[tmp_pixel_idx_range_itr].is_blink_pulse_incrementing_up = val_increment_direction_start;
        pixel_data[tmp_pixel_idx_range_itr].colors[0] = color_with_max_value;
        pixel_data[tmp_pixel_idx_range_itr].color_buf = color_with_max_value;
        
        pixel_data[tmp_pixel_idx_range_itr].last_adjacent_idx_of_similar_pixel_setting = pulse_end_idx;

        tmp_pixel_idx_range_itr++;
    }

    status = RGB_Wrapper_Status_INIT_PIXEL_TYPES;
    
    return status;
}


        
RGB_Wrapper_Status_e RGB_Wrapper::set_rainbow_pixel_settings(uint8_t rainbow_start_idx, \
                                                uint8_t rainbow_end_idx, \
                                                uint8_t brightness_value, \
                                                float hue_start_point, \
                                                float hue_increment_update_val)
{
    if(rainbow_start_idx > rainbow_end_idx
    || rainbow_start_idx >= num_of_leds_to_cmd
    || rainbow_end_idx >= num_of_leds_to_cmd
    || hue_start_point < 0.0f 
    || hue_start_point >=360.0f)
        return status;
    
    uint8_t tmp_pixel_idx_range_itr = rainbow_start_idx;
    
    while(tmp_pixel_idx_range_itr <= rainbow_end_idx)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Rainbow_Pixel;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_accumulator = hue_start_point;
        pixel_data[tmp_pixel_idx_range_itr].color_buf.value = brightness_value;
        pixel_data[tmp_pixel_idx_range_itr].color_buf.hue = hue_start_point;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_increment_val = hue_increment_update_val;
        
        pixel_data[tmp_pixel_idx_range_itr].last_adjacent_idx_of_similar_pixel_setting = rainbow_end_idx;
        
        tmp_pixel_idx_range_itr++;
    }

    status = RGB_Wrapper_Status_INIT_PIXEL_TYPES;
    
    return status;
}




RGB_Wrapper_Status_e RGB_Wrapper::set_circle_pixel_settings(uint8_t circle_start_idx_, \
                                                uint8_t circle_end_idx_, \
                                                RGB_Wrapper_Circle_Type_e circle_type_, \
                                                float phi_upright_hue_or_value_, \
                                                float light_persistence_decay_val_, \
                                                float num_of_circle_leds_on_at_once_,
                                                RGB_Wrapper_HV_Color_s init_color_data)
{
    uint8_t tmp_num_of_circle_pixels = circle_end_idx_ - circle_start_idx_ + 1;
    if(circle_start_idx_ > circle_end_idx_
    || circle_start_idx_ >= num_of_leds_to_cmd
    || circle_end_idx_ >= num_of_leds_to_cmd
    || circle_type != RGB_Wrapper_Circle__Unset
    || num_of_circle_leds_on_at_once_ == 0.0f
    || num_of_circle_leds_on_at_once_ >= (float)(tmp_num_of_circle_pixels)) // only one "circle" is supported
        return status;
    
    uint8_t tmp_pixel_idx_range_itr = circle_start_idx_;
    
    while(tmp_pixel_idx_range_itr <= circle_end_idx_)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Circle_Pixel; 
        pixel_data[tmp_pixel_idx_range_itr].last_adjacent_idx_of_similar_pixel_setting = circle_end_idx_; 
        pixel_data[tmp_pixel_idx_range_itr].persistence_HV_value = 0.0f; 
        pixel_data[tmp_pixel_idx_range_itr].color_buf = init_color_data; 
        
        tmp_pixel_idx_range_itr++;
    }

    num_of_circle_pixels_u8 = tmp_num_of_circle_pixels;
    num_of_circle_pixels_f32 = (float)num_of_circle_pixels_u8;
    circle_start_idx = circle_start_idx_;
    circle_end_idx = circle_end_idx_;
    circle_type = circle_type_;
    light_persistence_decay_val = light_persistence_decay_val_;
    phi_upright_hue_or_value = phi_upright_hue_or_value_;
    num_of_circle_leds_on_at_once = num_of_circle_leds_on_at_once_;
    half_num_of_circle_leds_on_at_once = (num_of_circle_leds_on_at_once_ / 2.0f);

    status = RGB_Wrapper_Status_INIT_PIXEL_TYPES;
    
    return status;
}








RGB_Wrapper_Status_e RGB_Wrapper::update_pixels(float pdm_phi, float pdm_theta)
{
    uint8_t tmp_pixel_idx = 0;
    RGB_Wrapper_HV_Color_s tmp_similar_pixel_setting_color_buf;
    uint8_t tmp_last_similar_pixel_idx;

    switch(status)
    {
        default:
        case RGB_Wrapper_Status_RGB_DRIVER_UNLINKED:
        case RGB_Wrapper_Status_INIT_PERIPH:
        case RGB_Wrapper_Status_INIT_FAIL:
        case RGB_Wrapper_Status_INIT_PIXEL_TYPES:
            // do nothing, simply keep the status the same
        break;



        case RGB_Wrapper_Status_UPDATING_LED_BUF:
        
            //digitalWrite(GPIO1__SSD_nCS, HIGH);
            while(tmp_pixel_idx < num_of_leds_to_cmd)
            {
                switch(pixel_data[tmp_pixel_idx].pixel_type)
                {
                    case RGB_Wrapper__Blink_Pixel:
                        if(pixel_data[tmp_pixel_idx].is_blink_pulse_incrementing_up == true)
                        {
                            pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator += \
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_increment_val; 
                            
                            if(pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator > BLINK_ACCUMULATION_MAX_VALUE)
                            {
                                pixel_data[tmp_pixel_idx].color_buf.value = 0;
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator = BLINK_ACCUMULATION_MAX_VALUE; 
                                pixel_data[tmp_pixel_idx].is_blink_pulse_incrementing_up = false;
                            }
                        }
                        else
                        {
                            pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator -= \
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_increment_val; 
                            
                            if(pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator < 0.0f)
                            {
                                pixel_data[tmp_pixel_idx].color_buf = pixel_data[tmp_pixel_idx].colors[0];
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator = 0.0f; 
                                pixel_data[tmp_pixel_idx].is_blink_pulse_incrementing_up = true;
                            }
                        }

                        tmp_similar_pixel_setting_color_buf = pixel_data[tmp_pixel_idx].color_buf;
                        
                        tmp_last_similar_pixel_idx = pixel_data[tmp_pixel_idx].last_adjacent_idx_of_similar_pixel_setting;
                        
                        while(tmp_pixel_idx <= tmp_last_similar_pixel_idx)
                        {
                            led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                        tmp_similar_pixel_setting_color_buf.hue,
                                                                        tmp_similar_pixel_setting_color_buf.value);
                            tmp_pixel_idx++;
                        }
                    break;


                    case RGB_Wrapper__Pulse_Pixel:
                        if(pixel_data[tmp_pixel_idx].is_blink_pulse_incrementing_up == true)
                        {
                            pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator += \
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_increment_val; 
                            
                            if(pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator 
                                > pixel_data[tmp_pixel_idx].blink_pulse_hue_max_accumulator_val)

                            {
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator = pixel_data[tmp_pixel_idx].blink_pulse_hue_max_accumulator_val; 
                                pixel_data[tmp_pixel_idx].is_blink_pulse_incrementing_up = false;
                                pixel_data[tmp_pixel_idx].color_buf = pixel_data[tmp_pixel_idx].colors[0];
                            }
                            else
                            {
                                pixel_data[tmp_pixel_idx].color_buf.value = (uint8_t)pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator;
                            }
                        }
                        else
                        {
                            pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator -= \
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_increment_val; 
                            
                            if(pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator < 0.0f)
                            {
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator = 0.0f; 
                                pixel_data[tmp_pixel_idx].is_blink_pulse_incrementing_up = true;
                                pixel_data[tmp_pixel_idx].color_buf.value = 0;
                            }
                            else
                            {
                                pixel_data[tmp_pixel_idx].color_buf.value = (uint8_t)pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator;
                            }
                        }

                        tmp_similar_pixel_setting_color_buf = pixel_data[tmp_pixel_idx].color_buf;
                        
                        tmp_last_similar_pixel_idx = pixel_data[tmp_pixel_idx].last_adjacent_idx_of_similar_pixel_setting;
                        
                        while(tmp_pixel_idx <= tmp_last_similar_pixel_idx)
                        {
                            pixel_data[tmp_pixel_idx].color_buf = tmp_similar_pixel_setting_color_buf;
                            led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                        pixel_data[tmp_pixel_idx].color_buf.hue,
                                                                        pixel_data[tmp_pixel_idx].color_buf.value);
                            tmp_pixel_idx++;
                        }
                    break;


                    case RGB_Wrapper__Rainbow_Pixel:
                        pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator += \
                            pixel_data[tmp_pixel_idx].blink_pulse_hue_increment_val; 
                        
                        if(pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator >= 360.0f)
                        {
                            pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator -= 360.0f; 
                        }
                        else if(pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator < 0.0f)
                        {
                            pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator += 360.0f; 
                        }

                        pixel_data[tmp_pixel_idx].color_buf.hue = pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator;

                        tmp_similar_pixel_setting_color_buf = pixel_data[tmp_pixel_idx].color_buf;

                        tmp_last_similar_pixel_idx = pixel_data[tmp_pixel_idx].last_adjacent_idx_of_similar_pixel_setting;
                        
                        while(tmp_pixel_idx <= tmp_last_similar_pixel_idx)
                        {
                            led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                        tmp_similar_pixel_setting_color_buf.hue,
                                                                        tmp_similar_pixel_setting_color_buf.value);
                            tmp_pixel_idx++;
                        }
                    break;


                    case RGB_Wrapper__Circle_Pixel:
                        update_circle_pixels(fabsf(pdm_phi), pdm_theta, &tmp_pixel_idx);
                    break;


                    default:
                    case RGB_Wrapper__Button_Pixel:
                    case RGB_Wrapper__Unused_Pixel:
                        /** Increment the idx so we eventually leave this state in the case of 
                         * an invalid LED setting.  */
                        tmp_pixel_idx++;
                    break;
                }
            }
            status = RGB_Wrapper_Status_PROCESSING_PIXEL_BITSTREAM;
            //digitalWrite(GPIO1__SSD_nCS, LOW);
        break;



        case RGB_Wrapper_Status_PROCESSING_PIXEL_BITSTREAM:
        {
            WS2812B_Status_e tmp_led_drv_status = led_strip_drv_ptr->process_bitfield_array(10);
            switch(tmp_led_drv_status)
            {
                case WS2812B_READY_TO_UPLOAD_BITSTREAM:
                    if(led_strip_drv_ptr->write_bitfield_array_via_dma() == WS2812B_TRANSMITTING_DATA)
                        status = RGB_Wrapper_Status_UPDATING_LED_BUF;
                    else
                        status = RGB_Wrapper_Status_INIT_FAIL;

                break;
                
                case WS2812B_TRANSMITTING_DATA:
                case WS2812B_WAITING_TO_PROCESS_PIXEL_BITSTREAM:
                    // do nothing, keep waiting till it's done.
                    status = RGB_Wrapper_Status_PROCESSING_PIXEL_BITSTREAM;
                break;

                default:
                case WS2812B_INIT_PERIPHERALS:
                case WS2812B_INIT_FAIL:
                    status = RGB_Wrapper_Status_INIT_FAIL;
                break;
            }
        }
        break;

    }

    return status;
}





inline void RGB_Wrapper::update_circle_pixels(float phi_in_radians, float theta_in_radians, uint8_t* led_buf_idx)
{
    RGB_Wrapper_HV_Color_s tmp_similar_pixel_setting_color_buf;

    update_circle_tracking_info(phi_in_radians, theta_in_radians);
    
    switch(circle_type)
    {
        case RGB_Wrapper_Circle__Phi_Rainbow:
        {
            pixel_data[*led_buf_idx].color_buf.hue = phi_in_hue_or_value;

            tmp_similar_pixel_setting_color_buf = pixel_data[*led_buf_idx].color_buf;
            led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                        pixel_data[*led_buf_idx].color_buf.hue,
                                                        pixel_data[*led_buf_idx].color_buf.value);
            (*led_buf_idx)++;
            
            while(*led_buf_idx < pixel_data[*led_buf_idx].last_adjacent_idx_of_similar_pixel_setting)
            {
                pixel_data[*led_buf_idx].color_buf.hue = tmp_similar_pixel_setting_color_buf.hue;
                led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                            pixel_data[*led_buf_idx].color_buf.hue,
                                                            pixel_data[*led_buf_idx].color_buf.value);
                (*led_buf_idx)++;
            }
        }
        break;


        case RGB_Wrapper_Circle__Phi_Pulse:
        {
            pixel_data[*led_buf_idx].color_buf.value = (uint8_t)phi_in_hue_or_value;

            tmp_similar_pixel_setting_color_buf = pixel_data[*led_buf_idx].color_buf;
            led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                        pixel_data[*led_buf_idx].color_buf.hue,
                                                        pixel_data[*led_buf_idx].color_buf.value);
            (*led_buf_idx)++;
            
            while(*led_buf_idx < pixel_data[*led_buf_idx].last_adjacent_idx_of_similar_pixel_setting)
            {
                pixel_data[*led_buf_idx].color_buf.hue = tmp_similar_pixel_setting_color_buf.hue;
                led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                            pixel_data[*led_buf_idx].color_buf.hue,
                                                            pixel_data[*led_buf_idx].color_buf.value);
                (*led_buf_idx)++;
            }
        }
        break;


        case RGB_Wrapper_Circle__Phi_Rainbow_Theta_Solid_Value:
        {
            if(circle_led_on_start_c_idx < circle_led_on_end_c_idx)
            {
                while(*led_buf_idx < (circle_start_idx + circle_led_on_start_c_idx))
                {
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 0, 0, 0);
                    (*led_buf_idx)++;
                }
                
                while(*led_buf_idx <= (circle_start_idx + circle_led_on_end_c_idx))
                {
                    /** static HV "Value" was set as initial color buf value during circle led init function call. */
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx,
                                                                phi_in_hue_or_value,
                                                                pixel_data[*led_buf_idx].color_buf.value);
                    (*led_buf_idx)++;
                }

                while(*led_buf_idx <= circle_end_idx)
                {
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 0, 0, 0);
                    (*led_buf_idx)++;
                }
            }
            else
            {
                while(*led_buf_idx <= (circle_start_idx + circle_led_on_end_c_idx))
                {
                    /** static HV "Value" was set as initial color buf value during circle led init function call. */
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx,
                                                                phi_in_hue_or_value,
                                                                pixel_data[*led_buf_idx].color_buf.value);
                    (*led_buf_idx)++;
                }

                while(*led_buf_idx < (circle_start_idx + circle_led_on_start_c_idx))
                {
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 0, 0, 0);
                    (*led_buf_idx)++;
                }

                while(*led_buf_idx <= circle_end_idx)
                {
                    /** static HV "Value" was set as initial color buf value during circle led init function call. */
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                                phi_in_hue_or_value,
                                                                pixel_data[*led_buf_idx].color_buf.value);
                    (*led_buf_idx)++;
                }
            }
        }
        break;


        case RGB_Wrapper_Circle__Phi_Rainbow_Theta_Q:
        {
            if(circle_led_on_start_c_idx < circle_led_on_end_c_idx)
            {
                while(*led_buf_idx < (circle_start_idx + circle_led_on_start_c_idx))
                {
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 0, 0, 0);
                    (*led_buf_idx)++;
                }
                
                while(*led_buf_idx <= (circle_start_idx + circle_led_on_end_c_idx))
                {
                    /** for the second param, max HV "Value" was set as initial color buf value during circle led init function call. */
                    uint8_t tmp_new_value = get_hv_value_based_on_distance_from_pdm((*led_buf_idx - circle_start_idx), 
                                                                                    pixel_data[*led_buf_idx].color_buf.value);
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx,
                                                                phi_in_hue_or_value,
                                                                tmp_new_value);
                    (*led_buf_idx)++;
                }

                while(*led_buf_idx <= circle_end_idx)
                {
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 0, 0, 0);
                    (*led_buf_idx)++;
                }
            }
            else
            {
                while(*led_buf_idx <= (circle_start_idx + circle_led_on_end_c_idx))
                {
                    /** for the second param, max HV "Value" was set as initial color buf value during circle led init function call. */
                    uint8_t tmp_new_value = get_hv_value_based_on_distance_from_pdm((*led_buf_idx - circle_start_idx), 
                                                                                    pixel_data[*led_buf_idx].color_buf.value);
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx,
                                                                phi_in_hue_or_value,
                                                                tmp_new_value);
                    (*led_buf_idx)++;
                }

                while(*led_buf_idx < (circle_start_idx + circle_led_on_start_c_idx))
                {
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 0, 0, 0);
                    (*led_buf_idx)++;
                }

                while(*led_buf_idx <= circle_end_idx)
                {
                    /** for the second param, max HV "Value" was set as initial color buf value during circle led init function call. */
                    uint8_t tmp_new_value = get_hv_value_based_on_distance_from_pdm((*led_buf_idx - circle_start_idx), 
                                                                                    pixel_data[*led_buf_idx].color_buf.value);
                    led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx,
                                                                phi_in_hue_or_value,
                                                                tmp_new_value);
                    (*led_buf_idx)++;
                }
            }
        }
        break;


        case RGB_Wrapper_Circle__Phi_Value_Theta_Solid_Value:

        break;


        case RGB_Wrapper_Circle__Phi_Theta_Persistence_Rainbow:

        break;


        case RGB_Wrapper_Circle__Phi_Theta_Persistence_Solid:

        break;

        
        default:
        case RGB_Wrapper_Circle__Unset:
            // do nothing
        break;
    }
}





/** for an angle of "theta+offset = pi" and an LED count of 5,
 * this value would be 5/2pi*pi = 5/2 = 2.5.  The peak
 * point of the "Q" would be halfway between LED[2] and LED[3].
 * 
 * NOTE: Keep in mind that due to the circular alignment of the 
 * LED's, the max index would not be = to "num_of_circle_pixels_u8." 
 * The max index would be (num_of_circle_pixels_u8 + 1) to account for
 * the distance between the led's at index [0] and [num_of_circle_pixels_u8]. */

/**  Updates: 
 *    - theta_with_led_offset_in_radians
 *    - theta_in_circle_idx
 *    - phi_in_hue_or_value
 *    - circle_led_on_start_c_idx
 *    - circle_led_on_end_c_idx
 */
inline void RGB_Wrapper::update_circle_tracking_info(float phi_in_radians, float theta_in_radians)
{
    float tmp_theta_summation;

    if(theta_tracking_direction == RGB_Wrapper_Theta_Tracking_Direction__NORMAL)
        tmp_theta_summation = theta_in_radians;
    else
        tmp_theta_summation = (_2PI - theta_in_radians);
    
    tmp_theta_summation += theta_offset_in_radians;

    if(tmp_theta_summation >= _2PI)
        tmp_theta_summation -= _2PI;
    else if(tmp_theta_summation < 0.0f)
        tmp_theta_summation += _2PI;
    else
        { /** do nothing */}

    switch(circle_type)
    {
        /** scale phi to a usable H,V "Value" [0, 255]  */
        case RGB_Wrapper_Circle__Phi_Pulse:
        case RGB_Wrapper_Circle__Phi_Value_Theta_Solid_Value:
        {
            float tmp_value_scaled_to_255;
            
            /** if phi_upright_hue_or_value is entered as a negative value,
             * that means there will be 0 brightness at phi = 0, and full brightness
             * (or whatever the absolute magnitude of phi_upright_hue_or_value is)
             * at phi = PI. */
            if(phi_upright_hue_or_value < 0.0f)
                tmp_value_scaled_to_255 = _PI - phi_in_radians;
            else
                tmp_value_scaled_to_255 = phi_in_radians;
            
            tmp_value_scaled_to_255 *= ONE_OVER_PI_;
            tmp_value_scaled_to_255 *= fabsf(phi_upright_hue_or_value);
            
            phi_in_hue_or_value = tmp_value_scaled_to_255;
        }
        break;
        
        
        /** scale phi to a usable H,V "Hue" [0, 360)  */
        default:
        case RGB_Wrapper_Circle__Unset:
        case RGB_Wrapper_Circle__Phi_Rainbow:
        case RGB_Wrapper_Circle__Phi_Rainbow_Theta_Q:
        case RGB_Wrapper_Circle__Phi_Rainbow_Theta_Solid_Value:
        case RGB_Wrapper_Circle__Phi_Theta_Persistence_Rainbow:
        case RGB_Wrapper_Circle__Phi_Theta_Persistence_Solid:
        {
            
            float tmp_value_scaled_to_360;
            if(phi_upright_hue_or_value < 0.0f)
                tmp_value_scaled_to_360 = _2PI - phi_in_radians;
            else
                tmp_value_scaled_to_360 = phi_in_radians;

            float tmp_hue_in_degrees = (tmp_value_scaled_to_360 * 1.0f * RADIANS_TO_DEGREES_COEFF) + phi_upright_hue_or_value;

            if(tmp_hue_in_degrees >= 360.0f)
                tmp_hue_in_degrees -= 360.0f;
            else
                { /** do nothing */}

            phi_in_hue_or_value = tmp_hue_in_degrees;
        }
        break;
    }

    theta_with_led_offset_in_radians = tmp_theta_summation;

    theta_in_circle_idx = theta_with_led_offset_in_radians * num_of_circle_pixels_f32 * ONE_OVER_2PI_;

    float tmp_led_on_start_idx = theta_in_circle_idx - half_num_of_circle_leds_on_at_once;
    if(tmp_led_on_start_idx < 0.0f)
        tmp_led_on_start_idx += num_of_circle_pixels_f32;
    tmp_led_on_start_idx += 0.5f;

    circle_led_on_start_c_idx = (uint8_t)tmp_led_on_start_idx;
    if(circle_led_on_start_c_idx == num_of_circle_pixels_u8)
        circle_led_on_start_c_idx = 0;
    

    float tmp_led_on_end_idx = theta_in_circle_idx + half_num_of_circle_leds_on_at_once;
    if(tmp_led_on_end_idx >= num_of_circle_pixels_f32)
        tmp_led_on_end_idx -= num_of_circle_pixels_f32;
    tmp_led_on_end_idx += 0.5f;

    circle_led_on_end_c_idx = (uint8_t)tmp_led_on_end_idx;
    if(circle_led_on_end_c_idx == num_of_circle_pixels_u8)
        circle_led_on_end_c_idx = 0;
    

}


inline uint8_t RGB_Wrapper::get_hv_value_based_on_distance_from_pdm(uint8_t led_circle_idx, 
                                                                    uint8_t max_value)
{
    float tmp_linear_distance_between_pdm_base_and_led_idx = fabsf(theta_in_circle_idx - (float)led_circle_idx);
    float tmp_cyclical_distance_between_pdm_base_and_led_in_idx;

    /** Must account for when the LED index has wrapped around the circle before the
     * angle of the pendulum base has.  Analogous to 2 vectors having a phase shift of 3 vs. 357 degrees; We 
     * want the lower number. */
    if(tmp_linear_distance_between_pdm_base_and_led_idx > (half_num_of_circle_leds_on_at_once + 1.0f)) 
        tmp_cyclical_distance_between_pdm_base_and_led_in_idx = num_of_circle_pixels_f32 - tmp_linear_distance_between_pdm_base_and_led_idx;
    else
        tmp_cyclical_distance_between_pdm_base_and_led_in_idx = tmp_linear_distance_between_pdm_base_and_led_idx;

    /** half of total LED ON width = num_of_circle_leds_on_at_once / 2 = half_num_of_circle_leds_on_at_once   since the varying
     * levels of LED brightness/value decay symetrically across the pendulum's base angle. */
    float tmp_distance_as_prcnt_of_half_total = tmp_cyclical_distance_between_pdm_base_and_led_in_idx / half_num_of_circle_leds_on_at_once;
    
    float tmp_hv_value_f32 = (float)max_value * get_scaled_val_for_q_shaped_value_pulse_via_lut(tmp_distance_as_prcnt_of_half_total);

    /** I tried to have this be a patch fix for the bad lookup table to fix the aestetics of
     * LED's turning off/on asymetrically to the pendulum base angle.  The lookup table needs to be 
     * improved to ensure LED's turn on/off symetrical to the pendulum base angle, and that the 
     * decay of LED brighness is also symetrical.  I think a near linear progression would work better. */
    #if 0
    /** To make sure that a consistent number of LED's are on at once, ensure all
     * LED's have at least an HV "Value" of 1 */
    if(tmp_hv_value_f32 < 1.0f)
        tmp_hv_value_f32 = 1.0f;
    #endif 

    return (uint8_t)tmp_hv_value_f32;
}

