
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

/** value_in_radians * RADIANS_TO_DEGREES_COEFF = value_in_degrees */
#define RADIANS_TO_DEGREES_COEFF        (360.0f / _2PI)






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
    
    while(tmp_pixel_idx_range_itr < pulse_end_idx)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Blink_Pixel;
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
    
    while(tmp_pixel_idx_range_itr < rainbow_end_idx)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Blink_Pixel;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_accumulator = hue_start_point;
        pixel_data[tmp_pixel_idx_range_itr].blink_pulse_hue_max_accumulator_val = brightness_value;
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
                                                float light_persistence_coefficient_, \
                                                float num_of_circle_leds_on_at_once_,
                                                RGB_Wrapper_HV_Color_s init_color_data)
{
    if(circle_start_idx_ > circle_end_idx_
    || circle_start_idx_ >= num_of_leds_to_cmd
    || circle_end_idx_ >= num_of_leds_to_cmd
    || circle_type != RGB_Wrapper_Circle__Unset
    || num_of_circle_leds_on_at_once_ == 0) // only one "circle" is supported
        return status;
    
    uint8_t tmp_pixel_idx_range_itr = circle_start_idx_;
    
    while(tmp_pixel_idx_range_itr < circle_end_idx_)
    {
        pixel_data[tmp_pixel_idx_range_itr].pixel_type = RGB_Wrapper__Circle_Pixel; 
        pixel_data[tmp_pixel_idx_range_itr].last_adjacent_idx_of_similar_pixel_setting = circle_end_idx_; 
        pixel_data[tmp_pixel_idx_range_itr].color_buf = init_color_data; 
        
        tmp_pixel_idx_range_itr++;
    }

    num_of_circle_pixels = circle_end_idx_ - circle_start_idx_ + 1;
    circle_start_idx = circle_start_idx_;
    circle_end_idx = circle_end_idx_;
    circle_type = circle_type_;
    light_persistence_coefficient = light_persistence_coefficient_;
    phi_upright_hue_or_value = phi_upright_hue_or_value_;
    num_of_circle_leds_on_at_once = num_of_circle_leds_on_at_once_;

    status = RGB_Wrapper_Status_INIT_PIXEL_TYPES;
    
    return status;
}








RGB_Wrapper_Status_e RGB_Wrapper::update_pixels(float pdm_phi, float pdm_theta)
{
    uint8_t tmp_pixel_idx = 0;
    RGB_Wrapper_HV_Color_s tmp_similar_pixel_setting_color_buf;

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
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator = BLINK_ACCUMULATION_MAX_VALUE; 
                                pixel_data[tmp_pixel_idx].is_blink_pulse_incrementing_up = false;
                                pixel_data[tmp_pixel_idx].color_buf = pixel_data[tmp_pixel_idx].colors[0];
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
                        }

                        tmp_similar_pixel_setting_color_buf = pixel_data[tmp_pixel_idx].color_buf;
                        led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                    pixel_data[tmp_pixel_idx].color_buf.hue,
                                                                    pixel_data[tmp_pixel_idx].color_buf.value);
                        tmp_pixel_idx++;
                        
                        while(tmp_pixel_idx < pixel_data[tmp_pixel_idx].last_adjacent_idx_of_similar_pixel_setting)
                        {
                            pixel_data[tmp_pixel_idx].color_buf = tmp_similar_pixel_setting_color_buf;
                            led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                        pixel_data[tmp_pixel_idx].color_buf.hue,
                                                                        pixel_data[tmp_pixel_idx].color_buf.value);
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
                                pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator = BLINK_ACCUMULATION_MAX_VALUE; 
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
                        led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                    pixel_data[tmp_pixel_idx].color_buf.hue,
                                                                    pixel_data[tmp_pixel_idx].color_buf.value);
                        tmp_pixel_idx++;
                        
                        while(tmp_pixel_idx < pixel_data[tmp_pixel_idx].last_adjacent_idx_of_similar_pixel_setting)
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

                        pixel_data[tmp_pixel_idx].color_buf.hue = (uint8_t)pixel_data[tmp_pixel_idx].blink_pulse_hue_accumulator;

                        tmp_similar_pixel_setting_color_buf = pixel_data[tmp_pixel_idx].color_buf;
                        led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                    pixel_data[tmp_pixel_idx].color_buf.hue,
                                                                    pixel_data[tmp_pixel_idx].color_buf.value);
                        tmp_pixel_idx++;
                        
                        while(tmp_pixel_idx < pixel_data[tmp_pixel_idx].last_adjacent_idx_of_similar_pixel_setting)
                        {
                            pixel_data[tmp_pixel_idx].color_buf = tmp_similar_pixel_setting_color_buf;
                            led_strip_drv_ptr->modify_pixel_buffer_single(tmp_pixel_idx, 
                                                                        pixel_data[tmp_pixel_idx].color_buf.hue,
                                                                        pixel_data[tmp_pixel_idx].color_buf.value);
                            tmp_pixel_idx++;
                        }
                    break;


                    case RGB_Wrapper__Circle_Pixel:
                        update_circle_pixels(pdm_phi, pdm_theta, &tmp_pixel_idx);
                    break;


                    default:
                    case RGB_Wrapper__Button_Pixel:
                    case RGB_Wrapper__Unused_Pixel:

                    break;
                }
            }
        break;

        case RGB_Wrapper_Status_PROCESSING_PIXEL_BITSTREAM:
        {
            WS2812B_Status_e tmp_led_drv_status = led_strip_drv_ptr->process_bitfield_array(num_of_leds_to_cmd);
            switch(tmp_led_drv_status)
            {
                case WS2812B_READY_TO_UPLOAD_BITSTREAM:
                    if(led_strip_drv_ptr->write_bitfield_array_via_dma())
                        status = RGB_Wrapper_Status_SEND_OUT_PIXEL_BITSTREAM;
                    else
                        status = 0;

                break;
                
                case WS2812B_TRANSMITTING_DATA:
                    // do nothing, keep waiting till it's done.
                    status = RGB_Wrapper_Status_SEND_OUT_PIXEL_BITSTREAM;
                break;

                default:
                case WS2812B_INIT_PERIPHERALS:
                case WS2812B_INIT_FAIL:
                case WS2812B_WAITING_TO_PROCESS_PIXEL_BITSTREAM:
                    status = RGB_Wrapper_Status_INIT_FAIL;
                break;
            }
        }
        break;

        case RGB_Wrapper_Status_SEND_OUT_PIXEL_BITSTREAM:
            // do nothing, simply keep the status the same
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
            float tmp_hue_in_degrees = (phi_in_radians * 2.0f * RADIANS_TO_DEGREES_COEFF) + phi_upright_hue_or_value;

            if(tmp_hue_in_degrees >= 360.0f)
                tmp_hue_in_degrees -= 360.0f;
            else
                { /** do nothing */}

            pixel_data[*led_buf_idx].color_buf.hue = tmp_hue_in_degrees;

            tmp_similar_pixel_setting_color_buf = pixel_data[*led_buf_idx].color_buf;
            led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                        pixel_data[*led_buf_idx].color_buf.hue,
                                                        pixel_data[*led_buf_idx].color_buf.value);
            *led_buf_idx++;
            
            while(*led_buf_idx < pixel_data[*led_buf_idx].last_adjacent_idx_of_similar_pixel_setting)
            {
                pixel_data[*led_buf_idx].color_buf.hue = tmp_similar_pixel_setting_color_buf.hue;
                led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                            pixel_data[*led_buf_idx].color_buf.hue,
                                                            pixel_data[*led_buf_idx].color_buf.value);
                *led_buf_idx++;
            }
        }
        break;


        case RGB_Wrapper_Circle__Phi_Pulse:
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

            tmp_value_scaled_to_255 /= _PI;
            tmp_value_scaled_to_255 *= fabsf(phi_upright_hue_or_value);

            pixel_data[*led_buf_idx].color_buf.value = (uint8_t)tmp_value_scaled_to_255;

            tmp_similar_pixel_setting_color_buf = pixel_data[*led_buf_idx].color_buf;
            led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                        pixel_data[*led_buf_idx].color_buf.hue,
                                                        pixel_data[*led_buf_idx].color_buf.value);
            *led_buf_idx++;
            
            while(*led_buf_idx < pixel_data[*led_buf_idx].last_adjacent_idx_of_similar_pixel_setting)
            {
                pixel_data[*led_buf_idx].color_buf.hue = tmp_similar_pixel_setting_color_buf.hue;
                led_strip_drv_ptr->modify_pixel_buffer_single(*led_buf_idx, 
                                                            pixel_data[*led_buf_idx].color_buf.hue,
                                                            pixel_data[*led_buf_idx].color_buf.value);
                *led_buf_idx++;
            }
        }
        break;


        case RGB_Wrapper_Circle__Phi_Theta_Solid_Value:

        break;


        case RGB_Wrapper_Circle__Phi_Theta_Q:

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




inline void RGB_Wrapper::update_circle_tracking_info(float phi_in_radians, float theta_in_radians)
{
    float tmp_theta_summation = theta_in_radians + theta_offset_in_radians;

    if(tmp_theta_summation >= _2PI)
        tmp_theta_summation -= _2PI;
    else if(tmp_theta_summation < 0.0f)
        tmp_theta_summation += _2PI;
    else
        { /** do nothing */}

    theta_with_led_offset_in_radians = tmp_theta_summation;

    float tmp_pdm_angle_in_led_idx = (theta_with_led_offset_in_radians) * (float)num_of_leds_to_cmd / _2PI;
}




