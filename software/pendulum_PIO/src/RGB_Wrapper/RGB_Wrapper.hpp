#ifndef __RGB_WRAPPER_HPP
#define __RGB_WRAPPER_HPP

#include "Arduino.h"
#include <SPI.h>
#include "Drivers/WS2812B_RGB_LED_Strip/WS2812B_RGB_LED_Strip.hpp" // RGB LED's
//#include "stm32f405xx.h"

/** This software component is meant to be a "wrapper" to aid in 
 * the concurrent execution of controlling the RGB LED strip.  
 * 
 * This is meant to be called in the same calling function that runs 
 * the main control loop at 5kHz.  This aids in timing since it ensures
 * safe concurrency of the tasks, which lets the control loop run unhindered
 * while still taking up a considerable amount of CPU load (like 20%). */
//



/** the idea behind this is to have a button illuminate a certain color
 * for how many times it's been quickly pressed in a row.  This will
 * give feedback for if 2 successive button presses have counted as a double 
 * click vs 2 single clicks.  I think a "triple click" will be the most 
 * clicks for a given purpose so the maximum number of colors a pixel 
 * will ever be assigned will be 3. */
#define MAX_BUTTON_COLORS 3



typedef enum : uint8_t {
    RGB_Wrapper__Unused_Pixel = 0,  // default value
    RGB_Wrapper__Button_Pixel,  // blink a color when pressed or depressed, takes button state as input
    RGB_Wrapper__Blink_Pixel, // blinking a solid color between off and a static brightness/value
    RGB_Wrapper__Pulse_Pixel, // pulsing a solid color different brightnesses
    RGB_Wrapper__Rainbow_Pixel, // cycling through the rainbow
    RGB_Wrapper__Circle_Pixel, // one of the LED's that iluminate around the pendulum 
} RGB_Wrapper_Pixel_Type_e;

typedef enum : uint8_t {
    RGB_Wrapper_Circle__Phi_Rainbow = 0, // whole circle is illuminated a different color as a function of phi
    RGB_Wrapper_Circle__Phi_Pulse, // whole circle is illuminated a solid color but different value as f(phi)
    RGB_Wrapper_Circle__Phi_Theta_Q, // lights dim around theta as a function of 1+|x|^(1/2)
    RGB_Wrapper_Circle__Phi_Theta_Solid_Value, // lights do not dim around theta
    RGB_Wrapper_Circle__Phi_Theta_Persistence_Rainbow,
    RGB_Wrapper_Circle__Phi_Theta_Persistence_Solid,
} RGB_Wrapper_Circle_Type_e;


typedef struct {
    float hue;
    uint8_t value;
} RGB_Wrapper_HV_Color_s;


typedef struct {
    RGB_Wrapper_Pixel_Type_e pixel_type;
    RGB_Wrapper_HV_Color_s colors[MAX_BUTTON_COLORS]; // for button or blink pattern pixels
    // button class pointer
    uint8_t blink_pulse_or_rainbow_max_value;
    
    /** this will either scale from 0 to 1 for blink, or 0 to "(float)blink_pulse_or_rainbow_max_value"
     * for pulse LED pixel types. */
    float blink_pulse_value_to_increment_per_update;
    float rainbow_hue_to_increment_per_update;
    float blink_pulse_accumulator;
    bool is_blink_pulse_incrementing_up;
    
    bool normal_pixel_update(void);
    
    RGB_Wrapper_HV_Color_s color_buf;
    uint8_t last_adjacent_idx_of_similar_pixel_setting; 
} RGB_Wrapper_Pixel_Info_s;



typedef enum {
    RGB_Wrapper_Status_RGB_DRIVER_UNLINKED, // init peripherals and data 
    RGB_Wrapper_Status_INIT_PERIPH, // init peripherals and data 
    RGB_Wrapper_Status_INIT_FAIL, 
    RGB_Wrapper_Status_INIT_PIXEL_TYPES, // configure each pixel (rainbow, blink, circle, etc)
    RGB_Wrapper_Status_UPDATING_LED_BUF, 
    RGB_Wrapper_Status_PROCESSING_PIXEL_BITSTREAM,  
    RGB_Wrapper_Status_SEND_OUT_PIXEL_BITSTREAM, 
} RGB_Wrapper_Status_e;


class RGB_Wrapper
{
    public:
        RGB_Wrapper(uint8_t num_of_leds_to_cmd_);

        void link_ws2812b_rgb_led_driver_class(WS2812B_RGB_LED_Strip *led_drv_class_ptr);        

        /** initialize data to default values and initialize peripherals
         * needed to bitbang the WS2812B LED's via DMA writes to the GPIO
         * pin setting registers. */
        RGB_Wrapper_Status_e init_periphs_for_WS2812B(void);

        RGB_Wrapper_Status_e finish_initializing_pixel_types(void);
        
        RGB_Wrapper_Status_e set_blink_pixel_settings(uint8_t blink_start_idx, \
                                                        uint8_t blink_end_idx, \
                                                        float increment_start_point, \
                                                        float increment_update_val, \
                                                        bool increment_direction_start, \
                                                        RGB_Wrapper_HV_Color_s color);
        
        RGB_Wrapper_Status_e set_rainbow_pixel_settings(uint8_t rainbow_start_idx, \
                                                        uint8_t rainbow_end_idx, \
                                                        uint8_t max_value, \
                                                        float hue_increment_start_point, \
                                                        float hue_increment_update_val, \
                                                        bool hue_increment_direction_start);

        RGB_Wrapper_Status_e set_circle_pixel_settings(uint8_t circle_start_idx, \
                                                        uint8_t circle_end_idx, \
                                                        RGB_Wrapper_Circle_Type_e circle_type, \
                                                        float phi_upright_hue_or_value, \
                                                        float light_persistence_coefficient, \
                                                        float led_width_of_inv_sqrt_func_m1_to_p1);
        
        RGB_Wrapper_Status_e set_button_pixel_settings(uint8_t button_start_idx, \
                                                        uint8_t button_end_idx, \
                                                        // button class pointer,
                                                        RGB_Wrapper_HV_Color_s color_1, \
                                                        RGB_Wrapper_HV_Color_s color_2, \
                                                        RGB_Wrapper_HV_Color_s color_3);
        
        RGB_Wrapper_Status_e set_pulse_pixel_settings(uint8_t pulse_start_idx, \
                                                        uint8_t pulse_end_idx, \
                                                        float val_increment_start_point, \
                                                        float val_increment_update_val, \
                                                        bool val_increment_direction_start, \
                                                        RGB_Wrapper_HV_Color_s color_with_max_value);


        RGB_Wrapper_Status_e update_pixels(void);
        RGB_Wrapper_Status_e update_pixels(float pdm_phi);
        RGB_Wrapper_Status_e update_pixels(float pdm_phi, float pdm_theta);

        
    private:
        WS2812B_RGB_LED_Strip *led_strip_drv_ptr = NULL;
        uint8_t num_of_leds_to_cmd;
        WS2812B_Status_e driver_status;
        
        RGB_Wrapper_Status_e status = RGB_Wrapper_Status_RGB_DRIVER_UNLINKED;

        /** for a circle of 6 LED's, the indexing will be as follows:
         * [0] = directly on top of the first LED in the circle, also 
         * equaling 0+offset radians of pendulum base angle (theta). 
         * 
         * for an angle of pi, that would correspond to 30/2 = 15, which
         * is right on top of an LED at index 15.  */
        float theta_offset;

        /** for an angle of "theta+offset = pi" and an LED count of 5,
         * this value would be 5/2pi*pi = 5/2 = 2.5.  The peak
         * point of the "Q" would be halfway between LED[2] and LED[3] */
        float pdm_angle_index; 

        float persistence_coeff;
        
        RGB_Wrapper_Pixel_Info_s pixel_data[WS2812B_MAX_NUM_OF_LEDS];

};

#endif // __RGB_WRAPPER_HPP

