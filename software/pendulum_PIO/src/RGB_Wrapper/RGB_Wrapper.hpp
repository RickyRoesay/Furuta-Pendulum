#ifndef __RGB_WRAPPER_HPP
#define __RGB_WRAPPER_HPP

#include "Arduino.h"
#include <SPI.h>
#include "Drivers/WS2812B_RGB_LED_Strip/WS2812B_RGB_LED_Strip.hpp" // RGB LED's
#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc

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



/** this is meant to be a subset of
 * the HSV (Hue, Saturation, Value) color model. 
 * We are not using saturation, since it's almost impossible to 
 * change saturation on a single pixel RGBs. */
typedef struct {
    float hue;
    // ignoring saturation since it's very difficult to modulate that on single pixel LED's.
    uint8_t value;
} RGB_Wrapper_HV_Color_s;


typedef enum : uint8_t {
    RGB_Wrapper__Unused_Pixel = 0,  // default value
    RGB_Wrapper__Button_Pixel,  // blink a color when pressed or depressed, takes button state as input
    RGB_Wrapper__Blink_Pixel, // blinking a solid color between off and a static brightness/value
    RGB_Wrapper__Pulse_Pixel, // pulsing a solid color different brightnesses
    RGB_Wrapper__Rainbow_Pixel, // cycling through the rainbow
    RGB_Wrapper__Circle_Pixel, // one of the LED's that iluminate around the pendulum 
} RGB_Wrapper_Pixel_Type_e;


typedef enum : uint8_t {
    RGB_Wrapper_Circle__Unset = 0,
    RGB_Wrapper_Circle__Phi_Rainbow, // whole circle is illuminated a different color as a function of phi
    RGB_Wrapper_Circle__Phi_Pulse, // whole circle is illuminated a solid color but different value as f(phi)
    RGB_Wrapper_Circle__Phi_Rainbow_Theta_Solid_Value, // H,V "Hue" of LED's change as f(phi), static H,V Value for all 
    RGB_Wrapper_Circle__Phi_Rainbow_Theta_Q, //  H,V "Hue" of LED's change as f(phi), lights dim around theta as a function of 1+|x|^(1/2)
    RGB_Wrapper_Circle__Phi_Value_Theta_Solid_Value, // H,V "Value" of LED's change as f(phi), static H,V Value for all 
    
    /** LED's right under the pendulum illuminate as 
     * with hue being a function of Phi, and colors stay
     * illuminated for a slight time after the pendulum moves so that it decays in 
     * brightness. */
    RGB_Wrapper_Circle__Phi_Theta_Persistence_Rainbow,
    RGB_Wrapper_Circle__Phi_Theta_Persistence_Solid, // same thing as rainbow but "hue = static" and "value = f(phi)"
} RGB_Wrapper_Circle_Type_e;

#if 0
/** List of possible state values to "modulate" the Hue of the LED's
 * circling/tracking the pendulum's base angle in a circle. */
typedef enum : uint8_t {
    RGB_Wrapper_HV_Hue_Mod__Pendulum_Angle, 
    RGB_Wrapper_HV_Hue_Mod__Base_Ang_Vel, 
    RGB_Wrapper_HV_Hue_Mod__Num_Of_Types, 
} RGB_Wrapper_HV_Value_Modulation_Type_e;


/** List of possible state values to "modulate" the value */
typedef enum : uint8_t {
    RGB_Wrapper_HV_Value_Decay__None, // All of the circle LED's are on and have the same HV Value

    
    RGB_Wrapper_HV_Value_Decay__Static, // 
    RGB_Wrapper_HV_Value_Decay__, 
    RGB_Wrapper_HV_Value_Decay__, 
    RGB_Wrapper_HV_Value_Decay__, 
    RGB_Wrapper_HV_Value_Decay__, 
    
    /** LED's right under the pendulum illuminate as 
     * with hue being a function of Phi, and colors stay
     * illuminated for a slight time after the pendulum moves so that it decays in 
     * brightness. */
    RGB_Wrapper_Circle__Phi_Theta_Persistence_Rainbow,
    RGB_Wrapper_Circle__Phi_Theta_Persistence_Solid, // same thing as rainbow but "hue = static" and "value = f(phi)"
} RGB_Wrapper_HV_Value_Modulation_Type_e;
#endif 

typedef enum : uint8_t {
    RGB_Wrapper_Status_RGB_DRIVER_UNLINKED, // init peripherals and data 
    RGB_Wrapper_Status_INIT_PERIPH, // init peripherals and data 
    RGB_Wrapper_Status_INIT_FAIL, 
    RGB_Wrapper_Status_INIT_PIXEL_TYPES, // configure each pixel (rainbow, blink, circle, etc)
    RGB_Wrapper_Status_UPDATING_LED_BUF, 
    RGB_Wrapper_Status_PROCESSING_PIXEL_BITSTREAM,  
} RGB_Wrapper_Status_e;



typedef enum : uint8_t {
    RGB_Wrapper_Theta_Tracking_Direction__NORMAL, 
    RGB_Wrapper_Theta_Tracking_Direction__INVERSE
} RGB_Wrapper_Theta_Tracking_Direction;



typedef struct {
    RGB_Wrapper_Pixel_Type_e pixel_type;
    RGB_Wrapper_HV_Color_s colors[MAX_BUTTON_COLORS]; // for button or blink pattern pixels
    // button class pointer

    /** Accumulation for different LED modes: 
     * 
     * /////////////// BLINK: ////////////
     * - Scales between 0 and 1, unit is in in time.
     * - When the accumulator >= 1.0, LED turns on and accumulator
     * is incremented down until the accumulator is < 0.
     * When accumulator < 0, the LED turns off and the accumuator starts 
     * incrementing up. 
     * - Static Hue and Value of the led when it's ON will be stored 
     * in colors[0] on pixel setting initialization.
     * 
     * /////////////// PULSE: ////////////
     * - Scales between 0 and blink_pulse_hue_max_accumulator_val.
     * - Units are in "value" (HSV), aka LED brightness.
     * - Accumulator has similar up/down incrementation as blink,
     * but the difference is that the accumulator is converted to a 
     * a uint8 and sent as the LED pixel's "Value" in Hue/Value.  
     * - Max value and static hue will be stored in colors[0] on initialization.
     * 
     * /////////////// RAINBOW: ////////////
     * - Right now it simply increments up or down based on the polarity
     * of "blink_pulse_hue_increment_val" and cycles all the way around the rainbow.
     * - Units are in "Hue" (HSV), aka rainbow color as a function of 0-360 degrees.
     * - Static value will be stored in colors[0] on initialization. */
    float blink_pulse_hue_max_accumulator_val;
    float blink_pulse_hue_accumulator;
    float blink_pulse_hue_increment_val;

    bool is_blink_pulse_incrementing_up;

    float persistence_HV_value;
    
    RGB_Wrapper_HV_Color_s color_buf;
    uint8_t last_adjacent_idx_of_similar_pixel_setting; 
} RGB_Wrapper_Pixel_Info_s;





class RGB_Wrapper
{
    public:
        RGB_Wrapper(uint8_t num_of_leds_to_cmd_);

        void link_ws2812b_rgb_led_driver_class(WS2812B_RGB_LED_Strip *led_drv_class_ptr);        

        /** initialize data to default values and initialize peripherals
         * needed to bitbang the WS2812B LED's via DMA writes to the GPIO
         * pin setting registers. */
        RGB_Wrapper_Status_e init_periphs_for_WS2812B(void);
        
        RGB_Wrapper_Status_e set_theta_offset_in_radians(float pdm_theta_offset_rad, 
                                                        RGB_Wrapper_Theta_Tracking_Direction tracking_direction);

        /** Calling this function marks the end of the "INIT_PIXEL_TYPES"
         * state as long as all of the led's to command have their 
         * settings configured. This must be called after configuring or
         * reconfiguring the pixel settings. */
        RGB_Wrapper_Status_e finish_initializing_pixel_types(void);
        
        RGB_Wrapper_Status_e set_blink_pixel_settings(uint8_t blink_start_idx, \
                                                        uint8_t blink_end_idx, \
                                                        float increment_start_point, \
                                                        float increment_update_val, \
                                                        bool increment_direction_start, \
                                                        RGB_Wrapper_HV_Color_s color);
        
        RGB_Wrapper_Status_e set_pulse_pixel_settings(uint8_t pulse_start_idx, \
                                                        uint8_t pulse_end_idx, \
                                                        float val_increment_start_point, \
                                                        float val_increment_update_val, \
                                                        bool val_increment_direction_start, \
                                                        RGB_Wrapper_HV_Color_s color_with_max_value);
        
        RGB_Wrapper_Status_e set_rainbow_pixel_settings(uint8_t rainbow_start_idx, \
                                                        uint8_t rainbow_end_idx, \
                                                        uint8_t brightness_value, \
                                                        float hue_start_point, \
                                                        float hue_increment_update_val);

        RGB_Wrapper_Status_e set_circle_pixel_settings(uint8_t circle_start_idx_, \
                                                        uint8_t circle_end_idx_, \
                                                        RGB_Wrapper_Circle_Type_e circle_type_, \
                                                        float phi_upright_hue_or_value_, \
                                                        float light_persistence_decay_val_, \
                                                        float num_of_circle_leds_on_at_once_,
                                                        RGB_Wrapper_HV_Color_s init_color_data);
        
        RGB_Wrapper_Status_e set_button_pixel_settings(uint8_t button_start_idx, \
                                                        uint8_t button_end_idx, \
                                                        // button class pointer,
                                                        RGB_Wrapper_HV_Color_s color_1, \
                                                        RGB_Wrapper_HV_Color_s color_2, \
                                                        RGB_Wrapper_HV_Color_s color_3);

        RGB_Wrapper_Status_e update_pixels(float pdm_phi, float pdm_theta);


    private:

        RGB_Wrapper_Pixel_Info_s pixel_data[WS2812B_MAX_NUM_OF_LEDS];
        WS2812B_RGB_LED_Strip *led_strip_drv_ptr = NULL;
        uint8_t num_of_leds_to_cmd;
        
        RGB_Wrapper_Status_e status = RGB_Wrapper_Status_RGB_DRIVER_UNLINKED;
        

        /** Circle Pixel data */
        RGB_Wrapper_Circle_Type_e circle_type = RGB_Wrapper_Circle__Unset;
        uint8_t num_of_circle_pixels_u8;
        float num_of_circle_pixels_f32; // analogous to 360 degrees of a circle, but in units of leds
        uint8_t circle_start_idx;
        uint8_t circle_end_idx;
        float num_of_circle_leds_on_at_once; 
        float light_persistence_decay_val;
        float half_num_of_circle_leds_on_at_once; // math related value

        /** Pendulum Tracking Info: */
        float theta_offset_in_radians = _PI / 5.0f * 4.0f;
        /** if phi_upright_hue_or_value is entered as a negative value,
         * that means there will be 0 brightness at phi = 0, and full brightness
         * (or whatever the absolute magnitude of phi_upright_hue_or_value is) at phi = PI. */
        float phi_upright_hue_or_value; // for either hue or value
        uint8_t circle_led_on_start_c_idx;
        uint8_t circle_led_on_end_c_idx;
        RGB_Wrapper_Theta_Tracking_Direction theta_tracking_direction = RGB_Wrapper_Theta_Tracking_Direction__NORMAL;

        

        float theta_with_led_offset_in_radians;
        float theta_in_circle_idx;
        float phi_in_hue_or_value; 
        
        
        inline void update_circle_tracking_info(float phi_in_radians, float theta_in_radians);
        inline void update_circle_pixels(float phi_in_radians, float theta_in_radians, uint8_t* led_buf_idx);
        inline uint8_t get_hv_value_based_on_distance_from_pdm(uint8_t led_circle_idx, uint8_t max_value);

};


#endif // __RGB_WRAPPER_HPP

