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
    RGB_Wrapper__Pulse_Pixel, // blinking a solid color different brightnesses
    RGB_Wrapper__Rainbow_Pixel, // cycling through the rainbow
    RGB_Wrapper__Circle_Pixel, // one of the LED's that iluminate around the pendulum 
} RGB_Wrapper_Pixel_Type_e;


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

    bool rgb_wrapper_pixel_update(void);

    RGB_Wrapper_HV_Color_s color_buf;
} RGB_Wrapper_Pixel_Info_s;


class RGB_Wrapper
{
    public:
        RGB_Wrapper();
    
        WS2812B_Status_e init_rgb_strip();

        
    private:

};

#endif // __RGB_WRAPPER_HPP

