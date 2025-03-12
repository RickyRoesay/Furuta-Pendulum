#ifndef __WS2812B_HPP
#define __WS2812B_HPP

#include "Arduino.h"
#include <SPI.h>

/** Only up to 50 LED's are supported.  This number
 * was chosen arbitrarily as a static/compile time
 * limit to simplify implementation. */
#define WS2812B_MAX_NUM_OF_LEDS 50

typedef enum {
    WS2812B_INIT_PERIPHERALS, 
    WS2812B_INIT_FAIL, 
    WS2812B_TRANSMITTING_DATA, 
    WS2812B_IDLE, 
} WS2812B_Status_e;


typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} WS2812B_Led_Pixel_Info_s;


class WS2812B_RGB_LED_Strip
{
    public:
        WS2812B_RGB_LED_Strip(int DO_gpio) : DO_gpio_(DO_gpio) {}
        
        bool init(uint8_t num_of_leds_to_cmd = 0);

        /** Will only take effect if a valid parameter is sent 
         * (within 1 to WS2812B_MAX_NUM_OF_LEDS).
         * Returns false if there was an issue initializing peripherals, the
         * DO_gpio pin is not valid (passed into class constructor) or an invalid 
         * parameter was passed (0, >WS2812B_MAX_NUM_OF_LEDS). */
        bool update_num_of_leds_to_cmd(uint8_t num_of_leds_to_cmd);

        WS2812B_Status_e get_status();

        inline bool modify_pixel_buffer_single(uint8_t buffer_idx, WS2812B_Led_Pixel_Info_s * ptr_to_new_pixel_info);
        inline void modify_pixel_buffer_all_leds(WS2812B_Led_Pixel_Info_s * ptr_to_new_pixel_info);
        inline bool modify_pixel_buffer_multi(uint8_t buffer_idx_start, 
                                            uint8_t num_of_pixels_in_buf_to_update,
                                            WS2812B_Led_Pixel_Info_s * ptr_to_new_pixel_info);
        
        
    private:
        
        int DO_gpio_;

        WS2812B_Led_Pixel_Info_s led_pixel_buf[WS2812B_MAX_NUM_OF_LEDS];

        /** There needs to be 2 different variables for active and requested 
         * led num since we may be using active_led_num as an index limit
         * when a new number of leds to be controlled is requested. */
        uint16_t active_led_num;
        uint16_t requested_led_num;

};


#endif // __WS2812B_HPP
