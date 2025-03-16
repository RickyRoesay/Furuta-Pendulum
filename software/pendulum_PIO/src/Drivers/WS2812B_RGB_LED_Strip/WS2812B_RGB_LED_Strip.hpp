#ifndef __WS2812B_HPP
#define __WS2812B_HPP

#include "Arduino.h"
#include <SPI.h>
//#include "stm32f405xx.h"

/** Only up to 50 LED's are supported.  This number
 * was chosen arbitrarily as a static/compile time
 * limit to simplify implementation. */
#define WS2812B_MAX_NUM_OF_LEDS 8


#define WS2812B_NUM_OF_BITS_PER_PIXEL  24
#define WS2812B_NUM_OF_GPIO_WRITES_PER_BIT  4
/** When writing with 4 bytes per dma bump this is 4800 */
#define WS2812B_MAX_NUM_OF_GPIO_WRITES_FOR_LED (WS2812B_MAX_NUM_OF_LEDS \
                                                * WS2812B_NUM_OF_BITS_PER_PIXEL \
                                                * WS2812B_NUM_OF_GPIO_WRITES_PER_BIT)
                               
#define WS2812B_NUM_OF_GPIO_WRITES_PER_PIXEL (WS2812B_NUM_OF_BITS_PER_PIXEL \
                                            * WS2812B_NUM_OF_GPIO_WRITES_PER_BIT)

/** The reset time for the LED chain is at least 50 microseconds.
 * To add margin and make it a number that's divisible by the 
 * used timing quantum (~0.3us) we get 75 microseconds. 
 * 75us / ~0.3us = 250 */
#define WS2812B_NUM_OF_BITS_PER_RESET 250                 

/** this ends up being 5050 */
#define WS2812B_NUM_OF_GPIO_WRITES_TOTAL (WS2812B_MAX_NUM_OF_GPIO_WRITES_FOR_LED \
                                          + WS2812B_NUM_OF_BITS_PER_RESET)


/** DMA1, Stream 6, Channel 6 */
 

typedef enum : uint8_t {
    WS2812B_BITSTREAM_0 = 0, 
    WS2812B_BITSTREAM_1 = 1, 
    WS2812B_NUM_OF_BITSTREAMS = 2
} WS2812B_Bitstream_Index_e;

typedef enum {
    WS2812B_INIT_PERIPHERALS, 
    WS2812B_INIT_FAIL, 
    WS2812B_WAITING_FOR_PIXEL_DATA,  
    WS2812B_TRANSMITTING_DATA, 
    WS2812B_READY_TO_UPLOAD_BITSTREAM, 
} WS2812B_Status_e;


typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} WS2812B_Led_Pixel_Info_s;


class WS2812B_RGB_LED_Strip
{
    public:
        WS2812B_RGB_LED_Strip(int DO_gpio_);
    
        WS2812B_Status_e init_dma_and_timer_peripherals(uint8_t num_of_leds_to_cmd);

        /** Will only take effect if a valid parameter is sent 
         * (within 1 to WS2812B_MAX_NUM_OF_LEDS).
         * Returns false if there was an issue initializing peripherals, the
         * DO_gpio_ pin is not valid (passed into class constructor) or an invalid 
         * parameter was passed (0, >WS2812B_MAX_NUM_OF_LEDS). */
        bool update_num_of_leds_to_cmd(uint8_t num_of_leds_to_cmd);

        WS2812B_Status_e get_status();

        void modify_pixel_buffer_all_leds(uint8_t red, uint8_t green, uint8_t blue);

        inline bool modify_pixel_buffer_single(uint8_t buffer_idx,
                                                uint8_t red, uint8_t green, uint8_t blue);
             
                                            
        /** returns the status of the driver.  This function
         * allows for more flexible speeds of how fast the bitfield is 
         * processed. */
        WS2812B_Status_e process_bitfield_array(uint32_t num_of_pixels_to_process);
        
        
        /** triggers DMA to start transfer of the bitfield array into
         * the GPIO peripheral BSRR register, which then pulses the 
         * gpio pin configured as the LED data out. */
        bool write_bitfield_array_via_dma(void);

        
        void set_gpio_pin_level(bool pin_level); // used for testing
        
        
    private:
        /** Return false if the ulPin is not a valid digital pin on the MCU. */
        bool set_bit_masks_and_dma_dest_pointer_from_arduino_pin_macro(uint32_t ulPin);
        
        void write_pixel_data_to_bitstream(uint32_t *ptr_to_bitstream, 
        WS2812B_Led_Pixel_Info_s * ptr_to_pixel_info);
        
        inline void write_bit_data_to_bitstream(uint32_t *ptr_to_bitstream, uint8_t bit_level);
        inline void write_reset_data_to_bitstream(WS2812B_Bitstream_Index_e bitstream_number);
            
        inline bool is_dma_transfer_in_progress(void);

        int DO_gpio;

        GPIO_TypeDef *gpio_port_ptr;
        HardwareTimer timer_handle; // used for triggering dma every ~300ns
        DMA_HandleTypeDef dma_handle;
        
        uint32_t gpio_dma_pin_set_mask; 
        uint32_t gpio_dma_pin_clear_mask;

        WS2812B_Status_e status = WS2812B_INIT_PERIPHERALS;

        
        WS2812B_Led_Pixel_Info_s led_pixel_buf[WS2812B_MAX_NUM_OF_LEDS];

        

        /** There needs to be 2 different variables for active and requested 
         * led num since we may be using active_led_num as an index limit
         * when a new number of leds to be controlled is requested. */
        uint8_t active_led_num;
        uint8_t requested_led_num;

        uint8_t next_led_buf_idx_to_process;
        
        uint8_t bitstream_idx_for_led_buf = WS2812B_BITSTREAM_1;
        uint8_t bitstream_idx_for_dma = WS2812B_BITSTREAM_0;
        
        uint32_t bitstream[WS2812B_NUM_OF_BITSTREAMS][WS2812B_NUM_OF_GPIO_WRITES_TOTAL];

};


#endif // __WS2812B_HPP
