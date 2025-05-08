#ifndef __WS2812B_HPP
#define __WS2812B_HPP

#include "Arduino.h"
#include <SPI.h>
//#include "stm32f405xx.h"


/** The order of operations for this concurrent driver is as follows:
 *      1). Write to pixel buffer, actually choosing what the next "frame" will be.
 *      2). Process Bitfield array until status is "WS2812B_READY_TO_UPLOAD_BITSTREAM."
 *      3). Trigger dma transfer using "write_bitfield_array_via_dma" function.
 * 
 * NOTE: This driver utilizes DMA2, Stream 1, Channel 7. This means we use
 * TIM8 Update to trigger the dma bumps.
 * 
 * IMPORTANT: NOTE: DMA1 CAN NOT WRITE TO GPIO
 * on the STM32F4!!!!  A helpful guide that goes into more detail can be found using 
 * this link:  http://www.efton.sk/STM32/gotcha/g30.html
 */



/** Only up to 50 LED's are supported.  This number
 * was chosen arbitrarily as a static/compile time
 * limit to simplify implementation. */
#define WS2812B_MAX_NUM_OF_LEDS 100 //50


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


 

typedef enum : uint8_t {
    WS2812B_BITSTREAM_0 = 0, 
    WS2812B_BITSTREAM_1 = 1, 
    WS2812B_NUM_OF_BITSTREAMS = 2
} WS2812B_Bitstream_Index_e;

/** NOTE: It is possible to process pixel buffer and
 * the next bitstream data while the DMA transfer is active.
 * 
 * However, when the DMA transfer is active, the status will 
 * always be "WS2812B_TRANSMITTING_DATA." */
typedef enum {
    WS2812B_INIT_PERIPHERALS, 
    WS2812B_INIT_FAIL, 
    WS2812B_WAITING_TO_PROCESS_PIXEL_BITSTREAM,  
    WS2812B_TRANSMITTING_DATA, 
    WS2812B_READY_TO_UPLOAD_BITSTREAM, 
} WS2812B_Status_e;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} WS2812B_Led_Pixel_Colors_s;




class WS2812B_RGB_LED_Strip
{
    public:
        WS2812B_RGB_LED_Strip(int DO_gpio_);
    
        WS2812B_Status_e init_dma_and_timer_peripherals(uint8_t num_of_leds_to_cmd);

        /** Will only take effect if a valid parameter is sent 
         * (within 1 to WS2812B_MAX_NUM_OF_LEDS).
         * Returns false if there was an issue initializing peripherals, the
         * DO_gpio_ pin is not valid (passed into class constructor) or an invalid 
         * parameter was passed (0, > WS2812B_MAX_NUM_OF_LEDS). */
        bool update_num_of_leds_to_cmd(uint8_t num_of_leds_to_cmd);

        WS2812B_Status_e get_status(void);

        void modify_pixel_buffer_all_leds(uint8_t red, uint8_t green, uint8_t blue);
        void modify_pixel_buffer_all_leds(float hue_degrees, uint8_t value);

        bool modify_pixel_buffer_single(uint8_t buffer_idx,
                                        uint8_t red, uint8_t green, uint8_t blue);
        bool modify_pixel_buffer_single(uint8_t buffer_idx,
                                        float hue_degrees, uint8_t value); // max value == 255
             
                                            
        /** returns the status of the driver.  This function
         * allows for more flexible speeds of how fast the bitfield is 
         * processed. */
        WS2812B_Status_e process_bitfield_array(uint32_t num_of_pixels_to_process);
        
        
        /** triggers DMA to start transfer of the bitfield array into
         * the GPIO peripheral BSRR register, which then pulses the 
         * gpio pin configured as the LED data out. */
        WS2812B_Status_e write_bitfield_array_via_dma(void);

        void set_gpio_pin_level(bool pin_level); // used for testing
        
    private:
        /** Return false if the ulPin is not a valid digital pin on the MCU. */
        bool set_bit_masks_and_dma_dest_pointer_from_arduino_pin_macro(uint32_t ulPin);
        
        inline void write_pixel_data_to_bitstream(uint32_t *ptr_to_bitstream, 
                                            WS2812B_Led_Pixel_Colors_s * ptr_to_pixel_info);
        
        inline void write_bit_data_to_bitstream(uint32_t *ptr_to_bitstream, uint8_t bit_level);

        inline void write_reset_data_to_bitstream(WS2812B_Bitstream_Index_e bitstream_number);
            
        inline bool is_dma_transfer_in_progress(void);

        inline WS2812B_Led_Pixel_Colors_s hue_value_to_rgb(float hue_degrees, uint8_t value);

        int DO_gpio;

        GPIO_TypeDef *gpio_port_ptr;
        HardwareTimer timer_handle; // used for triggering dma every ~300ns
        DMA_HandleTypeDef dma_handle;
        
        uint32_t gpio_dma_pin_set_mask; 
        uint32_t gpio_dma_pin_clear_mask;

        WS2812B_Status_e status = WS2812B_INIT_PERIPHERALS;

        WS2812B_Led_Pixel_Colors_s led_pixel_buf[WS2812B_MAX_NUM_OF_LEDS];
        uint8_t next_led_buf_idx_to_process;

        /** This is the number of LED's that we intend to write to 
         * per update, and does not necessarily have to be 50.  It 
         * can be any value between 1 and 50 inclusively. */
        uint8_t active_led_num;

        uint8_t bitstream_idx_for_led_buf = WS2812B_BITSTREAM_1;
        uint8_t bitstream_idx_for_dma = WS2812B_BITSTREAM_0;
        
        uint32_t bitstream[WS2812B_NUM_OF_BITSTREAMS][WS2812B_NUM_OF_GPIO_WRITES_TOTAL];

};


#endif // __WS2812B_HPP
