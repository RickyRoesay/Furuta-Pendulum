
#include "WS2812B_RGB_LED_Strip.hpp"
#include "Arduino.h"
#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"
#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc


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


/** NOTE: These values are for the STM32F4xx MCU and may differ for other variants.
 * TRM:SECION: 2.3 Memory Map. */
#define LOWEST_ADDRESS_OF_GPIO_RAM      0x40020000
#define HIGHEST_ADDRESS_OF_GPIO_RAM     0x40022BFF


/** By default, the timer is configured with prescaler and clock divider
 * as 1 during class construction.  We only need to set the overflow to 50 counts
 * to get the desired overflow frequency of around 300ns. */
#define TMR_OVF_VAL_TO_GET_300NS_PRD_WITH_NO_CLKDIV_OR_PSC 50

#define BIT_LEVEL_HIGH 1
#define BIT_LEVEL_LOW 0




///////////////////////////////////////////////////////////////////////////////////////

WS2812B_RGB_LED_Strip::WS2812B_RGB_LED_Strip(int DO_gpio_)
{
  DO_gpio = DO_gpio_;
  timer_handle = HardwareTimer(TIM8);
}

///////////////////////////////////////////////////////////////////////////////////////

WS2812B_Status_e  WS2812B_RGB_LED_Strip::init_dma_and_timer_peripherals(uint8_t num_of_leds_to_cmd)
{
  if(num_of_leds_to_cmd > WS2812B_MAX_NUM_OF_LEDS
  || set_bit_masks_and_dma_dest_pointer_from_arduino_pin_macro(DO_gpio) == false
  || timer_handle.getHandle()->Instance == nullptr) //timer class construction failed if = nullptr
  {
    status = WS2812B_INIT_FAIL;
  }
  else
  {
    pinMode(DO_gpio, OUTPUT);

    active_led_num = num_of_leds_to_cmd;
    
    /** Somewhere during init, between when the WS2812B_RGB_LED_Strip 
     * is called and when this init function is called, it seems the 
     * TIM8EN bit in the APB2ENR is getting stomped on and reset back 
     * to 0.  Simply re-enable the timer peripheral before continuing. */
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    /** By default, the timer is configured with prescaler and clock divider
     * as 1 during class construction.  We only need to set the overflow to 50 counts
     * to get the desired overflow frequency of around 300ns. */
    timer_handle.setOverflow(TMR_OVF_VAL_TO_GET_300NS_PRD_WITH_NO_CLKDIV_OR_PSC, TICK_FORMAT);
    timer_handle.getHandle()->Instance->DIER = 1 << 8; // set UDE update DMA req enable bit HIGH
    timer_handle.resume();

    __HAL_RCC_DMA2_CLK_ENABLE();

    /** Trigger on TIM5 UP */
    dma_handle.Instance = DMA2_Stream1;
    dma_handle.Init.Channel = DMA_CHANNEL_7;

    dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    
    dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_handle.Init.MemInc = DMA_MINC_ENABLE;
    
    /** PSIZE and MSIZE */
    dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

    dma_handle.Init.Mode = DMA_NORMAL;

    dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    
    dma_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    /** FIFOThreshold is not used, will be overwritten by hw at
     * the start of a direct mode DMA transfer*/

    dma_handle.Init.MemBurst = DMA_MBURST_SINGLE;
    dma_handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
    
    
    HAL_DMA_Init(&dma_handle);
    
    /** mem source address is set right before enabling dma transfers in
     * the "write_bitfield_array_via_dma" function. */
    dma_handle.Instance->PAR = (uint32_t)&(gpio_port_ptr->BSRR);

    write_reset_data_to_bitstream(WS2812B_BITSTREAM_0);
    write_reset_data_to_bitstream(WS2812B_BITSTREAM_1);
  }
  return status;
}



inline bool WS2812B_RGB_LED_Strip::is_dma_transfer_in_progress(void)
{
  return (dma_handle.Instance->CR & 0x00000001);
}




bool WS2812B_RGB_LED_Strip::modify_pixel_buffer_single(uint8_t buffer_idx, 
                                                  uint8_t red, uint8_t green, uint8_t blue)
{
  if(buffer_idx > active_led_num)
    return 0;

  led_pixel_buf[buffer_idx].red = red;
  led_pixel_buf[buffer_idx].green = green;
  led_pixel_buf[buffer_idx].blue = blue;
  
  buffer_idx++;

  /** Reset to 0 signaling the buffer has been updated since the last time 
   * any WS2812B driver functions have been called. */
  next_led_buf_idx_to_process = 0;

  return 1;
}
bool WS2812B_RGB_LED_Strip::modify_pixel_buffer_single(uint8_t buffer_idx, 
                                                    float hue_degrees, uint8_t value)
{
  if(buffer_idx > active_led_num \
  || hue_degrees > 360.0f \
  || hue_degrees < 0.0f)
    return 0;

  WS2812B_Led_Pixel_Colors_s pixel_colors = hue_value_to_rgb(hue_degrees, value);

  led_pixel_buf[buffer_idx].red = pixel_colors.red;
  led_pixel_buf[buffer_idx].green = pixel_colors.green;
  led_pixel_buf[buffer_idx].blue = pixel_colors.blue;
  
  buffer_idx++;

  /** Reset to 0 signaling the buffer has been updated since the last time 
   * any WS2812B driver functions have been called. */
  next_led_buf_idx_to_process = 0;

  return 1;
}




/** This function takes ~1.3us for 9 led's */
void WS2812B_RGB_LED_Strip::modify_pixel_buffer_all_leds(uint8_t red, uint8_t green, uint8_t blue)
{
  for(uint8_t tmp_pixel_buf_idx = 0; tmp_pixel_buf_idx < active_led_num; tmp_pixel_buf_idx++)
  {
    led_pixel_buf[tmp_pixel_buf_idx].red = red;
    led_pixel_buf[tmp_pixel_buf_idx].green = green;
    led_pixel_buf[tmp_pixel_buf_idx].blue = blue;
  }
  
  /** Reset to 0 signaling the buffer has been updated since the last time 
   * any WS2812B driver functions have been called. */
  next_led_buf_idx_to_process = 0;
}
void WS2812B_RGB_LED_Strip::modify_pixel_buffer_all_leds(float hue_degrees, uint8_t value)
{
  WS2812B_Led_Pixel_Colors_s pixel_colors = hue_value_to_rgb(hue_degrees, value);

  for(uint8_t tmp_pixel_buf_idx = 0; tmp_pixel_buf_idx < active_led_num; tmp_pixel_buf_idx++)
  {
    led_pixel_buf[tmp_pixel_buf_idx].red = pixel_colors.red;
    led_pixel_buf[tmp_pixel_buf_idx].green = pixel_colors.green;
    led_pixel_buf[tmp_pixel_buf_idx].blue = pixel_colors.blue;
  }
  
  /** Reset to 0 signaling the buffer has been updated since the last time 
   * any WS2812B driver functions have been called. */
  next_led_buf_idx_to_process = 0;
}




bool WS2812B_RGB_LED_Strip::update_num_of_leds_to_cmd(uint8_t num_of_leds_to_cmd)
{
  bool tmp_ret_val;

  if(num_of_leds_to_cmd <= WS2812B_MAX_NUM_OF_LEDS)
  {
    active_led_num = num_of_leds_to_cmd; 
    next_led_buf_idx_to_process = 0; //reset value
    tmp_ret_val = true;
  }
  else
    tmp_ret_val = false;

  return tmp_ret_val;
}




/** This function takes ~4.4us */
WS2812B_Status_e  WS2812B_RGB_LED_Strip::process_bitfield_array(uint32_t num_of_pixels_to_process)
{
  uint32_t tmp_num_of_pixels_written_to = 0;

  uint32_t tmp_bitfield_idx_offset = WS2812B_NUM_OF_BITS_PER_RESET 
                                    + (WS2812B_NUM_OF_GPIO_WRITES_PER_PIXEL * next_led_buf_idx_to_process);

  if(status == WS2812B_INIT_FAIL)
    return WS2812B_INIT_FAIL;
    
  while(tmp_num_of_pixels_written_to < num_of_pixels_to_process
  && next_led_buf_idx_to_process < active_led_num)
  {
    write_pixel_data_to_bitstream(&bitstream[bitstream_idx_for_led_buf][tmp_bitfield_idx_offset], 
                                  &led_pixel_buf[next_led_buf_idx_to_process]);
    
    tmp_bitfield_idx_offset += WS2812B_NUM_OF_GPIO_WRITES_PER_PIXEL;
    tmp_num_of_pixels_written_to++;
    next_led_buf_idx_to_process++;
  }

  if(is_dma_transfer_in_progress() == true)
    status = WS2812B_TRANSMITTING_DATA;
  else if(next_led_buf_idx_to_process >= active_led_num)
  {
    status = WS2812B_READY_TO_UPLOAD_BITSTREAM;
    next_led_buf_idx_to_process = 0;
  }
  else
    status = WS2812B_WAITING_TO_PROCESS_PIXEL_DATA;

  return status;
}





bool WS2812B_RGB_LED_Strip::write_bitfield_array_via_dma()
{
  if((dma_handle.Instance->CR & 0x00000001)
  || status == WS2812B_INIT_FAIL)
    return 0;

  uint8_t tmp_bitstream_swap_space = bitstream_idx_for_led_buf;
  bitstream_idx_for_led_buf = bitstream_idx_for_dma;
  bitstream_idx_for_dma = tmp_bitstream_swap_space;
  
  /** Reset to 0 signaling the buffer has been updated since the last time 
   * any WS2812B driver functions have been called. */
  next_led_buf_idx_to_process = 0;

  /** DMA stream interrupts flags must be cleared before enabling DMA. */
  DMA2->LIFCR = 0xFFFFFFFF;

  dma_handle.Instance->NDTR = WS2812B_NUM_OF_GPIO_WRITES_TOTAL;
  dma_handle.Instance->M0AR = (uint32_t)bitstream[bitstream_idx_for_dma];
  __HAL_DMA_ENABLE(&dma_handle);

  status = WS2812B_TRANSMITTING_DATA;

  return 1;
}



void WS2812B_RGB_LED_Strip::set_gpio_pin_level(bool pin_level)
{
  if(pin_level == HIGH)
    gpio_port_ptr->BSRR = gpio_dma_pin_set_mask;
  else
    gpio_port_ptr->BSRR = gpio_dma_pin_clear_mask;
}





///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PRIVATE: FUNCTIONS: //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////


inline void WS2812B_RGB_LED_Strip::write_reset_data_to_bitstream(WS2812B_Bitstream_Index_e bitstream_number)
{
  if(bitstream_number != WS2812B_BITSTREAM_0 && bitstream_number != WS2812B_BITSTREAM_1)
    return;
  
  for(uint32_t tmp_idx = 0; tmp_idx < WS2812B_NUM_OF_BITS_PER_RESET; tmp_idx++)
  {
    bitstream[bitstream_number][tmp_idx] = gpio_dma_pin_clear_mask;
  }
}




inline void WS2812B_RGB_LED_Strip::write_bit_data_to_bitstream(uint32_t *ptr_to_bitstream, uint8_t bit_level)
{
  switch(bit_level)
  {
    case BIT_LEVEL_LOW:
      ptr_to_bitstream[0] = gpio_dma_pin_set_mask;
      ptr_to_bitstream[1] = gpio_dma_pin_clear_mask;
      ptr_to_bitstream[2] = gpio_dma_pin_clear_mask;
      ptr_to_bitstream[3] = gpio_dma_pin_clear_mask;
    break;
    
    default:
    case BIT_LEVEL_HIGH:
      ptr_to_bitstream[0] = gpio_dma_pin_set_mask;
      ptr_to_bitstream[1] = gpio_dma_pin_set_mask;
      ptr_to_bitstream[2] = gpio_dma_pin_set_mask;
      ptr_to_bitstream[3] = gpio_dma_pin_clear_mask;
    break;
  }
}




inline void WS2812B_RGB_LED_Strip::write_pixel_data_to_bitstream(uint32_t *ptr_to_bitstream, 
                                                                WS2812B_Led_Pixel_Colors_s * ptr_to_pixel_info)
{
  static const uint8_t tmp_bitfield_mask = 0x01;  
  uint8_t tmp_bit_level;
  uint32_t tmp_ptr_to_bitstream_offset = 0;
  
  uint8_t tmp_pixel_shift_reg = ptr_to_pixel_info->green;
  
  for(uint8_t tmp_bitshft_idx = 8; tmp_bitshft_idx > 0; tmp_bitshft_idx--)
  {
    tmp_bit_level = tmp_bitfield_mask & (tmp_pixel_shift_reg >> (tmp_bitshft_idx - 1));
    write_bit_data_to_bitstream(&ptr_to_bitstream[tmp_ptr_to_bitstream_offset], tmp_bit_level);
    tmp_ptr_to_bitstream_offset += 4;
  }
  
  tmp_pixel_shift_reg = ptr_to_pixel_info->red;
  
  for(uint8_t tmp_bitshft_idx = 8; tmp_bitshft_idx > 0; tmp_bitshft_idx--)
  {
    tmp_bit_level = tmp_bitfield_mask & (tmp_pixel_shift_reg >> (tmp_bitshft_idx - 1));
    write_bit_data_to_bitstream(&ptr_to_bitstream[tmp_ptr_to_bitstream_offset], tmp_bit_level);
    tmp_ptr_to_bitstream_offset += 4;
  }
  
  tmp_pixel_shift_reg = ptr_to_pixel_info->blue;
  
  for(uint8_t tmp_bitshft_idx = 8; tmp_bitshft_idx > 0; tmp_bitshft_idx--)
  {
    tmp_bit_level = tmp_bitfield_mask & (tmp_pixel_shift_reg >> (tmp_bitshft_idx - 1));
    write_bit_data_to_bitstream(&ptr_to_bitstream[tmp_ptr_to_bitstream_offset], tmp_bit_level);
    tmp_ptr_to_bitstream_offset += 4;
  }
}





bool WS2812B_RGB_LED_Strip::set_bit_masks_and_dma_dest_pointer_from_arduino_pin_macro(uint32_t ulPin)
{
  bool tmp_ret_val;

  PinName tmp_pn = digitalPinToPinName(ulPin);

  gpio_port_ptr = get_GPIO_Port(STM_PORT(tmp_pn));

  uint32_t tmp_pinmask_as_raw_16bits_unshifted = STM_LL_GPIO_PIN(tmp_pn);

  gpio_dma_pin_set_mask = tmp_pinmask_as_raw_16bits_unshifted;
  gpio_dma_pin_clear_mask = tmp_pinmask_as_raw_16bits_unshifted << 16;

  if(gpio_port_ptr == __null
  || tmp_pn == NC
  || (uint32_t)gpio_port_ptr < LOWEST_ADDRESS_OF_GPIO_RAM
  || (uint32_t)gpio_port_ptr > HIGHEST_ADDRESS_OF_GPIO_RAM)
  {
    tmp_ret_val = false;
  }
  else
    tmp_ret_val = true;

  return tmp_ret_val;
}


/** A helpful guide to the use of HSV with RGB LED's can be found here:
 * https://www.instructables.com/How-to-Make-Proper-Rainbow-and-Random-Colors-With-/
 * 
 * This is an implementation of what that author calls a sine wave rainbow. */
inline WS2812B_Led_Pixel_Colors_s WS2812B_RGB_LED_Strip::hue_value_to_rgb(float hue_degrees, uint8_t value)
{
  WS2812B_Led_Pixel_Colors_s return_pixel_colors;
  float value_f32 = (float)value;
  float hue_radians = hue_degrees / 360.0f * _2PI;
  
  if(hue_radians >= _2PI || hue_radians < 0.0f)
  {
    return_pixel_colors.blue = 0;
    return_pixel_colors.red = 0;
    return_pixel_colors.green = 0;
  }
  else if(hue_radians < _120_D2R) // < 120 degrees
  {
    return_pixel_colors.blue = 0;
    return_pixel_colors.green = (uint8_t)(value_f32 * _sin(hue_radians * 0.75f));
    return_pixel_colors.red = (uint8_t)(value_f32 * _cos(hue_radians * 0.75f));
  }
  else if(hue_radians < (2.0f * _120_D2R)) // <240 degrees
  {
    hue_radians -= _120_D2R;
    return_pixel_colors.red = 0;
    return_pixel_colors.blue = (uint8_t)(value_f32 * _sin(hue_radians * 0.75f));
    return_pixel_colors.green = (uint8_t)(value_f32 * _cos(hue_radians * 0.75f));
  }
  else // >=240 degrees
  {
    hue_radians -= 2.0f * _120_D2R;
    return_pixel_colors.green = 0;
    return_pixel_colors.red = (uint8_t)(value_f32 * _sin(hue_radians * 0.75f));
    return_pixel_colors.blue = (uint8_t)(value_f32 * _cos(hue_radians * 0.75f));
  }

  return return_pixel_colors;
}




