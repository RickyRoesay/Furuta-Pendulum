
#include "WS2812B_RGB_LED_Strip.hpp"
#include "Arduino.h"
#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"


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



static inline void write_bit_data_to_bitstream(uint32_t *ptr_to_bitstream, uint8_t bit_level);




///////////////////////////////////////////////////////////////////////////////////////

WS2812B_RGB_LED_Strip::WS2812B_RGB_LED_Strip(int DO_gpio_)
{
  DO_gpio = DO_gpio_;
  timer_handle = HardwareTimer(TIM7);
}

///////////////////////////////////////////////////////////////////////////////////////

WS2812B_Status_e  WS2812B_RGB_LED_Strip::init_dma_and_timer_peripherals(uint8_t num_of_leds_to_cmd)
{
  //timer_handle.setup(TIM5);
  //timer_handle = HardwareTimer(TIM5);

  if(num_of_leds_to_cmd > WS2812B_MAX_NUM_OF_LEDS
  || set_bit_masks_and_dma_dest_pointer_from_arduino_pin_macro(DO_gpio) == false
  || timer_handle.getHandle()->Instance == nullptr) //timer class construction failed if = nullptr
  {
    status = WS2812B_INIT_FAIL;
  }
  else
  {
    pinMode(DO_gpio, OUTPUT);

    requested_led_num = num_of_leds_to_cmd;
    active_led_num = num_of_leds_to_cmd;

    /** By default, the timer is configured with prescaler and clock divider
     * as 1 during class construction.  We only need to set the overflow to 50 counts
     * to get the desired overflow frequency of around 300ns. */
    //timer_handle.setOverflow(TMR_OVF_VAL_TO_GET_300NS_PRD_WITH_NO_CLKDIV_OR_PSC, TICK_FORMAT);
    //timer_handle.resume();

    __HAL_RCC_DMA2_CLK_ENABLE();

    /** Trigger on TIM5 UP */
    dma_handle.Instance = DMA2_Stream1;
    dma_handle.Init.Channel = DMA_CHANNEL_7;

    dma_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    //dma_handle.Init.Direction = DMA_MEMORY_TO_MEMORY;
    
    dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_handle.Init.MemInc = DMA_MINC_ENABLE;
    
    /** PSIZE and MSIZE */
    dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

    dma_handle.Init.Mode = DMA_NORMAL;

    dma_handle.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    
    dma_handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    //dma_handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    /** FIFOThreshold is not used, will be overwritten by hw at
     * the start of a direct mode DMA transfer */

    dma_handle.Init.MemBurst = DMA_MBURST_SINGLE;
    dma_handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
    
    
    HAL_DMA_Init(&dma_handle);
    
    /** mem source address */
    dma_handle.Instance->M0AR = (uint32_t)(bitstream[bitstream_idx_for_dma]);
    dma_handle.Instance->M1AR = (uint32_t)(bitstream[bitstream_idx_for_dma]);
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




bool WS2812B_RGB_LED_Strip::write_bitfield_array_via_dma()
{
  if((dma_handle.Instance->CR & 0x00000001)
  || status == WS2812B_INIT_FAIL)
    return 0;

  uint8_t tmp_bitstream_swap_space = bitstream_idx_for_led_buf;
  bitstream_idx_for_led_buf = bitstream_idx_for_dma;
  bitstream_idx_for_dma = tmp_bitstream_swap_space;

  //HAL_DMA_Start(&dma_handle, 0x20001000, 0x20001E00, WS2812B_NUM_OF_GPIO_WRITES_TOTAL);

  dma_handle.Instance->NDTR = WS2812B_NUM_OF_GPIO_WRITES_TOTAL;
  dma_handle.Instance->M0AR = (uint32_t)bitstream[bitstream_idx_for_dma];
  dma_handle.Instance->PAR = 0x40020018;
  __HAL_DMA_ENABLE(&dma_handle);

  status = WS2812B_TRANSMITTING_DATA;

  return 1;
}




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
    status = WS2812B_WAITING_FOR_PIXEL_DATA;

  return status;
}




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




void WS2812B_RGB_LED_Strip::write_pixel_data_to_bitstream(uint32_t *ptr_to_bitstream, 
                                                          WS2812B_Led_Pixel_Info_s * ptr_to_pixel_info)
{
  const static uint8_t tmp_bitfield_mask = 0x01;
  uint8_t tmp_bit_level;
  uint32_t tmp_ptr_to_bitstream_offset = 0;

  uint8_t tmp_pixel_shift_reg = ptr_to_pixel_info->red;
  
  for(uint8_t tmp_bitshft_idx = 0; tmp_bitshft_idx < 8; tmp_bitshft_idx++)
  {
    tmp_bit_level = tmp_bitfield_mask & tmp_pixel_shift_reg;
    tmp_pixel_shift_reg = tmp_pixel_shift_reg >> 1;
    write_bit_data_to_bitstream(&ptr_to_bitstream[tmp_ptr_to_bitstream_offset], tmp_bit_level);
    tmp_ptr_to_bitstream_offset += 4;
  }

  tmp_pixel_shift_reg = ptr_to_pixel_info->green;
  
  for(uint8_t tmp_bitshft_idx = 0; tmp_bitshft_idx < 8; tmp_bitshft_idx++)
  {
    tmp_bit_level = tmp_bitfield_mask & tmp_pixel_shift_reg;
    tmp_pixel_shift_reg = tmp_pixel_shift_reg >> 1;
    write_bit_data_to_bitstream(&ptr_to_bitstream[tmp_ptr_to_bitstream_offset], tmp_bit_level);
    tmp_ptr_to_bitstream_offset += 4;
  }

  tmp_pixel_shift_reg = ptr_to_pixel_info->blue;
  
  for(uint8_t tmp_bitshft_idx = 0; tmp_bitshft_idx < 8; tmp_bitshft_idx++)
  {
    tmp_bit_level = tmp_bitfield_mask & tmp_pixel_shift_reg;
    tmp_pixel_shift_reg = tmp_pixel_shift_reg >> 1;
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





bool WS2812B_RGB_LED_Strip::update_num_of_leds_to_cmd(uint8_t num_of_leds_to_cmd)
{
  bool tmp_ret_val;

  if(num_of_leds_to_cmd <= WS2812B_MAX_NUM_OF_LEDS)
  {
    requested_led_num = num_of_leds_to_cmd; 
    tmp_ret_val = true;
  }
  else
    tmp_ret_val = false;

  return tmp_ret_val;
}





void WS2812B_RGB_LED_Strip::set_gpio_pin_level(bool pin_level)
{
  if(pin_level == HIGH)
    gpio_port_ptr->BSRR = gpio_dma_pin_set_mask;
  else
    gpio_port_ptr->BSRR = gpio_dma_pin_clear_mask;
}




