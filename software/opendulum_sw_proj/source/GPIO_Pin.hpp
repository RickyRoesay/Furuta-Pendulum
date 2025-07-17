
#include "stdint.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "system_stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g474xx.h"


#pragma once

/** Arbitrarily chosen gpio base to act as a default when initialization
 * fails.  This is used as the placeholder in case an incorrectly configured
 * pointer value for "gpio_port_ptr" is dereferenced, preventing the MCU from crashing.*/
#define DEFAULT_GPIO_BASE         GPIOA

#define IS_GPIO_INIT_PARAM_STRUCT_IN_SRAM1(INSTANCE)        (((INSTANCE) >= SRAM1_BASE) && \
                                                            ((INSTANCE) <= (SRAM1_BASE + SRAM1_SIZE_MAX)))
//


// typedef enum {
//   GPIO_Pin__TYPE_OUTPUT,
//   GPIO_Pin__TYPE_INPUT,
//   GPIO_Pin__TYPE_ALTERNATE_FUNCTION,
//   GPIO_Pin__TYPE_ANALOG,
// } GPIO_Pin_Type_e;


/*
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
*/


#define IS_LL_GPIO_PIN(__VALUE__)          (((0x00000000U) < (__VALUE__)) && ((__VALUE__) <= (LL_GPIO_PIN_ALL)))

#define IS_LL_GPIO_MODE(__VALUE__)         (((__VALUE__) == LL_GPIO_MODE_INPUT)     ||\
                                            ((__VALUE__) == LL_GPIO_MODE_OUTPUT)    ||\
                                            ((__VALUE__) == LL_GPIO_MODE_ALTERNATE) ||\
                                            ((__VALUE__) == LL_GPIO_MODE_ANALOG))

#define IS_LL_GPIO_OUTPUT_TYPE(__VALUE__)  (((__VALUE__) == LL_GPIO_OUTPUT_PUSHPULL)  ||\
                                            ((__VALUE__) == LL_GPIO_OUTPUT_OPENDRAIN))

#define IS_LL_GPIO_SPEED(__VALUE__)        (((__VALUE__) == LL_GPIO_SPEED_FREQ_LOW)       ||\
                                            ((__VALUE__) == LL_GPIO_SPEED_FREQ_MEDIUM)    ||\
                                            ((__VALUE__) == LL_GPIO_SPEED_FREQ_HIGH)      ||\
                                            ((__VALUE__) == LL_GPIO_SPEED_FREQ_VERY_HIGH))

#define IS_LL_GPIO_PULL(__VALUE__)         (((__VALUE__) == LL_GPIO_PULL_NO)   ||\
                                            ((__VALUE__) == LL_GPIO_PULL_UP)   ||\
                                            ((__VALUE__) == LL_GPIO_PULL_DOWN))

#define IS_LL_GPIO_ALTERNATE(__VALUE__)    (((__VALUE__) == LL_GPIO_AF_0  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_1  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_2  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_3  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_4  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_5  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_6  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_7  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_8  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_9  )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_10 )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_11 )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_12 )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_13 )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_14 )   ||\
                                            ((__VALUE__) == LL_GPIO_AF_15 ))


class GPIO_Pin 
{
  public:

    GPIO_Pin(GPIO_TypeDef * gpio_periph_instance, LL_GPIO_InitTypeDef * gpio_init_struct)
    {
      if(IS_GPIO_ALL_INSTANCE(gpio_periph_instance) 
      && IS_GPIO_INIT_PARAM_STRUCT_IN_SRAM1((uint32_t)gpio_init_struct))
      {
        init_status = HAL_OK;

        gpio_port_ptr = gpio_periph_instance;

        // VERIFY THAT THERE IS ONLY ONE PIN IN THE gpio_init_struct-pin BITMAP!!!!
        bool tmp_has_a_bit_in_the_bitfield_been_found = false;
        uint32_t tmp_bitmap_param;
        for(uint8_t i = 0; i < 16; i++)
        {
          tmp_bitmap_param = gpio_init_struct->Pin >> i;

          if(tmp_bitmap_param & 0x1UL)
          {
            if(tmp_has_a_bit_in_the_bitfield_been_found == false)
              tmp_has_a_bit_in_the_bitfield_been_found = true;
            else
              init_status = HAL_ERROR;
          }
          else
          {
            // do nothing, keep iterating through the for loop
          }
        }

        if(tmp_has_a_bit_in_the_bitfield_been_found != true)
        {
          init_status = HAL_ERROR;
        }
        else if(init_status == HAL_OK)
        {
          uint32_t tmp_bitmask = (0x0000FFFFUL & gpio_init_struct->Pin);
          gpio_pin_read_input_data_mask = tmp_bitmask;
          gpio_pin_set_bsrr_mask = tmp_bitmask;
          gpio_pin_clear_bsrr_mask = tmp_bitmask << 16;
        }
        else
        {
          /** Do nothing.  We will only ever reach this 
           * state if more than one pin was selected in the pin
           * "bitfield" parameter which is not a valid setting
           * for this class. */
        }

        if(init_status == HAL_OK)
        {
          switch((uint32_t)gpio_port_ptr)
          {
            case GPIOA_BASE:
              LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
            break;
            
            case GPIOB_BASE:
              LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
            break;
            
            case GPIOC_BASE:
              LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
            break;
            
            case GPIOD_BASE:
              LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
            break;
            
            case GPIOE_BASE:
              LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
            break;
            
            case GPIOF_BASE:
              LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
            break;
            
            case GPIOG_BASE:
              LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
            break;
            
            default:
              /** This should never happen!  */
              init_status = HAL_ERROR;
            break;
          }
        }
        else
        {
          /** do nothing, do not enable any of the clocks for
           * the GPIO pins if settings are wrong */
        }
        
        if(init_status == HAL_OK
        && IS_LL_GPIO_PIN(gpio_init_struct->Pin)
        && IS_LL_GPIO_MODE(gpio_init_struct->Mode)
        && IS_LL_GPIO_PULL(gpio_init_struct->Pull)
        && IS_LL_GPIO_SPEED(gpio_init_struct->Speed)
        && IS_LL_GPIO_OUTPUT_TYPE(gpio_init_struct->OutputType))
        {
          init_status = (HAL_StatusTypeDef)LL_GPIO_Init(gpio_port_ptr, gpio_init_struct);
          if(init_status == HAL_OK)
          {
            gpio_bsrr_reg_ptr = (uint32_t *)&(gpio_port_ptr->BSRR);
            gpio_idr_reg_ptr = (uint32_t *)&(gpio_port_ptr->IDR);
          }
          else
          {
            // set default pins
            set_bitmasks_and_pointers_to_default();
          }
        }
        else
        {
          // set default pins
          set_bitmasks_and_pointers_to_default();
          
          init_status = HAL_ERROR;
        }
      }
      else
      {
        /** Set structures to an unused pin configured as input/default. */
        
        // set default pins
        set_bitmasks_and_pointers_to_default();

        init_status = HAL_ERROR;
      }
    }

    inline HAL_StatusTypeDef get_init_status(void)
    {
      return init_status;
    }

    inline bool get_pin_level(void)
    {
      return (*gpio_idr_reg_ptr & gpio_pin_read_input_data_mask);
    }

    inline void set_pin_level_high(void)
    {
      *gpio_bsrr_reg_ptr = gpio_pin_set_bsrr_mask;
    }

    inline void set_pin_level_low(void)
    {
      *gpio_bsrr_reg_ptr = gpio_pin_clear_bsrr_mask;
    }

    inline void set_pin_level(bool pin_level)
    {
      switch(pin_level)
      {
        case 1:
          set_pin_level_high();
        break;

        default:
        case 0:
          set_pin_level_low();
        break;
      }
    }

    
    /** These functions are primarily used with 
     * the WS2812B LED driver that utilized DMA to write into
     * the GPIO pin's BSRR register every ~300ns. */
    inline uint32_t get_pin_set_bitfield(void)
    {
      return gpio_pin_set_bsrr_mask;
    }
    inline uint32_t get_pin_reset_bitfield(void)
    {
      return gpio_pin_clear_bsrr_mask;
    }
    inline uint32_t* get_bsrr_address(void)
    {
      return gpio_bsrr_reg_ptr;
    }

    

  private:
    HAL_StatusTypeDef init_status;
  
    GPIO_TypeDef *gpio_port_ptr;

    uint32_t * gpio_bsrr_reg_ptr;
    uint32_t * gpio_idr_reg_ptr;

    uint32_t gpio_pin_read_input_data_mask;
    uint32_t gpio_pin_set_bsrr_mask; 
    uint32_t gpio_pin_clear_bsrr_mask;

    inline void set_bitmasks_and_pointers_to_default(void)
    {
      
      /** Prevent the micro from crashing if a function that
       * dereferences these pointers is called: */
      gpio_port_ptr = GPIOA;
      gpio_bsrr_reg_ptr = (uint32_t *)&(gpio_port_ptr->BSRR);
      gpio_idr_reg_ptr = (uint32_t *)&(gpio_port_ptr->IDR);
      
      /** Prevent gpio writes from doing anything. */
      gpio_pin_read_input_data_mask = 0UL;
      gpio_pin_set_bsrr_mask = 0UL;
      gpio_pin_clear_bsrr_mask = 0UL;
    }
};





static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //LL_GPIO_SetOutputPin
}


