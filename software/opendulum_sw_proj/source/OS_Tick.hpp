
#include "stdint.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "system_stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"

/** This class is meant to act as an interface to a timer
 * that counts down at the same speed as the main SYSCLK.
 * 
 * The intended uses are:
 *  -For getting clk counts in a simple scheduler
 *  -To wait for a specific time using polling (just burning clock cycles)
 */

#pragma once


/** the idea behind using a static frequency for timing calculations 
 * is to intentionally have the OS_Tick class use the wrong speed
 * for calculating task periodicities.  We would do this intentionally
 * so that it is very obvious if either the external crystal resonator
 * circuit or the PLL never stabilized.  */
#define OS_TICK__USE_STATIC_FREQUENCY_FOR_SYSCLK_CONVERSIONS

#ifdef OS_TICK__USE_STATIC_FREQUENCY_FOR_SYSCLK_CONVERSIONS
#define OS_TICK_SYSCLK_FREQ   170000000
#else
#define OS_TICK_SYSCLK_FREQ   SystemCoreClock
#endif 



#define IS_OS_TICK_TIM_PERIPH_TIM2(INSTANCE)        ((INSTANCE) == TIM2)

#define IS_OS_TICK_TIM_PERIPH_VALID(INSTANCE)       (IS_OS_TICK_TIM_PERIPH_TIM2((INSTANCE))   || \
                                                    ((INSTANCE) == TIM5))
//


typedef enum {
  OS_TICK_POLLING_TIMER_INIT,
  OS_TICK_POLLING_TIMER_EXPIRED,
  OS_TICK_POLLING_TIMER_RUNNING,  
} OS_Tick_Polling_Timer_Status_e;


class OS_Tick 
{
  public:

    /** On the STM32G4 only TIM2 and TIM5 have 32 bit counters. 
     * 
     * Using a 32 bit timer simplifies implementation
     * as 16 bit timers may overflow multiple times depending on the
     * use case. */
    OS_Tick(TIM_TypeDef * tim_periph_instance_to_use_as_tick)
    {
      timer_hal_handle.Instance = tim_periph_instance_to_use_as_tick;
    }

    bool configure_os_tick_timer(void)
    {
      if(IS_OS_TICK_TIM_PERIPH_VALID(timer_hal_handle.Instance))
      {
        HAL_StatusTypeDef tmp_hal_err_ret_val;

        if(IS_OS_TICK_TIM_PERIPH_TIM2(timer_hal_handle.Instance))
          LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
        else
          LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

        timer_hal_handle.Init.Period = 0xFFFFFFFF;
        timer_hal_handle.Init.Prescaler = 0;
        timer_hal_handle.Init.ClockDivision = 0;
        timer_hal_handle.Init.CounterMode = TIM_COUNTERMODE_DOWN;
        timer_hal_handle.Init.RepetitionCounter = 0;

        tmp_hal_err_ret_val = HAL_TIM_Base_Init(&timer_hal_handle);

        tmp_hal_err_ret_val = (HAL_StatusTypeDef)(HAL_TIM_Base_Start(&timer_hal_handle) | tmp_hal_err_ret_val);

        if(tmp_hal_err_ret_val == HAL_OK)
        {
          polling_timer_status = OS_TICK_POLLING_TIMER_EXPIRED;
          was_initialization_successful = true;
        }
        else
        {          
          polling_timer_status = OS_TICK_POLLING_TIMER_INIT;
          was_initialization_successful = false;
        }
      }
      else
      {
        /** Set the timer handle instance to a valid value
         * so the "get_counter_val" function won't crash the mcu */
        timer_hal_handle.Instance = TIM2;
        
        polling_timer_status = OS_TICK_POLLING_TIMER_INIT;
        was_initialization_successful = false;
      }
      return was_initialization_successful;
    }



    inline uint32_t get_counter_val(void)
    {
      return timer_hal_handle.Instance->CNT;
    }



    inline void wait_us(uint32_t wait_time_in_us)
    {
      uint32_t tmp_ticks_to_wait_for = wait_time_in_us * (OS_TICK_SYSCLK_FREQ / 1000000);
      uint32_t tmp_start_tick = get_counter_val();

      while(tmp_ticks_to_wait_for > (tmp_start_tick - get_counter_val()) \
      && was_initialization_successful == true)
      {
        // do nothing, simply wait until the correct time has elapsed
      }
    }



    inline void start_polling_timer(uint32_t wait_time_in_us)
    {
      if(was_initialization_successful == false)
        return;

      polling_timer_start_tick = get_counter_val();
      polling_timer_status = OS_TICK_POLLING_TIMER_RUNNING;
      polling_timer_length_in_ticks = wait_time_in_us * (OS_TICK_SYSCLK_FREQ / 1000000);
    }



    inline OS_Tick_Polling_Timer_Status_e update_and_get_polling_timer_status(void)
    {
      switch(polling_timer_status)
      {
        default: 
        case OS_TICK_POLLING_TIMER_EXPIRED:
          // do nothing, keep the status the same
        break;

        case OS_TICK_POLLING_TIMER_RUNNING:
          if(polling_timer_length_in_ticks < (polling_timer_start_tick - get_counter_val()))
            polling_timer_status = OS_TICK_POLLING_TIMER_EXPIRED;
        break;
      }

      return polling_timer_status;
    }


  private:
    TIM_HandleTypeDef timer_hal_handle;
    OS_Tick_Polling_Timer_Status_e polling_timer_status; 
    uint32_t polling_timer_start_tick;
    uint32_t polling_timer_length_in_ticks;
    bool was_initialization_successful;

};






