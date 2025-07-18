
#include <stdint.h>
#include <math.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "system_stm32g4xx.h"

#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_ll_bus.h"


#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_hal_rcc.h"

#include "pins.hpp"
#include "WS2812B_RGB_LED_Strip.hpp"
#include "OS_Tick.hpp"


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


WS2812B_RGB_LED_Strip rbg_led_strip_drv = WS2812B_RGB_LED_Strip(&gpio_led_data_out);

OS_Tick os_tick = OS_Tick(TIM2);

__IO uint8_t startup_status = 0;



//static void MX_GPIO_Init(void);

void     LED_Blinking(uint32_t Period);





int main(void)
{
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
  LL_PWR_DisableUCPDDeadBattery();







  
  /** We are using the default, weak version of the HAL's SysTick
   * implementation that triggers an interrupt via SysTick. This prevents
   * an infinite while loop from occurring if the HSE or PLL fail to stabilize
   * (applicable when using the vendor supplied HAL_RCC_OscConfig function).
   * 
   * NOTE: HAL internal variable SystemCoreClock is initialized to the HSI_VALUE (16MHz) at startup,
   * and is automatically reconfigured when calling function HAL_RCC_ClockConfig.
   * 
   * More info on the ARM SysTick can be found here: 
   * https://developer.arm.com/documentation/101407/0542/Debugging/Debug-Windows-and-Dialogs/Core-Peripherals/Armv7-M-cores/Armv7-M--System-Tick-Timer */
  if (HAL_InitTick(0) != HAL_OK)
  {
    startup_status |= HAL_ERROR;
  }
  
  
  /** Configure the main internal regulator output voltage, this is required for 
   * running the PLL at speeds greater than 150MHz. */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  

  
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  /** Since there is no crystal resonator or high precision clock source
   * for the moteus c1 (the HW i'm testing on), so just initialize the LSI 
   * for the IWDG. I don't want to overwrite the trim values of the 
   * HSI so we will not select that as a clock source to configure during 
   * the HAL based "HAL_RCC_OscConfig" function. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;// | RCC_OSCILLATORTYPE_HSE;
  //RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;

  /** As per TRM Section 7.4.4, 
   *   - f(VCO_clock) = f(PLL_clock_input) * (PLLN / PLLM)
   *   - f(PLL_x) = f(VCO_clock) / PLLx    (where x = P, Q and R)
   * 
   * As per STM32G474xx datasheet (DS12288 Rev 6) section 5.3.9, 
   * the maximum PLL VCO frequency is 344MHz while in voltage scaling 
   * range 1. 
   * 
   * The intended SYSCLK frequency is 170MHz, which is the maximum
   * that this device can support.  HSI freq = 16MHz, 
   * N = 85, M = 4, VCO = 340MHz, P, Q and R CLK's = 170MHz */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; //RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    startup_status |= HAL_ERROR;
  }


  /** Initializes the CPU, AHB and APB buses clocks. */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  /** TRM section 4.3.3 (or 3.3.3 or 5.3.3), for max frequency we need 4 wait states, or 5 cpu 
   * cycles for flash access latency. 
   * 
   * NOTE: This function automatically adjusts SystemCoreClock and calls 
   * HAL_InitTick again to account for changes in SYSCLK frequency. */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    startup_status |= HAL_ERROR;
  }

  pins_configure_gpio();

  rbg_led_strip_drv.set_gpio_pin_level(1);
  rbg_led_strip_drv.set_gpio_pin_level(0);

  rbg_led_strip_drv.init_dma_and_timer_peripherals(20);

  rbg_led_strip_drv.modify_pixel_buffer_all_leds(66.0f, 20);
  rbg_led_strip_drv.process_bitfield_array(20);
  rbg_led_strip_drv.write_bitfield_array_via_dma();

  (void)os_tick.configure_os_tick_timer();

  while (1)
  {
    gpio_led1.set_pin_level_high();
    gpio_led2.set_pin_level_high();
    os_tick.wait_us(1000000);
    gpio_led1.set_pin_level_low();
    gpio_led2.set_pin_level_low();
    os_tick.wait_us(1000000);
  }
}







/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

extern "C" void SysTick_Handler(void)
{
  HAL_IncTick();
}
