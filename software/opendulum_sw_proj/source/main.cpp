
#include <stdint.h>
// #include <math.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "system_stm32g4xx.h"

// #include "stm32g4xx_hal_flash.h"
// #include "stm32g4xx_ll_bus.h"


// #include "stm32g4xx_ll_rcc.h"
// #include "stm32g4xx_ll_crs.h"
// #include "stm32g4xx_ll_system.h"
// #include "stm32g4xx_ll_exti.h"
// #include "stm32g4xx_ll_cortex.h"
// #include "stm32g4xx_ll_utils.h"
// #include "stm32g4xx_ll_pwr.h"
// #include "stm32g4xx_ll_dma.h"
// #include "stm32g4xx_ll_gpio.h"
// #include "stm32g4xx_hal_rcc.h"

#include "pins.hpp"
#include "WS2812B_RGB_LED_Strip.hpp"
#include "OS_Tick.hpp"

extern "C" {
  #include "startup.h"
}


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


WS2812B_RGB_LED_Strip rbg_led_strip_drv = WS2812B_RGB_LED_Strip(&gpio_led_data_out);

OS_Tick os_tick = OS_Tick(TIM2);

HAL_StatusTypeDef startup_status; // tracks startup status, unused for now

float test = 0.0f;

int main(void)
{
  startup_status = startup_run();

  (void)os_tick.configure_os_tick_timer();
  
  pins_configure_gpio();

  rbg_led_strip_drv.set_gpio_pin_level(1);
  rbg_led_strip_drv.set_gpio_pin_level(0);

  rbg_led_strip_drv.init_dma_and_timer_peripherals(20);

  rbg_led_strip_drv.modify_pixel_buffer_all_leds(66.0f, 20);
  rbg_led_strip_drv.process_bitfield_array(20);
  rbg_led_strip_drv.write_bitfield_array_via_dma();


  while (1)
  {
    test += 5.023401f;
    gpio_led1.set_pin_level_high();
    gpio_led2.set_pin_level_high();
    os_tick.wait_us(1000000);
    gpio_led1.set_pin_level_low();
    gpio_led2.set_pin_level_low();
    os_tick.wait_us(1000000);
  }
}



