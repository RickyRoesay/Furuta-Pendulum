#include "startup.h"


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

#define TICK_PRIORITY_HAL_SYSTICK 0 // max priority is lowest number, 0

HAL_StatusTypeDef startup_run(void)
{
    HAL_StatusTypeDef tmp_status_ret_val = HAL_OK;

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

    /** We are using the default, weak version of the HAL's 
     * SysTick implementation (found in stm32g4xx_hal.c) that triggers 
     * an interrupt via SysTick. This prevents
     * an infinite while loop from occurring if the HSE or PLL fail to stabilize
     * (applicable when using the vendor supplied HAL_RCC_OscConfig function).
     * 
     * NOTE: HAL internal variable SystemCoreClock is initialized to the HSI_VALUE (16MHz) at startup,
     * and is automatically reconfigured when calling function HAL_RCC_ClockConfig.
     * 
     * More info on the ARM SysTick can be found here: 
     * https://developer.arm.com/documentation/101407/0542/Debugging/Debug-Windows-and-Dialogs/Core-Peripherals/Armv7-M-cores/Armv7-M--System-Tick-Timer */
    tmp_status_ret_val |= HAL_InitTick(TICK_PRIORITY_HAL_SYSTICK);

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
    tmp_status_ret_val |= HAL_RCC_OscConfig(&RCC_OscInitStruct);


    /** Initializes the CPU, AHB and APB buses clocks. */
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    /** TRM section 4.3.3 (or 3.3.3 or 5.3.3), for max frequency we need 4 wait states, 
     * or 5 cpu cycles for flash access latency. 
     * 
     * NOTE: This function automatically adjusts SystemCoreClock and calls 
     * HAL_InitTick again to account for changes in SYSCLK frequency. */
    tmp_status_ret_val |= HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

    return tmp_status_ret_val;
}



/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

void SysTick_Handler(void)
{
  HAL_IncTick();
}

