
#include "ADC_Interface.hpp"
#include "Arduino.h"
#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include "../lib/arduino-foc/src/current_sense/hardware_specific/stm32/stm32f4/stm32f4_utils.h"
#include "../lib/arduino-foc/src/current_sense/hardware_specific/stm32/stm32f4/stm32f4_hal.h"
#include "NTC_LUT.h"


/** this is a redundant class that has the same peripheral instance as the
 * ADC class used by SimpleFOC SW components (can be found in "stm32f4_hal.cpp").
 * 
 * We use this here to get access to the raw counts for current sensor measurements
 * and to access the counts for the Vbus measurement that was hacked onto injected 
 * measurement 3 within the "stm32f4_hal.cpp" file. */
ADC_HandleTypeDef adc1_handle__SOx__Vbus;

ADC_HandleTypeDef adc2_handle__M_AUX_temps;

////////////////////////// STATIC: FUNCTION: DECLARATIONS: //////////////////////////
static int adc_if_adc2_init(HardwareTimer* timer);





////////////////////////// PUBLIC: FUNCTION: DEFINITIONS: //////////////////////////


void adc_if_init(HardwareTimer* inj_ch_trig_timer)
{
    adc1_handle__SOx__Vbus.Instance = (ADC_TypeDef *)pinmap_peripheral(analogInputToPinName(SO1), PinMap_ADC);
    //adc2_handle__M_AUX_temps.Instance = (ADC_TypeDef *)ADC2_BASE;
    (void)adc_if_adc2_init(inj_ch_trig_timer);
}



uint32_t adc_if_get_phB_LS_CS_SO1_counts(void)
{
    return HAL_ADCEx_InjectedGetValue(&adc1_handle__SOx__Vbus, ADC_INJECTED_RANK_1);
}

uint32_t adc_if_get_phC_LS_CS_SO2_counts(void)
{
    return  HAL_ADCEx_InjectedGetValue(&adc1_handle__SOx__Vbus, ADC_INJECTED_RANK_2);
}



uint32_t adc_if_get_vbus_counts(void)
{
    return HAL_ADCEx_InjectedGetValue(&adc1_handle__SOx__Vbus, ADC_INJECTED_RANK_3);
}

float adc_if_get_vbus_v(void)
{
    return (adc_if_get_vbus_counts() * VBUS_CNTS_TO_VOLTAGE_GAIN);
}



uint32_t adc_if_get_pcb_temp_near_motor_fets_cnts(void)
{
    return HAL_ADCEx_InjectedGetValue(&adc2_handle__M_AUX_temps, ADC_INJECTED_RANK_1);
}

float adc_if_get_pcb_temp_near_motor_fets_C(void)
{
    return get_NXP15XH103_temp_via_LUT(adc_if_get_pcb_temp_near_motor_fets_cnts());
}



uint32_t adc_if_get_pcb_temp_near_aux_fets_cnts(void)
{
    return HAL_ADCEx_InjectedGetValue(&adc2_handle__M_AUX_temps, ADC_INJECTED_RANK_2);
}

float adc_if_get_pcb_temp_near_aux_fets_C(void)
{
    return get_NXP15XH103_temp_via_LUT(adc_if_get_pcb_temp_near_aux_fets_cnts());
}




////////////////////////// STATIC: FUNCTION: DECLARATIONS: //////////////////////////

static int adc_if_adc2_init(HardwareTimer* timer)
{
    ADC_InjectionConfTypeDef sConfigInjected;

    pinMode(M_TEMP,   INPUT);
    pinMode(AUX_TEMP, INPUT);
    
    pinmap_pinout(analogInputToPinName(M_TEMP), PinMap_ADC);
    pinmap_pinout(analogInputToPinName(AUX_TEMP), PinMap_ADC);
    
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
    adc2_handle__M_AUX_temps.Instance = (ADC_TypeDef *)ADC2_BASE;
    
    __HAL_RCC_ADC2_CLK_ENABLE();

    adc2_handle__M_AUX_temps.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    adc2_handle__M_AUX_temps.Init.Resolution = ADC_RESOLUTION_12B;
    adc2_handle__M_AUX_temps.Init.ScanConvMode = ENABLE;
    adc2_handle__M_AUX_temps.Init.ContinuousConvMode = ENABLE;
    adc2_handle__M_AUX_temps.Init.DiscontinuousConvMode = DISABLE;
    adc2_handle__M_AUX_temps.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc2_handle__M_AUX_temps.Init.ExternalTrigConv = ADC_SOFTWARE_START; // for now
    adc2_handle__M_AUX_temps.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc2_handle__M_AUX_temps.Init.NbrOfConversion = 1;
    adc2_handle__M_AUX_temps.Init.DMAContinuousRequests = DISABLE;
    adc2_handle__M_AUX_temps.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if ( HAL_ADC_Init(&adc2_handle__M_AUX_temps) != HAL_OK)
    {
        return -1;
    }
        
    /** Configures for the selected ADC injected channel 
     * its corresponding rank in the sequencer and its sample time */
    sConfigInjected.InjectedNbrOfConversion = 2;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;  
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;

    #if 1
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    #else
    // automating TRGO flag finding - taken from SimpleFOC
    uint32_t trigger_flag = _timerToInjectedTRGO(timer);

    if(trigger_flag == _TRGO_NOT_AVAILABLE) 
        return -1;
    else
        sConfigInjected.ExternalTrigInjecConv = trigger_flag;
    #endif 

    // first channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(M_TEMP));
    if (HAL_ADCEx_InjectedConfigChannel(&adc2_handle__M_AUX_temps, &sConfigInjected) != HAL_OK){ return -1; }
    
    
    // second channel
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(AUX_TEMP));
    if (HAL_ADCEx_InjectedConfigChannel(&adc2_handle__M_AUX_temps, &sConfigInjected) != HAL_OK){ return -1; }

    adc2_handle__M_AUX_temps.Instance->CR2 |= 0x1;

    return 0;
}