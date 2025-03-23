
#include "ADC_Interface.hpp"
#include "Arduino.h"
#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"



/** this is a redundant class that has the same peripheral instance as the
 * ADC class used by SimpleFOC SW components (can be found in "stm32f4_hal.cpp").
 * 
 * We use this here to get access to the raw counts for current sensor measurements
 * and to access the counts for the Vbus measurement that was hacked onto injected 
 * measurement 3 within the "stm32f4_hal.cpp" file. */
ADC_HandleTypeDef adc1_handle__SOx__Vbus;

ADC_HandleTypeDef adc2_handle__M_AUX_temps;


void adc_if_init(void)
{
    adc1_handle__SOx__Vbus.Instance = (ADC_TypeDef *)pinmap_peripheral(analogInputToPinName(SO1), PinMap_ADC);
    adc2_handle__M_AUX_temps.Instance = (ADC_TypeDef *)ADC2_BASE;
}

