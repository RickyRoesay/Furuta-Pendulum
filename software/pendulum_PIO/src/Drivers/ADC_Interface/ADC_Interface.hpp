#ifndef __ADC_INTERFACE_HPP
#define __ADC_INTERFACE_HPP

#include "Arduino.h"
#include <SPI.h>
#include "gpio.h"

#include "stm32f4xx_ll_gpio.h"
#include "PortNames.h"
#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"

extern ADC_HandleTypeDef adc1_handle__SOx__Vbus;

extern ADC_HandleTypeDef adc2_handle__M_AUX_temps;

void adc_if_init(HardwareTimer* inj_ch_trig_timer);

inline uint32_t adc_if_get_phB_LS_CS_SO1_counts(void);
inline uint32_t adc_if_get_phC_LS_CS_SO2_counts(void);


inline uint32_t adc_if_get_vbus_counts(void);
float adc_if_get_vbus_v(void);



 

#endif // __ADC_INTERFACE_HPP
