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
#include "NTC_LUT.h"


/** This is a ODESC 4.2 PCB-specific interface for getting 
 * access to any signal measured via the STM32's ADC's.
 * These signals include: 
 * - SO1 & SO2 Phase Current Shunt Amplifier measurement via DRV8301. 
 * - "M_TEMP" and "AUX_TEMP" NTC's, part numbers are believed to be:
 * NXP15XH103F03RC based on the ODrive Bill of Materials. */




/** NOTE: Unlike the ODrive, the ODESC 4p2 does not use 
 * an dedicated external ADC reference IC.  It seems the designers 
 * of this PCB have connected the STM32's ADC ref input directly to 
 * the LDO's 3.3V output.  They may have (hopefully) used bead resistors
 * but I am not sure of this. */
#define ADC_VREF_F32 3.3f

/** These are the resistor divider values from the ODrive Schematics: */
#define VBUS_SENSE_RDIV_TOP_R_OHMS_F32  10000.0f
//#define VBUS_SENSE_RDIV_BOTTOM_R_OHMS_F32  1000.0f
#define VBUS_SENSE_RDIV_BOTTOM_R_OHMS_F32  560.0f 

#define ADC_RESOLUTION_IN_BITS 12
#define ADC_MAX_COUNTS_U (1 << ADC_RESOLUTION_IN_BITS)
#define ADC_MAX_COUNTS_F32 ((float)ADC_MAX_COUNTS_U)


#ifndef USE_CUSTOM_VBUS_GAIN_CALIBRATION
#define VBUS_CNTS_TO_VOLTAGE_GAIN   ((VBUS_SENSE_RDIV_TOP_R_OHMS_F32 \
                                        + VBUS_SENSE_RDIV_BOTTOM_R_OHMS_F32) \
                                    / VBUS_SENSE_RDIV_BOTTOM_R_OHMS_F32 \
                                    / ADC_MAX_COUNTS_F32 \
                                    * ADC_VREF_F32)
#else
#define VBUS_CNTS_TO_VOLTAGE_GAIN   (20.12f / 1333.0f)

#endif

extern ADC_HandleTypeDef adc1_handle__SOx__Vbus;

extern ADC_HandleTypeDef adc2_handle__M_AUX_temps;




void adc_if_init(HardwareTimer* inj_ch_trig_timer);



uint32_t adc_if_get_phB_LS_CS_SO1_counts(void);
uint32_t adc_if_get_phC_LS_CS_SO2_counts(void);


uint32_t adc_if_get_vbus_counts(void);
float adc_if_get_vbus_v(void);

uint32_t adc_if_get_pcb_temp_near_motor_fets_cnts(void);
float adc_if_get_pcb_temp_near_motor_fets_C(void);

uint32_t adc_if_get_pcb_temp_near_aux_fets_cnts(void);
float adc_if_get_pcb_temp_near_aux_fets_C(void);


 

#endif // __ADC_INTERFACE_HPP
