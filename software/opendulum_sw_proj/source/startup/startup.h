/** The "Startup" software component is meant to handle the misc startup
 * functions called once at the start of the main function.  
 * 
 * This does NOT include things that happen pre-main.  This handles
 * prefetch and instruction/data cache, Sys clock configuration, 
 * hal 1ms tick config, and other small things like disabling 
 * the UCPD Dead battery functionality of the UCPD peripheral. 
 * 
 * NOTE: This is also where the hal tick ISR is defined. 
*/


#include "stdint.h"
#include "stm32g4xx_hal_def.h"

#ifndef STARTUP_H
#define STARTUP_H


HAL_StatusTypeDef startup_run(void);


#endif //STARTUP_H

