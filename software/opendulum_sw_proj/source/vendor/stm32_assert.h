/**
  ******************************************************************************
  * @file    stm32_assert.h
  * @author  MCD Application Team
  * @brief   STM32 assert template file.
  *          This file should be copied to the application folder and renamed
  *          to stm32_assert.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under the BSD-3 Clause, and the license file
  * can be found here: https://github.com/STMicroelectronics/STM32CubeG4/blob/master/LICENSE.md
  * 
  * @attention This file has been adapted from "stm32_assert_template.h" 
  * located in the STM32G4 HAL Github repo, and can be found 
  * at this link: https://github.com/STMicroelectronics/stm32g4xx_hal_driver.git
  * 
  * It has been altered to fit the needs of the "Opendulum" software project.
  * 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32_ASSERT_H
#define STM32_ASSERT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/


/** Using the custom "full assert" function unforturnately nuked everything 
 * and some of the HAL functions have no return type, either. */
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it returns HAL_ERROR
  * @retval None
  */
//#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))


/** In both CMSIS and HAL error returns types, and error has the value of 1. */
#define assert_param(expr) {if(expr) {(void)0U;} else {return 1}}
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* STM32_ASSERT_H */


