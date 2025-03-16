#include <Arduino.h>
#include "STM32_CAN.h"
#include "../lib/Arduino-FOC/src/SimpleFOC.h"


#include "Drivers/DRV8301_Gate_Driver/DRV8301_Gate_Driver.hpp" 
#include "Drivers/AS5048A_MagSenseSPI/AS5048A_MagSenseSPI.hpp" // mag sensor
#include "Drivers/WS2812B_RGB_LED_Strip/WS2812B_RGB_LED_Strip.hpp" // RGB LED's

#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc

#include "CAN_TP/can_tp.hpp"

#include "Biquad.hpp"


#include "gpio.h"
#include "config.hpp"
#include "Gimbal/gimbal.hpp"

#include "Pendulum/pendulum.hpp"





////////////////////////// DEFINITIONS: ////////////////////////////

#if !defined(_STM32_DEF_)
#define _STM32_DEF_
#endif 

#if !defined(SIMPLEFOC_STM32_DEBUG)
#define SIMPLEFOC_STM32_DEBUG
#endif 





//////////////////////////// CLASS: DECLARATIONS: ////////////////////////////

ADC_HandleTypeDef my_hadc;
HardwareTimer ctrl_loop_5kHz_timer = HardwareTimer(TIM3);
HardwareTimer test_tmr = HardwareTimer(TIM8);

/************* DRV8301: DRIVER: *************/
/** NOTE: both sw_bldc_driver and hw_bldc_driver classes have control over 
 * the EN_GATE, but the only time hw_bldc_driver uses it is on initialization. 
 * 
 * after the hw_bldc_driver's init function is called only the "sw_bldc_driver" class
 * has control of the EN_GATE pin. */
DRV8301_Gate_Driver hw_bldc_driver = DRV8301_Gate_Driver(SPI_nCS_DRV, EN_GATE, nFAULT);
void on_nFAULT() { hw_bldc_driver.fault_pin_asserted_isr_callback(); }


/************* MAG: SENSE: *************/
AS5048A_MagSenseSPI mag_sense = AS5048A_MagSenseSPI(SPI_nCS_IO6);

/** Params are: MOSI, MISO, SCLK, Chip select (optional). Chip select will be controlled 
 * by the calling function. */
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI_SCK); 


/************* COMM: *************/
HardwareSerial hw_serial(GPIO4_UART_RX, GPIO3_UART_TX);

Commander command = Commander(hw_serial);// commander interface
void on_motor(char* cmd){ command.motor(&motor, cmd); }


WS2812B_RGB_LED_Strip led_strip = WS2812B_RGB_LED_Strip(USB_DP__LED_DO);











////////////////////////////////// 5kHz: CONTROL: LOOP: ISR: /////////////////////////////////

void Update_IT_callback(void)
{
  digitalWrite(GPIO2, HIGH); 
  
  pdm_run_control_loop();
  
  /** pack and send CAN msg */
  send_can_Debug1_message(motor.current_sp,
                          motor.current.q,
                          pdm.pdm_theta_dot_filt,
                          motor.shaft_velocity,
                          pdm.pdm_phi,
                          encoder.getMechanicalAngle(),
                          0);
  
  digitalWrite(GPIO2, LOW); 
}







////////////////////////////////// SETUP: /////////////////////////////////

void setup() 
{
  hw_serial.begin(250000); 

  /** only used for timing analysis for now */
  pinMode(GPIO1, OUTPUT);
  pinMode(GPIO2, OUTPUT);
  digitalWrite(GPIO1, LOW);
  digitalWrite(GPIO2, LOW);
  
  test_tmr.setOverflow(50, TICK_FORMAT);
  test_tmr.getHandle()->Instance->DIER = 1 << 8; // set UDE update DMA req enable bit HIGH
  test_tmr.resume();
  

  led_strip.init_dma_and_timer_peripherals(5);

  /** datasheet is super amazing and gives 0 info, but through
   * experimentation the first param ends up being green,
   * second param is red, third is blue */
  led_strip.modify_pixel_buffer_all_leds(0, 255, 255);

  digitalWrite(GPIO2, HIGH);
  led_strip.process_bitfield_array(1);
  digitalWrite(GPIO2, LOW);

  if(led_strip.process_bitfield_array(10) == WS2812B_READY_TO_UPLOAD_BITSTREAM)
  {
    (void)led_strip.write_bitfield_array_via_dma();
  }
  

  

  /** Prescaler is automatically set when setting overflow with format != tick */
  ctrl_loop_5kHz_timer.setOverflow(200UL, MICROSEC_FORMAT);
  ctrl_loop_5kHz_timer.attachInterrupt(Update_IT_callback);




  pinMode(AUX_H, OUTPUT); 
  pinMode(AUX_L, OUTPUT);
  digitalWrite(AUX_H, LOW);
  digitalWrite(AUX_L, LOW);

  /** Set as inputs to be configured in the simpleFOC's hardware specific
   * "_adc_init" in stm32f4_hal.cpp file.  We will used dual adc mode, 
   * of type "injected Simultaneous" so we can read the raw values
   * at any time (either in the low priority tasks or in the JEOC ISR) */
  pinMode(M_TEMP,   INPUT);
  pinMode(AUX_TEMP, INPUT);
  pinMode(VBUS_SNS, INPUT);
  
  // init magnetic angle sensor
  if(mag_sense.init(&SPI_2) != true)
  {
    hw_serial.println("Failed to initialize AS5048A Mag Sensor!");

    /** return early, do not control the motor! */
    return;
  }
  
  mag_sense.update();
  
  pdm_set_theta_offset_when_pointed_down();
  
  if(hw_bldc_driver.init(DRV8301_GAIN_SETTING, &SPI_2, on_nFAULT) != true)
  {
    hw_serial.println("Failed to set gain settings for DRV8301!");

    /** return early, do not control the motor! */
    return;
  }

  can_tp_init();

  gimbal_init(hw_serial);

  pdm_torque_setpoint_biquad_c.set_steady_state_val(0.0f);

  command.add('M', on_motor,"my motor motion");
  command.add('C', pdm_on_constants,"Control Constants");
  
  my_hadc.Instance = (ADC_TypeDef *)pinmap_peripheral(analogInputToPinName(SO1), PinMap_ADC);

  command.run();
  motor.monitor();
  
  ctrl_loop_5kHz_timer.resume();

  

  motor.enable();
}










void loop() 
{
  //digitalWrite(GPIO2, HIGH); 

  command.run();
  motor.monitor();

  if(pdm.D == 1.0f)
  {
    hw_serial.print(pdm.pdm_phi, 6); //
    hw_serial.print("    ");
    hw_serial.print(pdm.pdm_theta_dot_filt, 6);
    hw_serial.print("    ");
    hw_serial.print(encoder.getVelocity(), 6);
    hw_serial.print("    ");
    hw_serial.print(encoder.getMechanicalAngle(), 6);
    hw_serial.print("    ");
    hw_serial.println(pdm.offset.median_phi_in_revolution, 6);
  }
  else if(pdm.D == 2.0f)
  {
    uint32_t test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_1);
    hw_serial.print(test); //
    hw_serial.print("    ");
    test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_2);
    hw_serial.print(test);
    hw_serial.print("    ");
    test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_3);
    hw_serial.println(test);
  }

  pdm_run_phi_offset_correction_checks();
  
  //digitalWrite(GPIO2, LOW);
}


