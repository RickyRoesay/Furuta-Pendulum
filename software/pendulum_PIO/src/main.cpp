#include <Arduino.h>
#include "STM32_CAN.h"
#include "../lib/Arduino-FOC/src/SimpleFOC.h"


#include "DRV8301/drv8301.hpp" 

#include "AS5048A_MagSenseSPI/AS5048A_MagSenseSPI.hpp" // mag sensor

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
HardwareTimer *ctrl_loop_5kHz_timer;

/************* DRV8301: DRIVER: *************/
/** NOTE: both sw_bldc_driver and drv_ic classes have control over 
 * the EN_GATE, but the only time drv_ic uses it is on initialization. 
 * 
 * after the drv_ic's init function is called only the "sw_bldc_driver" class
 * has control of the EN_GATE pin. */
Drv8301 drv_ic = Drv8301(SPI_nCS_DRV, EN_GATE, nFAULT);


/************* MAG: SENSE: *************/
AS5048A_MagSenseSPI mag_sense = AS5048A_MagSenseSPI(SPI_nCS_IO6);

/** Params are: MOSI, MISO, SCLK, Chip select (optional). Chip select will be controlled 
 * by the calling function. */
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI_SCK); 


/************* COMM: *************/
HardwareSerial hw_serial(GPIO4_UART_RX, GPIO3_UART_TX);

Commander command = Commander(hw_serial);// commander interface
void on_motor(char* cmd){ command.motor(&motor, cmd); }


/************* SETPOINT: BIQUAD: FILTER: *************/

#if 0 // proven, fast and loud
Biquad pdm_torque_setpoint_biquad_c = Biquad(0.20657128726265578, 
                             0.41314257452531156,
                             0.20657128726265578,
                            -0.36952595241514796,
                             0.19581110146577102);
#else // NEW:  THIS CAUSES OVER CURRENT FAULTS AND FALSE SETPOINTS!!!! FPU seems to over/underflow?

#endif 













////////////////////////////////// PDM: FUNCTION: DEFINITIONS: /////////////////////////////////

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
  hw_serial.begin(250000); //115200

  #if 0 /** this is set up and used by the hw_serial class (uart/serial) */
  pinMode(GPIO3_UART_TX, INPUT);
  pinMode(GPIO4_UART_RX, INPUT);
  #endif 

  /** only used for timing analysis for now */
  pinMode(GPIO1, OUTPUT);
  pinMode(GPIO2, OUTPUT);

  // Automatically retrieve TIM instance and channel associated to FLOATING_M1_CH_TIM3
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(FLOATING_M1_CH_TIM3), PinMap_PWM);
  ctrl_loop_5kHz_timer = new HardwareTimer(Instance);

  /** Prescaler is automatically set when setting overflow with format != tick */
  ctrl_loop_5kHz_timer->setOverflow(200UL, MICROSEC_FORMAT);
  ctrl_loop_5kHz_timer->attachInterrupt(Update_IT_callback);

  pinMode(SPI_nCS_DRV, OUTPUT);
  digitalWrite(SPI_nCS_DRV, HIGH);

  pinMode(SPI_nCS_IO6, OUTPUT);
  digitalWrite(SPI_nCS_IO6, HIGH);

  /** The Gate enable pin's pinmode will get overridden with the same 
   * value when the "sw_bldc_driver" has it's init function
   * called. */
  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, LOW);

  pinMode(nFAULT, INPUT);

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
    hw_serial.println("Failed to initialized AS5048A Mag Sensor!");

    /** return early, do not control the motor! */
    return;
  }
  
  mag_sense.update();
  pdm_set_theta_offset_when_pointed_down();
  
  
  if(drv_ic.init(DRV8301_GAIN_SETTING, &SPI_2) != true)
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
  
  ctrl_loop_5kHz_timer->resume();

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
    #if 0
    hw_serial.print("    ");
    test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_2);
    hw_serial.println(test);
    #endif 
  }

  
  int32_t num_of_full_rotations = encoder.getFullRotations();
  int32_t full_rotation_delta = num_of_full_rotations - pdm.offset.last_rotation_num_at_offset_check;
  
  if(full_rotation_delta == 2 || full_rotation_delta == -2)
  {
    pdm.offset.last_rotation_num_at_offset_check = num_of_full_rotations;
    // if phi min - phi max > threshold, reset value
    float phi_delta = pdm.offset.max_phi_in_revolution - pdm.offset.min_phi_in_revolution;
    if(phi_delta <= pdm.offset.phi_delta_thld_for_offset_restart)
    {
      if(pdm.control.state == PDM_STATE_UPRIGHT)
      {
        pdm.offset.median_phi_in_revolution = phi_delta / 2.0f;
        pdm.offset.median_phi_in_revolution += pdm.offset.min_phi_in_revolution;
        pdm_set_theta_offset_when_free_spinning(pdm.offset.median_phi_in_revolution);
      }
      else
      {
        pdm.control.state = PDM_STATE_RESET;
        motor.disable();
      }
    }
    // regardless, clear phi min and max
    pdm.offset.min_phi_in_revolution = _2PI;
    pdm.offset.max_phi_in_revolution = -_2PI;
  }
  else if(full_rotation_delta > 2 || full_rotation_delta < -2)
  {
    pdm.offset.last_rotation_num_at_offset_check = num_of_full_rotations;
    pdm.offset.min_phi_in_revolution = _2PI;
    pdm.offset.max_phi_in_revolution = -_2PI;
  }

  
  //digitalWrite(GPIO2, LOW);
}


