#include <Arduino.h>
#include "STM32_CAN.h"
#include "../lib/Arduino-FOC/src/SimpleFOC.h"


#include "DRV8301/drv8301.hpp" 

#include "AS5048A_MagSenseSPI/AS5048A_MagSenseSPI.hpp" // mag sensor

#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc

#include "CAN_TP/can_tp.hpp"

#include "Biquad.hpp"

#if !defined(_STM32_DEF_)
#define _STM32_DEF_
#endif 

#if !defined(SIMPLEFOC_STM32_DEBUG)
#define SIMPLEFOC_STM32_DEBUG
#endif 

#include "gpio.h"
#include "config.hpp"
#include "Gimbal/gimbal.hpp"

////////////////////////// DEFINITIONS: ////////////////////////////






//////////////////////////// CLASS: DECLARATIONS: ////////////////////////////





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



ADC_HandleTypeDef my_hadc;






void on_motor(char* cmd){ command.motor(&motor, cmd); }
void on_constants(char* cmd);
























////////////////////////////////// PDM: /////////////////////////////////

/** old filter values:  
 0.20657128726265578,
 0.41314257452531156
 0.20657128726265578
-0.36952595241514796
 0.19581110146577102 */

#if 1 // proven, fast and loud
Biquad iq_sp_biquad = Biquad(0.20657128726265578, 
                             0.41314257452531156,
                             0.20657128726265578,
                            -0.36952595241514796,
                             0.19581110146577102);
#else // NEW:  THIS CAUSES OVER CURRENT FAULTS AND FALSE SETPOINTS!!!! FPU seems to over/underflow?
Biquad iq_sp_biquad = Biquad(0.136890606806f, 
                             0.273781213612f,
                             0.136890606806f,
                            -0.780780215219f,
                             0.328342642443f);
#endif 

typedef enum
{
  PDM_STATE_INIT,
  PDM_STATE_INIT_DONE,
  PDM_STATE_SWING_UP,
  PDM_STATE_UPRIGHT,
  PDM_STATE_SWING_DOWN,
  PDM_STATE_RESET,
  PDM_STATE_NUM_OF_STATES,
} pdm_state_e;

typedef struct
{
  uint32_t reset_loop_wait_cnt;
  uint16_t swing_down_to_swing_up_cntr;
  float P; // +/- P radians for phi, when going from swing-up to upright
  float PH;
  float H; // +/- H radians for phi, when going from upright to swing-down
  pdm_state_e state;
} pdm_control_s;


typedef struct
{
  uint32_t offset_reset_cntr;
  int32_t last_rotation_num_at_offset_check; 
  float phi_delta_thld_for_offset_restart;
  float min_phi_in_revolution;
  float max_phi_in_revolution;
  float median_phi_in_revolution;
  float theta_offset;
} pdm_offset_s;


typedef struct 
{
  float pdm_theta_dot_filt;
  float pdm_theta;
  float pdm_phi;
  float exp_alpha_val; 

  float K; // final swingup coeff
  float A; // energy proportional coeff for swingup
  float C;

  
  float J; // final swingdown coeff
  float B; // energy proportional coeff for swingdown

  float I; // final upright coeff
  float L; // K matrix coeff for motor theta dot
  float M; // K matrix coeff for pendulum phi (angle from upright)
  float N; // K matrix coeff for pendulum phi dot

  float D; // Print out pendulum info on serial if = 1

  float tau_prev;
  float tau_iir_alpha;
  float tau_ramp_rate_limit;
  float S; // 0 = no setpoint ramp/filtering, 1 = ramp, 2 = iir/exponential filter

  pdm_offset_s offset;
  pdm_control_s control;
} pdm_info_s;

pdm_info_s spinny = {
  .pdm_theta_dot_filt = 0.0f, 
  .pdm_theta = 0.0f, 
  .pdm_phi = 0.0f,   
  .exp_alpha_val = 0.2, // old value is 0.025

  /** Swing-Up Controller: */
  .K = 0.00001f, //0.0015f   // initial value 
  .A = 0.018f, //0.018f     // proportional to energy required to swing pendulum
  .J = 0.0008f, // main value
  
  /** Swing-Down Controller: */
  .B = 0.02,

  /** Upright Controller: */
  .I = -0.05f,//-0.05f    // overall
  .L = -0.7f, //, -0.3, -0.23f, -0.25f   // theta dot
  .M = 25.0f, //20.0f,    // phi
  .N = -1.5f, //-1.5f     // phi dot

  .D = 0.0f, // Print out pendulum info on serial if = 1

  .tau_prev = 0.0f, // used for setpoint iir
  .tau_iir_alpha = 0.01f,
  .tau_ramp_rate_limit = 0.002f,
  .S = 1.0f, // 0 = no setpoint ramp/filtering, 1 = ramp, 2 = iir/exponential filter

  .offset = {
    .offset_reset_cntr = 0,
    .last_rotation_num_at_offset_check = 0,
    .phi_delta_thld_for_offset_restart = 0.10f,//0.07f,
    .theta_offset = 0.0f,
  },

  .control = {
    .reset_loop_wait_cnt = 0,
    .P = 1.0f, // +/- 1 radian for phi as state machine 
    .PH = 0.8f,
    .H = 0.8f, // +/- 0.1 radian for phi as state machine 
    .state = PDM_STATE_INIT,
  }
};

HardwareTimer *ctrl_loop_5kHz_timer;























////////////////////////////////// PDM: FUNCTION: DEFINITIONS: /////////////////////////////////


float get_upright_setpoint(void)
{
  float tmp_q_curr_req = spinny.I * ((spinny.L * motor.shaft_velocity ) 
                        + (spinny.M * spinny.pdm_phi ) 
                        + (spinny.N * spinny.pdm_theta_dot_filt));

  return tmp_q_curr_req;
}

float get_swing_up_setpoint(void)
{
  float cos_th_pow4 = _cos(spinny.pdm_theta)*_cos(spinny.pdm_theta)*_cos(spinny.pdm_theta)*_cos(spinny.pdm_theta);
  float cos_th_pow2 = _cos(spinny.pdm_theta)*_cos(spinny.pdm_theta);
  float th_dot_pow2 = spinny.pdm_theta_dot_filt*spinny.pdm_theta_dot_filt;

  float tmp_q_curr_req = spinny.K * cos_th_pow4 * spinny.pdm_theta_dot_filt * ((9.81f*(1.0f - _cos(spinny.pdm_theta))) - (spinny.A * th_dot_pow2));
  return tmp_q_curr_req;
}

float get_swing_down_setpoint(void)
{
  float cos_th_pow4 = _cos(spinny.pdm_theta)*_cos(spinny.pdm_theta)*_cos(spinny.pdm_theta)*_cos(spinny.pdm_theta);
  float th_dot_pow2 = spinny.pdm_theta_dot_filt*spinny.pdm_theta_dot_filt;

  //float tmp_q_curr_req = spinny.J * cos_th_pow4 * spinny.pdm_theta_dot_filt * ((spinny.B * th_dot_pow2) - (9.81f*(1.0f - _cos(spinny.pdm_theta))));
  float tmp_q_curr_req = spinny.J * cos_th_pow4 * spinny.pdm_theta_dot_filt * ((9.81f*(1.0f - _cos(spinny.pdm_theta))) - (spinny.B * th_dot_pow2));
  return tmp_q_curr_req;
}



/** These functions requires the mag sensor to be initialized! */
void set_theta_offset_when_pointed_down(void)
{
  /** Set theta offset for pendulum angle.  Having "+ _PI" in the offset 
   * helps choose an initial value that will align with phi = 0 being the 
   * upright position. */
  spinny.offset.theta_offset = mag_sense.getMechanicalAngle() + _PI; // 180deg = down
  if(spinny.offset.theta_offset >= _2PI) 
    spinny.offset.theta_offset -= _2PI; 
}
void set_theta_offset_when_pointed_up(void)
{
  spinny.offset.theta_offset = mag_sense.getMechanicalAngle(); // 0deg = upright
  if(spinny.offset.theta_offset >= _2PI) 
    spinny.offset.theta_offset -= _2PI;
}
void set_theta_offset_when_free_spinning(float median_phi)
{
  spinny.offset.theta_offset -= median_phi;
  if(spinny.offset.theta_offset >= _2PI) 
    spinny.offset.theta_offset -= _2PI;
}


void Update_IT_callback(void)
{
  digitalWrite(GPIO2, HIGH); 
  
  mag_sense.update();
  /** update theta and phi values. */
  spinny.pdm_theta = spinny.offset.theta_offset - mag_sense.getMechanicalAngle();
  if(spinny.pdm_theta < 0.0f) spinny.pdm_theta+=_2PI;
  spinny.pdm_phi = spinny.pdm_theta;
  if(spinny.pdm_phi > _PI)
  spinny.pdm_phi -= _2PI; 
  
  spinny.pdm_theta_dot_filt = (mag_sense.getVelocity() * spinny.exp_alpha_val) \
  + ((1.0-spinny.exp_alpha_val) * spinny.pdm_theta_dot_filt);
  
  if(spinny.offset.min_phi_in_revolution > spinny.pdm_phi)
  spinny.offset.min_phi_in_revolution = spinny.pdm_phi;
  
  if(spinny.offset.max_phi_in_revolution < spinny.pdm_phi)
  spinny.offset.max_phi_in_revolution = spinny.pdm_phi;
  
  float tmp_q_curr_req;
  
  switch(spinny.control.state)
  {
    case PDM_STATE_INIT:
    case PDM_STATE_RESET:
      tmp_q_curr_req = get_upright_setpoint();
      if(spinny.pdm_theta < 2.14 || spinny.pdm_theta > 4.14)
      {
        if(spinny.control.state == PDM_STATE_RESET)
          motor.enable();

        spinny.control.state = PDM_STATE_SWING_UP;
        spinny.S = 2.0f; // use higher cutoff freq biquad once initialized
      }

      /** If the offset is good, the pendulum will swing up by itself due to the 
       * innaccuracies of the position sensor + being right on top of the "kick"
       * area of the swing up controller.  This recalibrates the offset if
       * the pendulum doesn't kick itself up after 2 seconds.  
       * 
       * It has been noticed that after the motor calibrates, even if the end position 
       * of the pendulum is equally pointed down as when the mcu powers on, the offset
       * is no longer valid.  This may be due to the fact that the mag sensor has automatic
       * gain calibration, and it may change gain after the motor turns on due to the 
       * H-fields.  this has not been confirmed. */
      spinny.offset.offset_reset_cntr++;
      if(spinny.offset.offset_reset_cntr > 2000)
      {
        set_theta_offset_when_pointed_down();
        spinny.offset.offset_reset_cntr = 0;
      }
    break;
    
    default:
    case PDM_STATE_SWING_UP:
    case PDM_STATE_UPRIGHT:
      if(spinny.pdm_theta < spinny.control.PH || spinny.pdm_theta > (_2PI - spinny.control.PH))
      {
        tmp_q_curr_req = get_upright_setpoint();
        spinny.control.PH = spinny.control.P;

        /** don't wait for the q current setpoint to ramp up with the filter */
        if(spinny.control.state != PDM_STATE_UPRIGHT)
          iq_sp_biquad.set_steady_state_val(tmp_q_curr_req);

        spinny.control.state = PDM_STATE_UPRIGHT;
        spinny.K = spinny.J;
      }
      else 
      {
        tmp_q_curr_req = get_swing_up_setpoint();
        spinny.control.PH = spinny.control.H;

        /** don't wait for the q current setpoint to ramp up with the filter */
        if(spinny.control.state != PDM_STATE_SWING_UP)
        #if 0
        {
          iq_sp_biquad.set_steady_state_val(tmp_q_curr_req);
          spinny.control.state = PDM_STATE_SWING_UP;
        }
        #else 
        {
          spinny.control.state = PDM_STATE_RESET;
          motor.disable();
        }
        #endif 

      }
    break;
  }

  #if 0
  if(spinny.S == 2.0f)
    tmp_q_curr_req = (tmp_q_curr_req * spinny.tau_iir_alpha) + (spinny.tau_prev * (1.0 - spinny.tau_iir_alpha));
  #else
  if(spinny.S == 2.0f)
    tmp_q_curr_req = iq_sp_biquad.update_and_return_filt_value(tmp_q_curr_req);
  #endif 
  else if(spinny.S == 1.0f)
    tmp_q_curr_req = _constrain(tmp_q_curr_req, spinny.tau_prev - spinny.tau_ramp_rate_limit, spinny.tau_prev + spinny.tau_ramp_rate_limit);

  spinny.tau_prev = tmp_q_curr_req;
    
  /** pack and send CAN msg */
  if(0)
  {
    send_can_Debug1_message(motor.current_sp,
                            motor.current.q,
                            spinny.pdm_theta_dot_filt,
                            motor.shaft_velocity,
                            spinny.pdm_phi,
                            encoder.getMechanicalAngle(),
                            0);
  }

  motor.move(tmp_q_curr_req);
  motor.loopFOC();

  digitalWrite(GPIO2, LOW); 
}















////////////////////////////////// COMM: FUNCTIONS: THAT: NEED: SPINNY: /////////////////////////

void on_constants(char* cmd)
{ 
  char cmd_idx1 = cmd[0];// parse command letter
  char cmd_idx2 = cmd[1];
  // check if there is a subcommand or not
  int value_index = (cmd_idx2 >= 'A'  && cmd_idx2 <= 'Z') ||  (cmd_idx2 == '#') ?  2 :  1;
  // parse command values
  float value = atof(&cmd[value_index]);

  switch(cmd_idx1){
    case 'E':      
      spinny.exp_alpha_val = value;
      hw_serial.print(F("exp filter alpha val set to: "));
      hw_serial.println(value);
      break;
    case 'K':      
      spinny.K = value;
      hw_serial.print(F("K coeff set to: "));
      hw_serial.println(value);
      break;
    case 'A':      
      spinny.A = value;
      hw_serial.print(F("A coeff set to: "));
      hw_serial.println(value);
      break;
    case 'J':      
      spinny.J = value;
      hw_serial.print(F("J coeff set to: "));
      hw_serial.println(value);
      break;
    case 'B':      
      spinny.B = value;
      hw_serial.print(F("B coeff set to: "));
      hw_serial.println(value);
      break;
    case 'I':      
      spinny.I = value;
      hw_serial.print(F("I coeff set to: "));
      hw_serial.println(value);
      break;
    case 'L':      
      spinny.L = value;
      hw_serial.print(F("L coeff set to: "));
      hw_serial.println(value);
      break;
    case 'M':      
      spinny.M = value;
      hw_serial.print(F("M coeff set to: "));
      hw_serial.println(value);
      break;
    case 'N':      
      spinny.N = value;
      hw_serial.print(F("N coeff set to: "));
      hw_serial.println(value);
      break;
    case 'P':      
      spinny.control.P = value;
      hw_serial.print(F("P coeff set to: "));
      hw_serial.println(value);
      break;
    case 'H':      
      spinny.control.H = value;
      hw_serial.print(F("H coeff set to: "));
      hw_serial.println(value);
      break;
    case 'T':      
      spinny.tau_iir_alpha = value;
      hw_serial.print(F("Tau exp filt alpha coeff set to: "));
      hw_serial.println(value);
      break;
    case 'R':      
      spinny.tau_ramp_rate_limit = value;
      hw_serial.print(F("Tau exp filt alpha coeff set to: "));
      hw_serial.println(value);
      break;
    case 'S':      
      spinny.S = value;
      hw_serial.print(F("Tau ramp/filt setting set to: "));
      hw_serial.println(value);
      break;
    case 'D':      
      spinny.D = value;
      hw_serial.print(F("Debug info flag set to: "));
      hw_serial.println(value);
      break;
    default:
      // do nothing
      break;
  }
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
  


  if(drv_ic.init(DRV8301_GAIN_SETTING, &SPI_2) != true)
  {
    hw_serial.println("Failed to set gain settings for DRV8301!");

    /** return early, do not control the motor! */
    return;
  }

  mag_sense.init(&SPI_2);  // init magnetic angle sensor
  mag_sense.update();
  
  set_theta_offset_when_pointed_down();
  
  can_tp_init();

  gimbal_init(hw_serial);

  iq_sp_biquad.set_steady_state_val(0.0f);

  command.add('M', on_motor,"my motor motion");
  command.add('C', on_constants,"Control Constants");
  
  my_hadc.Instance = (ADC_TypeDef *)pinmap_peripheral(analogInputToPinName(SO1), PinMap_ADC);

  command.run();
  motor.monitor();
  
  ctrl_loop_5kHz_timer->resume();

  motor.enable();
}















void loop() 
{
  //digitalWrite(GPIO2, HIGH); 

  #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
    command.run();
    motor.monitor();
  #endif 

  if(spinny.D == 1.0f)
  {
    hw_serial.print(spinny.pdm_phi, 6); //
    hw_serial.print("    ");
    hw_serial.print(spinny.pdm_theta_dot_filt, 6);
    hw_serial.print("    ");
    hw_serial.print(encoder.getVelocity(), 6);
    hw_serial.print("    ");
    hw_serial.print(encoder.getMechanicalAngle(), 6);
    hw_serial.print("    ");
    hw_serial.println(spinny.offset.median_phi_in_revolution, 6);
  }
  else if(spinny.D == 2.0f)
  {
    uint32_t test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_1);
    hw_serial.print(test); //
    hw_serial.print("    ");
    test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_2);
    hw_serial.print(test);
    #if 0
    hw_serial.print("    ");
    test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_2);
    hw_serial.print(test);
    hw_serial.print("    ");
    test = HAL_ADCEx_InjectedGetValue(&my_hadc, ADC_INJECTED_RANK_2);
    hw_serial.println(test);
    #endif 
  }

  
  int32_t num_of_full_rotations = encoder.getFullRotations();
  int32_t full_rotation_delta = num_of_full_rotations - spinny.offset.last_rotation_num_at_offset_check;
  
  if(full_rotation_delta == 2 || full_rotation_delta == -2)
  {
    spinny.offset.last_rotation_num_at_offset_check = num_of_full_rotations;
    // if phi min - phi max > threshold, reset value
    float phi_delta = spinny.offset.max_phi_in_revolution - spinny.offset.min_phi_in_revolution;
    if(phi_delta <= spinny.offset.phi_delta_thld_for_offset_restart)
    {
      if(spinny.control.state == PDM_STATE_UPRIGHT)
      {
        spinny.offset.median_phi_in_revolution = phi_delta / 2.0f;
        spinny.offset.median_phi_in_revolution += spinny.offset.min_phi_in_revolution;
        set_theta_offset_when_free_spinning(spinny.offset.median_phi_in_revolution);
      }
      else
      {
        spinny.control.state = PDM_STATE_RESET;
        motor.disable();
      }
    }
    // regardless, clear phi min and max
    spinny.offset.min_phi_in_revolution = _2PI;
    spinny.offset.max_phi_in_revolution = -_2PI;
  }
  else if(full_rotation_delta > 2 || full_rotation_delta < -2)
  {
    spinny.offset.last_rotation_num_at_offset_check = num_of_full_rotations;
    spinny.offset.min_phi_in_revolution = _2PI;
    spinny.offset.max_phi_in_revolution = -_2PI;
  }

  
  //digitalWrite(GPIO2, LOW);
}


