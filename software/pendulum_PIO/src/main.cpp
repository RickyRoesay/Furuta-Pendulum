#include <Arduino.h>
#include "STM32_CAN.h"
#include "../lib/Arduino-FOC/src/SimpleFOC.h"

#include "DRV8301/drv8301.hpp"
#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc

#include "Biquad.hpp"

#if !defined(_STM32_DEF_)
#define _STM32_DEF_
#endif 

#if !defined(SIMPLEFOC_STM32_DEBUG)
#define SIMPLEFOC_STM32_DEBUG
#endif 




////////////////////////// DEFINITIONS: ////////////////////////////

#define ENABLE_SERIAL_DEBUG_OUTPUT

#define	GPIO1	        PA0 // use this as uart4 tx
#define	GPIO2	        PA1 // use this as uart4 rx
#define	GPIO3_UART_TX	PA2
#define	GPIO4_UART_RX	PA3

#define	SPI_MOSI      PC12
#define	SPI_MISO	    PC11
#define	SPI_SCK	      PC10
#define	SPI_nCS_IO6	  PB2
#define	SPI_nCS_DRV	  PC13  // not confirmed

#define	CAN_TX	      PB9   // not confirmed
#define	CAN_RX	      PB8   // not confirmed

#define	ENC_A	        PB4
#define	ENC_B	        PB5
#define	ENC_Z	        PC9

#define	USB_DM	      PA11  // not confirmed
#define	USB_DP	      PA12  // not confirmed

#define	nFAULT	      PD2   // not confirmed
#define	EN_GATE	      PB12
#define	AH	          PA8
#define	AL	          PB13
#define	BH	          PA9
#define	BL	          PB14
#define	CH	          PA10
#define	CL	          PB15

#define	AUX_H	        PB11
#define	AUX_L	        PB10

/** Analog Inputs, not confirmed: */
#define	SO1	          PC0
#define	SO2	          PC1

/** the "_ALT1" is for ADC2, and additionally "_ALT2" is for ADC3 */
#define	M_TEMP	      PC5_ALT1
#define	AUX_TEMP	    PA5_ALT1
#define	VBUS_SNS	    PA6_ALT1

/** this is an unused pin that corresponds to TIM3 when used as a 
 * timer pin via the HardwareTimer class.  We only need this to 
 * act as an interrupt generation source so we chose a pin that
 * is not connected to anything on the ODESC4.2 
 * 
 * NOTE: It is assumed that all "Motor1" (second instance of motors)
 * in the odrive 3.5 schematic have pins that are not connected to anything
 * on the ODESC4.2 */
#define FLOATING_M1_CH_TIM3  PC8




/** Number of pole pairs on the the GB54-1 is ?? config is 12n14p */
#define NUM_OF_POLE_PAIRS 7

/** Encoder Pulse/Counts per Revolution = 2048 AM103, default switch config */
#define ENCODER_PPR 2048UL
#define ENCODER_CPR 4U * ENCODER_PPR


#define FAKE_AS5048A_SENSOR
#ifdef FAKE_AS5048A_SENSOR
/** Magnetic Sensor P/N is AS5048A.  Bit resolution is 14 
 * It has been observed that bit 14 always remains HIGH, or at least is always
 * read in SW to have that value.  For now, lets just set the angle start bit and resolution 
 * to be 13 bits */
#define MAG_SNS_AS5048A_BIT_RES 13
#define MAG_SNS_ANGLE_START_BIT_IDX 12 // use to be 13 before above issue was noticed
#else
#define MAG_SNS_AS5048A_BIT_RES 14
#define MAG_SNS_ANGLE_START_BIT_IDX 13 
#endif 

#define MAG_SNS_ANGLE_REG_ADDR 0x3FFF
#define MAG_SNS_PARITY_BIT_IDX 15
#define MAG_SNS_nW_R_BIT_IDX 14

/** Shunt Resistor Resistance in Ohms 
 * NOTE: 500uOhms is default, but i've replaced the value with a 10mOhm shunt */
#define SHUNT_RESISTANCE_OHMS 0.010f // 

/** Lowside Motor Current sensor gain. 
 * This can be changed via spi comms by writing to a reg, by default it's 10 */
#define MOTOR_CURR_SNS_GAIN 10.0f 

#define SPI_CLOCK_SPEED_HZ_MAG 1000000














//////////////////////////// CLASS: DECLARATIONS: ////////////////////////////

bool shut_down_power_fault_active = false;

//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional input)
Encoder encoder = Encoder(ENC_A, ENC_B, /*ENCODER_CPR*/ENCODER_PPR/*, ENC_Z*/);

BLDCMotor motor = BLDCMotor( NUM_OF_POLE_PAIRS );

BLDCDriver6PWM driver = BLDCDriver6PWM(AH, AL, BH, BL, CH, CL, EN_GATE);

/** NOTE: both driver and drv_ic classes have control over 
 * the EN_GATE, but the only time drv_ic uses it is on initialization. 
 * 
 * after the drv_ic's init function is called only the "driver" class
 * has control of the EN_GATE pin. */
Drv8301 drv_ic = Drv8301(SPI_nCS_DRV, EN_GATE, nFAULT);

MagneticSensorSPIConfig_s AS5048A = {.spi_mode = SPI_MODE0, 
                                    .clock_speed = SPI_CLOCK_SPEED_HZ_MAG, 
                                    .bit_resolution = MAG_SNS_AS5048A_BIT_RES, 
                                    .angle_register = MAG_SNS_ANGLE_REG_ADDR, 
                                    .data_start_bit = MAG_SNS_ANGLE_START_BIT_IDX, 
                                    .command_rw_bit = MAG_SNS_nW_R_BIT_IDX, 
                                    .command_parity_bit = MAG_SNS_PARITY_BIT_IDX};

MagneticSensorSPI mag_sense = MagneticSensorSPI(AS5048A, SPI_nCS_IO6);

/** Params are: MOSI, MISO, SCLK, Chip select (optional). Chip select will be controlled 
 * by the calling function.
 */
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI_SCK); 

/** Note: SO1 is referenced to Phase B in HW, but in SW its references as Phase A.
 * Similary, SO2 is referenced as Phase C, but in software its reported as Phase B */
LowsideCurrentSense current_sense = LowsideCurrentSense(SHUNT_RESISTANCE_OHMS, 
                                                        MOTOR_CURR_SNS_GAIN, 
                                                        SO2, 
                                                        SO1);

HardwareSerial MySerial(GPIO4_UART_RX, GPIO3_UART_TX);

STM32_CAN Can( CAN1, ALT );  //Use PB8/9 pins for CAN1.

Commander command = Commander(MySerial);// commander interface

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
















////////////////////////////////// DUMB: SHIT: /////////////////////////////////

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
  .exp_alpha_val = 0.05, // 0.25

  /** Swing-Up Controller: */
  .K = 0.0001f, //0.0015f   // overall 
  .A = 0.018f, //0.018f     // proportional to energy required to swing pendulum
  
  /** Swing-Down Controller: */
  .J = -0.001f,
  .B = 0.02,

  /** Upright Controller: */
  .I = -0.05f,//-0.05f    // overall
  .L = -0.3f, //, -0.3, -0.23f, -0.25f   // theta dot
  .M = 20.0f, //20.0f,    // phi
  .N = -1.5f, //-1.5f     // phi dot

  .D = 0.0f, // Print out pendulum info on serial if = 1

  .tau_prev = 0.0f, // used for setpoint iir
  .tau_iir_alpha = 0.5f,
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

uint32_t channel;
volatile uint32_t FrequencyMeasured, LastCapture = 0, CurrentCapture;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0;
HardwareTimer *MyTim;


CAN_message_t CAN_TX_msg = {
  .id = 0x040,
  .len = 8,
};


typedef struct
{
  uint32_t first_bit:1;
  uint32_t theta_dot:11;
  uint32_t phi:8;
  uint32_t theta:8;
  uint32_t state:2;
  uint32_t rc:2;
} second_half_of_can_msg_bit;

typedef union 
{
  second_half_of_can_msg_bit bit;
  uint32_t all;
} second_half_of_can_msg;











////////////////////////////////// FUNCTION: DEFINITIONS: /////////////////////////////////

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

  #if 0
  switch(spinny.control.state)
  {
    case PDM_STATE_SWING_UP:
      if(spinny.pdm_theta < spinny.control.H || spinny.pdm_theta > (_2PI - spinny.control.H))
      {
        tmp_q_curr_req = get_upright_setpoint();
        spinny.control.state = PDM_STATE_UPRIGHT;
      }
      else 
        tmp_q_curr_req = get_swing_up_setpoint();
    break;

    case PDM_STATE_UPRIGHT:
      if(spinny.pdm_theta > spinny.control.P || spinny.pdm_theta < (_2PI - spinny.control.P))
      {
        tmp_q_curr_req = get_swing_down_setpoint();
        spinny.control.state = PDM_STATE_SWING_DOWN;
      }
      else 
        tmp_q_curr_req = get_upright_setpoint();
    break;

    case PDM_STATE_SWING_DOWN:
      if(spinny.pdm_theta < 4.14f && spinny.pdm_theta > 2.14f
      && spinny.control.swing_down_to_swing_up_cntr < 2500)
      {
        tmp_q_curr_req = 0;
        spinny.control.swing_down_to_swing_up_cntr++;
      }
      else if(spinny.pdm_theta < 4.14f && spinny.pdm_theta > 2.14f
      & spinny.control.swing_down_to_swing_up_cntr >= 2500)
      {
        tmp_q_curr_req = get_swing_up_setpoint();
        spinny.control.state = PDM_STATE_SWING_UP;
        spinny.control.swing_down_to_swing_up_cntr = 0;
      }
      else 
        tmp_q_curr_req = get_swing_down_setpoint();
    break;

    default:
      // do nothing
    break;
  }

  if(spinny.pdm_theta < spinny.control.PH || spinny.pdm_theta > (_2PI - spinny.control.PH))
  {
    tmp_q_curr_req = get_upright_setpoint();
    spinny.control.PH = spinny.control.P;
  }
  else 
  {
    tmp_q_curr_req = get_swing_up_setpoint();
    spinny.control.PH = spinny.control.H;
  }

  #else 
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
       * the pendulum doesn't kick itself up after 2/5 seconds.  
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
        spinny.K = 0.0025f;
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
  #endif 

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
    uint16_t tmp_bits = (uint16_t)((motor.current_sp * 1000.0f)+1024.0f);

    CAN_TX_msg.buf[0] = (uint8_t)(tmp_bits & 0x00FF);
    CAN_TX_msg.buf[1] = (uint8_t)((tmp_bits >> 8) & 0x0007);

    tmp_bits = (uint16_t)((motor.current.q * 1000.0f)+1024.0f);
    CAN_TX_msg.buf[1] |= (uint8_t)((tmp_bits & 0x001F) << 3);
    CAN_TX_msg.buf[2] =  (uint8_t)((tmp_bits >> 5) & 0x003F);
    tmp_bits = (uint16_t)((spinny.pdm_theta_dot_filt * 10.0f)+1024.0f);
    CAN_TX_msg.buf[2] |=  (uint8_t)((tmp_bits & 0x0003) << 6);
    CAN_TX_msg.buf[3]  =  (uint8_t)((tmp_bits >> 2));

    second_half_of_can_msg tx_info;
    tx_info.bit.first_bit = ((tmp_bits >> 10) & 0x0001);
    tx_info.bit.theta_dot = (uint32_t)((motor.shaft_velocity * 10.0f)+1024.0f) & 0x000007FF;
    tx_info.bit.theta = (uint32_t)((encoder.getMechanicalAngle()+3.2f) * 40.0f) & 0x000000FF;
    tx_info.bit.phi = (uint32_t)((spinny.pdm_phi+3.2f) * 40.0) & 0x000000FF;
    tx_info.bit.state = 0;
    static uint32_t rolling_counter = 0;
    tx_info.bit.rc = rolling_counter & 0x00000003;
    tx_info.bit.state = spinny.control.state & 0x00000003;
    rolling_counter++;
    rolling_counter &= 0x00000003;
    *(uint32_t*)&CAN_TX_msg.buf[4] = tx_info.all;
    Can.write(CAN_TX_msg);
  }

  motor.move(tmp_q_curr_req);
  motor.loopFOC();

  digitalWrite(GPIO2, LOW); 
}

// channel A and B callbacks
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
void doZ() { encoder.handleIndex(); }


void on_motor(char* cmd){ command.motor(&motor, cmd); }
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
      MySerial.print(F("exp filter alpha val set to: "));
      MySerial.println(value);
      break;
    case 'K':      
      spinny.K = value;
      MySerial.print(F("K coeff set to: "));
      MySerial.println(value);
      break;
    case 'A':      
      spinny.A = value;
      MySerial.print(F("A coeff set to: "));
      MySerial.println(value);
      break;
    case 'J':      
      spinny.J = value;
      MySerial.print(F("J coeff set to: "));
      MySerial.println(value);
      break;
    case 'B':      
      spinny.B = value;
      MySerial.print(F("B coeff set to: "));
      MySerial.println(value);
      break;
    case 'I':      
      spinny.I = value;
      MySerial.print(F("I coeff set to: "));
      MySerial.println(value);
      break;
    case 'L':      
      spinny.L = value;
      MySerial.print(F("L coeff set to: "));
      MySerial.println(value);
      break;
    case 'M':      
      spinny.M = value;
      MySerial.print(F("M coeff set to: "));
      MySerial.println(value);
      break;
    case 'N':      
      spinny.N = value;
      MySerial.print(F("N coeff set to: "));
      MySerial.println(value);
      break;
    case 'P':      
      spinny.control.P = value;
      MySerial.print(F("P coeff set to: "));
      MySerial.println(value);
      break;
    case 'H':      
      spinny.control.H = value;
      MySerial.print(F("H coeff set to: "));
      MySerial.println(value);
      break;
    case 'T':      
      spinny.tau_iir_alpha = value;
      MySerial.print(F("Tau exp filt alpha coeff set to: "));
      MySerial.println(value);
      break;
    case 'R':      
      spinny.tau_ramp_rate_limit = value;
      MySerial.print(F("Tau exp filt alpha coeff set to: "));
      MySerial.println(value);
      break;
    case 'S':      
      spinny.S = value;
      MySerial.print(F("Tau ramp/filt setting set to: "));
      MySerial.println(value);
      break;
    case 'D':      
      spinny.D = value;
      MySerial.print(F("Debug info flag set to: "));
      MySerial.println(value);
      break;
    default:
      // do nothing
      break;
  }
}














////////////////////////////////// SETUP: /////////////////////////////////

void setup() 
{
  #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
    MySerial.begin(250000); //115200
  #endif

  #if 0 /** this is set up and used by the MySerial class (uart/serial) */
  pinMode(GPIO3_UART_TX, INPUT);
  pinMode(GPIO4_UART_RX, INPUT);
  #endif 

  /** only used for timing analysis for now */
  pinMode(GPIO1, OUTPUT);
  pinMode(GPIO2, OUTPUT);

  // Automatically retrieve TIM instance and channel associated to FLOATING_M1_CH_TIM3
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(FLOATING_M1_CH_TIM3), PinMap_PWM);
  MyTim = new HardwareTimer(Instance);

  /** Prescaler is automatically set when setting overflow with format != tick */
  MyTim->setOverflow(200UL, MICROSEC_FORMAT);
  MyTim->attachInterrupt(Update_IT_callback);

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

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  /** set limits to a low value in case there are issues with FOC init */
  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit = 12.0f; // this is only used for centering pwm phase voltages
  motor.voltage_limit = 4.0f;
  motor.current_limit = 1.0f;

  /** NOTE: low pass filter implementation can be found by ctrl+shift+f this:
   * LowPassFilter::operator() 
   * 
   * The LPF is an exponential filter with an alpha value of .tf/(.tf + dt) 
   * so it is a function of time delta between sensor readings */

  motor.LPF_velocity.Tf = 0.01f; 

  motor.LPF_current_q.Tf = 0.004f; // old is 0.0005f
  motor.PID_current_q.P = 70.0f;  // old is 100
  motor.PID_current_q.I = 30.0f;  // old is 50
  motor.PID_current_q.D = 0.0f;

  motor.LPF_current_d.Tf = 0.004f; // old is 0.001f
  motor.PID_current_d.P = 70.0f;  // old is 100
  motor.PID_current_d.I = 30.0f;   // old is 50
  motor.PID_current_d.D = 0.0f;

  mag_sense.init(&SPI_2);  // init magnetic angle sensor
  mag_sense.update();

  set_theta_offset_when_pointed_down();

  encoder.init();
  encoder.enableInterrupts(doA,doB,doZ);

  motor.linkSensor(&encoder);  // link motor and sensor
  driver.init();              // init driver
  motor.linkDriver(&driver);  // link motor and driver
  current_sense.linkDriver(&driver);  // link the driver with the current sense
  motor.linkCurrentSense(&current_sense);  // link motor and current sense

  drv_ic.link_spi_class(&SPI_2);
  if(drv_ic.init(GainSetting_e::GainSetting_40_V_over_V) != true)
  {
    shut_down_power_fault_active = true;

    #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
      MySerial.println("Failed to set gain settings for DRV8301!!!!!");
    #endif

    return;
  }

  #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
    motor.useMonitoring(MySerial); 
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE; 
    motor.monitor_downsample = 0; // downsampling, 0 = disabled to start
  #endif

  /** This will speed up the initialization of the motor during the "initFOC"
   * if the zero_electric_angle is set: */
  //motor.zero_electric_angle = 0.0f;

  motor.init();  // init motor
  current_sense.init();  // init current sense
  motor.initFOC(); // align encoder and start FOC

  iq_sp_biquad.set_steady_state_val(0.0f);

  motor.move(0.0);
  motor.disable();
  
  Can.begin();
  Can.setBaudRate(1000000);  //1000KBPS

  /** set each limit to an appropriate value now that the motor has been initialized.
   * NOTE: because we use FOC_Current based torque control, voltage_limit is not limiting control
   * of the motor, but we will set driver and motor voltage limit for clarity.  Driver 
   * voltage limit is used to center the phase voltages. */
  motor.voltage_limit = 11.0f;
  motor.current_limit = 1.0f;  
  motor.PID_current_d.limit = 11.0f;
  motor.PID_current_q.limit = 11.0f; 

  command.add('M', on_motor,"my motor motion");
  command.add('C', on_constants,"Control Constants");

  command.run();
  motor.monitor();
  
  MyTim->resume();

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
    MySerial.print(spinny.pdm_phi, 6); //
    MySerial.print("    ");
    MySerial.print(spinny.pdm_theta_dot_filt, 6);
    MySerial.print("    ");
    MySerial.print(encoder.getVelocity(), 6);
    MySerial.print("    ");
    MySerial.print(encoder.getMechanicalAngle(), 6);
    MySerial.print("    ");
    MySerial.println(spinny.offset.median_phi_in_revolution, 6);
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

#if 0
  if(spinny.control.state == PDM_STATE_RESET)
  {
    spinny.control.reset_loop_wait_cnt++;
    if(spinny.control.reset_loop_wait_cnt >= 1000000)
    {
      spinny.control.reset_loop_wait_cnt = 0;
      spinny.control.state = PDM_STATE_SWING_UP;
      motor.enable();
    }
  }
#endif 
  
  //digitalWrite(GPIO2, LOW);
}


