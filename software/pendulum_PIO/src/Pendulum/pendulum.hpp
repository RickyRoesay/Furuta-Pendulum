#ifndef __PENDULUM_HPP
#define __PENDULUM_HPP

#include "stdint.h"
#include "DRV8301/drv8301.hpp" 
#include "AS5048A_MagSenseSPI/AS5048A_MagSenseSPI.hpp" // mag sensor
#include "CAN_TP/can_tp.hpp"
#include "Biquad.hpp"
#include "Gimbal/gimbal.hpp"

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




extern pdm_info_s pdm;


extern Drv8301 drv_ic;
extern AS5048A_MagSenseSPI mag_sense;
extern HardwareSerial hw_serial;
extern Commander command;





void on_constants(char* cmd);

float get_upright_setpoint(void);
float get_swing_up_setpoint(void);
float get_swing_down_setpoint(void);
void set_theta_offset_when_pointed_down(void);
void set_theta_offset_when_pointed_up(void);
void set_theta_offset_when_free_spinning(float median_phi);




#endif 

