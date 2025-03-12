#ifndef __PENDULUM_HPP
#define __PENDULUM_HPP

#include "stdint.h"
#include "Drivers/DRV8301_Gate_Driver/DRV8301_Gate_Driver.hpp" 
#include "Drivers/AS5048A_MagSenseSPI/AS5048A_MagSenseSPI.hpp" // mag sensor
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


  float I; // final upright coeff
  float L; // K matrix coeff for motor theta dot
  float M; // K matrix coeff for pendulum phi (angle from upright)
  float N; // K matrix coeff for pendulum phi dot

  float D; // Print out pendulum info on serial if = 1

  float tau_prev;
  float tau_iir_alpha;
  float tau_ramp_rate_limit;

  pdm_offset_s offset;
  pdm_control_s control;
} pdm_info_s;

extern pdm_info_s pdm;


extern DRV8301_Gate_Driver hw_bldc_driver;
extern AS5048A_MagSenseSPI mag_sense;
extern HardwareSerial hw_serial;
extern Commander command;

extern Biquad pdm_torque_setpoint_biquad_c;


/////////////////////////////////// CONTROL: FUNCTIONS: ///////////////////////////////////
int pdm_run_control_loop(void);

void pdm_run_phi_offset_correction_checks(void);

float pdm_get_upright_setpoint(void);

float pdm_get_swing_up_setpoint(void);

float pdm_get_swing_down_setpoint(void);


/////////////////////////////////// COMM: FUNCTIONS: ///////////////////////////////////
void pdm_on_constants(char* cmd);




void pdm_set_theta_offset_when_pointed_down(void);
void pdm_set_theta_offset_when_pointed_up(void);
void pdm_set_theta_offset_when_free_spinning(float median_phi);





#endif 

