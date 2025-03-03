
#include "pendulum.hpp"

#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc
#include "Gimbal/gimbal.hpp"
#include "Biquad.hpp"


#if 0
Biquad pdm_torque_setpoint_biquad_c = Biquad(0.06983610678422146f, 
                                              0.13967221356844292f,
                                              0.06983610678422146f,
                                              -1.1833219824875858f,
                                              0.46266640962447153f);
#else
Biquad pdm_torque_setpoint_biquad_c = Biquad(0.003656243805949168f, 
                                              0.007312487611898336f,
                                              0.003656243805949168f,
                                              -1.840089196941794f,
                                              0.8547141721655904f);
#endif 






pdm_info_s pdm = 
{
  .pdm_theta_dot_filt = 0.0f, 
  .pdm_theta = 0.0f, 
  .pdm_phi = 0.0f,   
  .exp_alpha_val = 0.25, // old value is 0.025

  /** Swing-Up Controller: */
  .K = 0.0003f, //0.0015f   // initial value 
  .A = 0.0001f, //0.018f     // proportional to energy required to swing pendulum
  
  /** Swing-Down Controller: */
  // nothing

  /** Upright Controller: */
  .I = -0.05f,//-0.05f    // overall
  .L = -0.9f, //, -0.3, -0.23f, -0.25f   // theta dot
  .M = 50.0f, //20.0f,    // phi
  .N = -1.5f, //-1.5f     // phi dot

  .D = 0.0f, // Print out pendulum info on serial if = 1


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
  
  
  

int pdm_run_control_loop(void)
{
  mag_sense.update();

  /** update theta and phi values. */
  pdm.pdm_theta = pdm.offset.theta_offset - mag_sense.getMechanicalAngle();

  if(pdm.pdm_theta < 0.0f) pdm.pdm_theta+=_2PI;
    pdm.pdm_phi = pdm.pdm_theta;

  if(pdm.pdm_phi > _PI)
    pdm.pdm_phi -= _2PI; 

  pdm.pdm_theta_dot_filt = (mag_sense.getVelocity() * pdm.exp_alpha_val) \
                            + ((1.0-pdm.exp_alpha_val) * pdm.pdm_theta_dot_filt);
  
  /** Check min and max phi values for offset correction */  
  if(pdm.offset.min_phi_in_revolution > pdm.pdm_phi)
    pdm.offset.min_phi_in_revolution = pdm.pdm_phi;
  
  if(pdm.offset.max_phi_in_revolution < pdm.pdm_phi)
    pdm.offset.max_phi_in_revolution = pdm.pdm_phi;
  
  float tmp_q_curr_req;
  
  switch(pdm.control.state)
  {
    case PDM_STATE_INIT:
    case PDM_STATE_RESET:
      tmp_q_curr_req = pdm_get_swing_up_setpoint();
      if(pdm.pdm_theta < 2.14 || pdm.pdm_theta > 4.14)
      {
        if(pdm.control.state == PDM_STATE_RESET)
          motor.enable();

        pdm.control.state = PDM_STATE_SWING_UP;
      }
    break;
    
    default:
    case PDM_STATE_SWING_UP:
    case PDM_STATE_UPRIGHT:
      if(pdm.pdm_theta < pdm.control.PH || pdm.pdm_theta > (_2PI - pdm.control.PH))
      {
        tmp_q_curr_req = pdm_get_upright_setpoint();
        pdm.control.PH = pdm.control.P;
     
        /** don't wait for the q current setpoint to ramp up with the filter */
        if(pdm.control.state != PDM_STATE_UPRIGHT)
          pdm_torque_setpoint_biquad_c.set_steady_state_val(tmp_q_curr_req);
       
        pdm.control.state = PDM_STATE_UPRIGHT;
      }
      else 
      {
        tmp_q_curr_req = pdm_get_swing_up_setpoint();
        pdm.control.PH = pdm.control.H;

        /** don't wait for the q current setpoint to ramp up with the filter */
        if(pdm.control.state != PDM_STATE_SWING_UP)
        {
          pdm.control.state = PDM_STATE_RESET;
          motor.disable();
        }
      }
    break;
  }

  tmp_q_curr_req = pdm_torque_setpoint_biquad_c.update_and_return_filt_value(tmp_q_curr_req);
  
  
  motor.move(tmp_q_curr_req);
  motor.loopFOC();

  return 1;
}

  





void pdm_run_phi_offset_correction_checks(void)
{
  int32_t num_of_full_rotations = encoder.getFullRotations();
  int32_t full_rotation_delta = num_of_full_rotations - pdm.offset.last_rotation_num_at_offset_check;
  float tmp_max_phi_delta;

  if(full_rotation_delta == 2 || full_rotation_delta == -2)
  {
    pdm.offset.last_rotation_num_at_offset_check = num_of_full_rotations;
    // if phi min - phi max > threshold, reset value
    tmp_max_phi_delta = pdm.offset.max_phi_in_revolution - pdm.offset.min_phi_in_revolution;
    if(tmp_max_phi_delta <= pdm.offset.phi_delta_thld_for_offset_restart)
    {
      if(pdm.control.state == PDM_STATE_UPRIGHT)
      {
        pdm.offset.median_phi_in_revolution = tmp_max_phi_delta / 2.0f;
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
}




  float pdm_get_upright_setpoint(void)
  {
    float tmp_q_curr_req = pdm.I * ((pdm.L * motor.shaft_velocity ) 
                          + (pdm.M * pdm.pdm_phi ) 
                          + (pdm.N * pdm.pdm_theta_dot_filt));
  
    return tmp_q_curr_req;
  }
  
  float pdm_get_swing_up_setpoint(void)
  {
    float cos_th_pow4 = _cos(pdm.pdm_theta)*_cos(pdm.pdm_theta)*_cos(pdm.pdm_theta)*_cos(pdm.pdm_theta);
    float cos_th_pow2 = _cos(pdm.pdm_theta)*_cos(pdm.pdm_theta);
    float th_dot_pow2 = pdm.pdm_theta_dot_filt*pdm.pdm_theta_dot_filt;
  
    float tmp_q_curr_req = pdm.K * cos_th_pow4 * pdm.pdm_theta_dot_filt * ((9.81f*(1.0f - _cos(pdm.pdm_theta))) - (pdm.A * th_dot_pow2));
    return tmp_q_curr_req;
  }
  
  float pdm_get_swing_down_setpoint(void)
  {
    float tmp_q_curr_req = pdm.I * ((pdm.L * motor.shaft_velocity ) 
                          + (pdm.M * pdm.pdm_phi ) 
                          + (pdm.N * pdm.pdm_theta_dot_filt));
  
    return tmp_q_curr_req;
  }
  
  
  
  /** These functions requires the mag sensor to be initialized! */
  void pdm_set_theta_offset_when_pointed_down(void)
  {
    /** Set theta offset for pendulum angle.  Having "+ _PI" in the offset 
     * helps choose an initial value that will align with phi = 0 being the 
     * upright position. */
    pdm.offset.theta_offset = mag_sense.getMechanicalAngle() + _PI; // 180deg = down
    if(pdm.offset.theta_offset >= _2PI) 
      pdm.offset.theta_offset -= _2PI; 
  }
  void pdm_set_theta_offset_when_pointed_up(void)
  {
    pdm.offset.theta_offset = mag_sense.getMechanicalAngle(); // 0deg = upright
    if(pdm.offset.theta_offset >= _2PI) 
      pdm.offset.theta_offset -= _2PI;
  }
  void pdm_set_theta_offset_when_free_spinning(float median_phi)
  {
    pdm.offset.theta_offset -= median_phi;
    if(pdm.offset.theta_offset >= _2PI) 
      pdm.offset.theta_offset -= _2PI;
  }
  












