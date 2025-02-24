
#include "pendulum.hpp"

#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc
#include "Gimbal/gimbal.hpp"








pdm_info_s pdm = {
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
    .L = -0.9f, //, -0.3, -0.23f, -0.25f   // theta dot
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
  
  
  









  float get_upright_setpoint(void)
  {
    float tmp_q_curr_req = pdm.I * ((pdm.L * motor.shaft_velocity ) 
                          + (pdm.M * pdm.pdm_phi ) 
                          + (pdm.N * pdm.pdm_theta_dot_filt));
  
    return tmp_q_curr_req;
  }
  
  float get_swing_up_setpoint(void)
  {
    float cos_th_pow4 = _cos(pdm.pdm_theta)*_cos(pdm.pdm_theta)*_cos(pdm.pdm_theta)*_cos(pdm.pdm_theta);
    float cos_th_pow2 = _cos(pdm.pdm_theta)*_cos(pdm.pdm_theta);
    float th_dot_pow2 = pdm.pdm_theta_dot_filt*pdm.pdm_theta_dot_filt;
  
    float tmp_q_curr_req = pdm.K * cos_th_pow4 * pdm.pdm_theta_dot_filt * ((9.81f*(1.0f - _cos(pdm.pdm_theta))) - (pdm.A * th_dot_pow2));
    return tmp_q_curr_req;
  }
  
  float get_swing_down_setpoint(void)
  {
    float cos_th_pow4 = _cos(pdm.pdm_theta)*_cos(pdm.pdm_theta)*_cos(pdm.pdm_theta)*_cos(pdm.pdm_theta);
    float th_dot_pow2 = pdm.pdm_theta_dot_filt*pdm.pdm_theta_dot_filt;
  
    //float tmp_q_curr_req = pdm.J * cos_th_pow4 * pdm.pdm_theta_dot_filt * ((pdm.B * th_dot_pow2) - (9.81f*(1.0f - _cos(pdm.pdm_theta))));
    float tmp_q_curr_req = pdm.J * cos_th_pow4 * pdm.pdm_theta_dot_filt * ((9.81f*(1.0f - _cos(pdm.pdm_theta))) - (pdm.B * th_dot_pow2));
    return tmp_q_curr_req;
  }
  
  
  
  /** These functions requires the mag sensor to be initialized! */
  void set_theta_offset_when_pointed_down(void)
  {
    /** Set theta offset for pendulum angle.  Having "+ _PI" in the offset 
     * helps choose an initial value that will align with phi = 0 being the 
     * upright position. */
    pdm.offset.theta_offset = mag_sense.getMechanicalAngle() + _PI; // 180deg = down
    if(pdm.offset.theta_offset >= _2PI) 
      pdm.offset.theta_offset -= _2PI; 
  }
  void set_theta_offset_when_pointed_up(void)
  {
    pdm.offset.theta_offset = mag_sense.getMechanicalAngle(); // 0deg = upright
    if(pdm.offset.theta_offset >= _2PI) 
      pdm.offset.theta_offset -= _2PI;
  }
  void set_theta_offset_when_free_spinning(float median_phi)
  {
    pdm.offset.theta_offset -= median_phi;
    if(pdm.offset.theta_offset >= _2PI) 
      pdm.offset.theta_offset -= _2PI;
  }
  


















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
        pdm.exp_alpha_val = value;
        hw_serial.print(F("exp filter alpha val set to: "));
        hw_serial.println(value);
        break;
      case 'K':      
        pdm.K = value;
        hw_serial.print(F("K coeff set to: "));
        hw_serial.println(value);
        break;
      case 'A':      
        pdm.A = value;
        hw_serial.print(F("A coeff set to: "));
        hw_serial.println(value);
        break;
      case 'J':      
        pdm.J = value;
        hw_serial.print(F("J coeff set to: "));
        hw_serial.println(value);
        break;
      case 'B':      
        pdm.B = value;
        hw_serial.print(F("B coeff set to: "));
        hw_serial.println(value);
        break;
      case 'I':      
        pdm.I = value;
        hw_serial.print(F("I coeff set to: "));
        hw_serial.println(value);
        break;
      case 'L':      
        pdm.L = value;
        hw_serial.print(F("L coeff set to: "));
        hw_serial.println(value);
        break;
      case 'M':      
        pdm.M = value;
        hw_serial.print(F("M coeff set to: "));
        hw_serial.println(value);
        break;
      case 'N':      
        pdm.N = value;
        hw_serial.print(F("N coeff set to: "));
        hw_serial.println(value);
        break;
      case 'P':      
        pdm.control.P = value;
        hw_serial.print(F("P coeff set to: "));
        hw_serial.println(value);
        break;
      case 'H':      
        pdm.control.H = value;
        hw_serial.print(F("H coeff set to: "));
        hw_serial.println(value);
        break;
      case 'T':      
        pdm.tau_iir_alpha = value;
        hw_serial.print(F("Tau exp filt alpha coeff set to: "));
        hw_serial.println(value);
        break;
      case 'R':      
        pdm.tau_ramp_rate_limit = value;
        hw_serial.print(F("Tau exp filt alpha coeff set to: "));
        hw_serial.println(value);
        break;
      case 'S':      
        pdm.S = value;
        hw_serial.print(F("Tau ramp/filt setting set to: "));
        hw_serial.println(value);
        break;
      case 'D':      
        pdm.D = value;
        hw_serial.print(F("Debug info flag set to: "));
        hw_serial.println(value);
        break;
      default:
        // do nothing
        break;
    }
  }
  
  
  








