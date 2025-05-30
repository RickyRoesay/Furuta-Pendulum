
#include "pendulum.hpp"

#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc
#include "Gimbal/gimbal.hpp"




void pdm_on_constants(char* cmd)
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











