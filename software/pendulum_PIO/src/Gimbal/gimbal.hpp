#ifndef __GIMBAL_HPP
#define __GIMBAL_HPP

#include "Arduino.h"

#include "../lib/Arduino-FOC/src/SimpleFOC.h"

/** Gimbal software component will act as a wrapper of sorts 
 * for the FOC based control of the gimbal motor with 
 * use of an encoder as positional feedback. */

extern Encoder encoder;
extern BLDCMotor motor;
extern BLDCDriver6PWM sw_bldc_driver;
extern LowsideCurrentSense current_sense;

void gimbal_init(Print &print);

#endif // __GIMBAL_HPP
