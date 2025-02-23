
#include "gimbal.hpp"


#include "config.hpp"
#include "gpio.h"





//  Encoder(int encA, int encB , int cpr, int index)
//  - encA, encB    - encoder A and B pins
//  - ppr           - impulses per rotation  (cpr=ppr*4)
//  - index pin     - (optional) we don't use this since it only serves to slow
//                    down the startup time when the initFOC function tries to find a 0.
Encoder encoder = Encoder(ENC_A, ENC_B, ENCODER_PPR);

BLDCMotor motor = BLDCMotor( NUM_OF_POLE_PAIRS );

BLDCDriver6PWM sw_bldc_driver = BLDCDriver6PWM(AH, AL, BH, BL, CH, CL, EN_GATE);

/** Note: SO1 is referenced to Phase B in HW, but in SW its references as Phase A.
 * Similary, SO2 is referenced as Phase C, but in software its reported as Phase B */
LowsideCurrentSense current_sense = LowsideCurrentSense(SHUNT_RESISTANCE_OHMS, 
    MOTOR_CURR_SNS_GAIN, 
    SO2, 
    SO1);


// channel A and B callbacks
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
//void doZ() { encoder.handleIndex(); }



void gimbal_init(Print &print)
{
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
  
    /** set limits to a low value in case there are issues with FOC init */
    sw_bldc_driver.voltage_power_supply = SW_BLDC_DRV__V_PSU;
    sw_bldc_driver.voltage_limit = SW_BLDC_DRV__V_LIMIT; // this is only used for centering pwm phase voltages
    motor.voltage_limit = MOTOR__V_LIMIT_PRE_FOC_INIT;
    motor.current_limit = MOTOR__I_LIMIT_PRE_FOC_INIT;
  
    /** NOTE: low pass filter implementation can be found by ctrl+shift+f this:
     * LowPassFilter::operator() 
     * 
     * The LPF is an exponential filter with an alpha value of .tf/(.tf + dt) 
     * so it is a function of time delta between sensor readings */
    motor.LPF_velocity.Tf = MOTOR__VELOCITY_LPF_TF; 
  
    motor.LPF_current_q.Tf = MOTOR__Q_CURR_MEAS_FILT_TF; // old is 0.004f
    motor.PID_current_q.P = MOTOR__Q_CURR_PID_P_GAIN;  // old is 70
    motor.PID_current_q.I = MOTOR__Q_CURR_PID_I_GAIN;  // old is 30
    motor.PID_current_q.D = MOTOR__Q_CURR_PID_D_GAIN;
    
    motor.LPF_current_d.Tf = MOTOR__D_CURR_MEAS_FILT_TF; // old is 0.004f
    motor.PID_current_d.P = MOTOR__D_CURR_PID_P_GAIN;  // old is 70
    motor.PID_current_d.I = MOTOR__D_CURR_PID_I_GAIN;   // old is 30
    motor.PID_current_d.D = MOTOR__D_CURR_PID_D_GAIN;
    
    encoder.init();
    encoder.enableInterrupts(doA,doB);
    
    motor.linkSensor(&encoder);  // link motor and sensor
    sw_bldc_driver.init();              // init sw_bldc_driver
    motor.linkDriver(&sw_bldc_driver);  // link motor and sw_bldc_driver
    current_sense.linkDriver(&sw_bldc_driver);  // link the sw_bldc_driver with the current sense
    motor.linkCurrentSense(&current_sense);  // link motor and current sense
    
    motor.useMonitoring(print); 
    motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D | _MON_CURR_Q | _MON_CURR_D | _MON_VEL | _MON_ANGLE; 
    motor.monitor_downsample = 0; // downsampling, 0 = disabled to start
    
    /** Depending on if SKIP_POLE_PAIR_AND_DIRECTION_CHECKS is 
     * defined in config.hpp, this will speed up the initialization of the 
     * motor during the "initFOC" if the direction is set beforehand: */
    motor.sensor_direction = MOTOR__SENSOR_DIRECTION;
    
    motor.init();  // init motor
    current_sense.init();  // init current sense
    motor.initFOC(); // align encoder and start FOC
    
    motor.move(0.0);
    motor.disable();
    
    /** set each limit to an appropriate value now that the motor has been initialized.
     * NOTE: because we use FOC_Current based torque control, voltage_limit is not limiting control
     * of the motor, but we will set sw_bldc_driver and motor voltage limit for clarity.  sw_bldc_driver 
     * voltage limit is used to center the phase voltages. */
    motor.voltage_limit = MOTOR__V_LIMIT_POST_FOC_INIT;
    motor.current_limit = MOTOR__I_LIMIT_POST_FOC_INIT;  
    
    /** NOTE: PID current D and Q saturation limit values
     *  need to be set after the init function call of the motor class 
     * if the alignment voltage needs to be less than the default of 3V 
     * (set by DEF_VOLTAGE_SENSOR_ALIGN in defaults.h file).  
     * 
     * If the motor.voltage_limit is < DEF_VOLTAGE_SENSOR_ALIGN at the time 
     * of calling motor.init(), motor.voltage_sensor_align will be 
     * limited to motor.voltage_limit.  Thus, in this implementation,
     * to make the alignment voltage < 3V, we write a lower value of a voltage init
     * before calling motor.init.print
     * 
     * Once motor.init is done, to set the dq current PID saturation limit values 
     * higher than the "voltage_limit" at the time of calling the function,
     * we must set them afterwards. */
    motor.PID_current_d.limit = MOTOR__D_CURR_PID_SAT_LIMIT;
    motor.PID_current_q.limit = MOTOR__Q_CURR_PID_SAT_LIMIT;   
}





