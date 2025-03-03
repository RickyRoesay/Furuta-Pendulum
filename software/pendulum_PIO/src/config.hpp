#ifndef __CONFIG_HPP
#define __CONFIG_HPP

/********************** TESTING: STUFF: ***********************/
/** In an implementation mistake, I gave the Bx coefficients
 * the wrong polarity, which on most slower cutoff biquad gains
 * caused some kind of FPU over/underflow that crashed a lot of things.
 * 
 * I haven't confirmed it's an FPU overflow but that seems like the 
 * most likely explanation.
 * 
 * NOTE: Causing the FPU to over/underflow like this seems to 
 * cause a lot of issues including over current faults on the 
 * DRV8301.  For this reason, we will use it to test 
 * fault detection and response on the DRV8301 driver. */
//#define CRASH_FPU_USING_WRONG_BIQUAD_GAIN_POLARITIES



/********************** GIMBAL: CONFIG: ***********************/

/** Dead zone as a percentage/factor of the switching period. 
 * The default value is 2% (or 0.02) but we are dividing that by 4
 * since it seems like way more than we need when looking at the 
 * H-bridge switch node voltage waveforms. 
 * 
 * With this new value the diodes on the H-bridges are conducting current
 * (hard switching, or partial/full ZVS) for about 140ns which seems 
 * appropriate. The MOSFET's that are used on the ODESC 4.2 have a part 
 * number of .
 * 
 * I couldn't find app notes for this family of FET's (P/N NTMFS5C628NL)
 * or recommended deadtime but the switching fall time (70ns) 
 * and the turn-off delay time (28ns)
 * are both well under the present deadtime. */
#define SW_BLDC_DRV__DEAD_ZONE__AS_FACTOR_OF_SW_PRD 0.005f

/** PSU voltage is what the sw driver uses to scale
 * the DQ voltage setpoints to the duty cycle 
 * of the h-bridges. */
#define SW_BLDC_DRV__V_PSU 20.0f
#define SW_BLDC_DRV__V_LIMIT 20.0f // this is only used for centering pwm phase voltages

/** set "pre foc init" limits to a low value in case there are issues 
 * with the HW before/during FOC init. 
 * NOTE: If MOTOR__V_LIMIT_PRE_FOC_INIT is < 3V, it will limit motor.voltage_sensor_align
 * voltage value from the default of 3.0 volts.  However,
 * with this motor we don't need to bother going below 3 volts during 
 * sensor alignment to the d axis.  */
#define MOTOR__V_LIMIT_PRE_FOC_INIT 4.0f 
#define MOTOR__V_LIMIT_POST_FOC_INIT 8.0f
#define MOTOR__I_LIMIT_PRE_FOC_INIT 1.0f
#define MOTOR__I_LIMIT_POST_FOC_INIT 1.0f

/** NOTE: low pass filter implementation can be found by ctrl+shift+f this:
 * LowPassFilter::operator() 
 * 
 * The LPF is an exponential filter with an alpha value of .tf/(.tf + dt) 
 * so it is a function of time delta between sensor readings */
#define MOTOR__VELOCITY_LPF_TF 0.04f  // old is 0.01f

#define MOTOR__Q_CURR_MEAS_FILT_TF  0.001f     // old is 0.004f
#define MOTOR__Q_CURR_PID_P_GAIN    200.0f     // old is 12
#define MOTOR__Q_CURR_PID_I_GAIN    100.0f     // old is 3
#define MOTOR__Q_CURR_PID_D_GAIN    0.0f
#define MOTOR__Q_CURR_PID_SAT_LIMIT     MOTOR__V_LIMIT_POST_FOC_INIT  // this is in units of voltage

#define MOTOR__D_CURR_MEAS_FILT_TF  0.02f       // old is 0.02f
#define MOTOR__D_CURR_PID_P_GAIN    0.0f     // old is 2000.0f
#define MOTOR__D_CURR_PID_I_GAIN    0.0f      // old is 600.0f
#define MOTOR__D_CURR_PID_D_GAIN    0.0f
#define MOTOR__D_CURR_PID_SAT_LIMIT     MOTOR__V_LIMIT_POST_FOC_INIT   // this is in units of voltage

/** To speed up startup time by over a second, we can
 * skip the pole pair, minimum movement, and direction checks 
 * by setting the "motor.sensor_direction" before calling initFOC.  
 * 
 * This requires:
 *   - Knowing what the direction is (on startup the detected 
 *     direction is printed out via serial comm)
 *   - Knowing there is not an issue with the physical 
 *     construction, wiring, or circuitry of the pendulum.  */
#define SKIP_POLE_PAIR_AND_DIRECTION_CHECKS

#ifdef SKIP_POLE_PAIR_AND_DIRECTION_CHECKS
#define MOTOR__SENSOR_DIRECTION    Direction::CW
#else
#define MOTOR__SENSOR_DIRECTION    Direction::UNKNOWN
#endif 





/** Number of pole pairs on the the GB54-1 is 7, config is 12n14p */
#define NUM_OF_POLE_PAIRS 7

/** Encoder Pulse/Counts per Revolution = 2048 AM103, default switch config */
#define ENCODER_PPR 2048UL

/** Shunt Resistor Resistance in Ohms 
 * NOTE: 500uOhms is default for the ODESC 4.2, but i've replaced the value
 * with a 10mOhm shunt to get better resolution for the amperage the phases 
 * will see in the GB54-2 gimbal motor. */
#define SHUNT_RESISTANCE_OHMS 0.200f 

/** Even with 80V/V of gain, 10mOhm may be too small of a value.
 * Next batch of electronics parts I buy I'll get a 100mOhm sensor to 
 * try out. */

/** Lowside Motor Current sensor gain. 
 * This can be changed via spi comms by writing to a reg, by default it's 10. */
#define MOTOR_CURR_SNS_GAIN 10.0f 
#define DRV8301_GAIN_SETTING DRV8301_GainSetting_10_V_over_V






#endif // __CONFIG_H
