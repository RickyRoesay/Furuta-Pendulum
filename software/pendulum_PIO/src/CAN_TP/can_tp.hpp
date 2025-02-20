#ifndef __CAN_TP_HPP
#define __CAN_TP_HPP

#include "stdint.h"

/** This can tp layer uses the pazi88's STM32_CAN library.
 * 
 * The arduino library can be found here: https://github.com/pazi88/STM32_CAN.git */

/** Initialize the CAN driver. */
void can_tp_init(void);


/** Packs and sendsthe CAN message "Debug1" from the DBC. */
void inline send_can_Debug1_message(float q_curr_sp, 
                                    float q_curr_fb, 
                                    float phi_dot, 
                                    float theta_dot,
                                    float phi, 
                                    float theta,
                                    uint16_t state);




#endif // __CAN_TP_HPP
