
#include <Arduino.h>
#include "STM32_CAN.h"




STM32_CAN Can( CAN1, ALT );  //Use PB8/9 pins for CAN1.

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

CAN_message_t CAN_TX_msg = 
{
    .id = 0x040,
    .len = 8,
};


/** Initialize the CAN driver. */
void can_tp_init(void)
{
  Can.begin();
  Can.setBaudRate(1000000);  //1000KBPS
}


/** Packs and sends the CAN message "Debug1" from the DBC. */
void inline send_can_Debug1_message(float q_curr_sp, 
                                    float q_curr_fb, 
                                    float phi_dot, 
                                    float theta_dot,
                                    float phi, 
                                    float theta,
                                    uint16_t state) 
{
    uint16_t tmp_bits = (uint16_t)((q_curr_sp * 1000.0f)+1024.0f);

    CAN_TX_msg.buf[0] = (uint8_t)(tmp_bits & 0x00FF);
    CAN_TX_msg.buf[1] = (uint8_t)((tmp_bits >> 8) & 0x0007);

    tmp_bits = (uint16_t)((q_curr_fb * 1000.0f)+1024.0f);
    CAN_TX_msg.buf[1] |= (uint8_t)((tmp_bits & 0x001F) << 3);
    CAN_TX_msg.buf[2] =  (uint8_t)((tmp_bits >> 5) & 0x003F);
    tmp_bits = (uint16_t)((phi_dot * 10.0f)+1024.0f);
    CAN_TX_msg.buf[2] |=  (uint8_t)((tmp_bits & 0x0003) << 6);
    CAN_TX_msg.buf[3]  =  (uint8_t)((tmp_bits >> 2));

    second_half_of_can_msg tx_info;
    tx_info.bit.first_bit = ((tmp_bits >> 10) & 0x0001);
    tx_info.bit.theta_dot = (uint32_t)((theta_dot * 10.0f)+1024.0f) & 0x000007FF;
    tx_info.bit.phi = (uint32_t)((phi+3.2f) * 40.0) & 0x000000FF;
    tx_info.bit.theta = (uint32_t)((theta+3.2f) * 40.0f) & 0x000000FF;
    tx_info.bit.state = 0;
    static uint32_t rolling_counter = 0;
    tx_info.bit.rc = rolling_counter & 0x00000003;
    tx_info.bit.state = state & 0x00000003;
    rolling_counter++;
    rolling_counter &= 0x00000003;
    *(uint32_t*)&CAN_TX_msg.buf[4] = tx_info.all;
    Can.write(CAN_TX_msg);
}



