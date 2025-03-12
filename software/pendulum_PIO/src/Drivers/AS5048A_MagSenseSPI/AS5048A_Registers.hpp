#ifndef AS5048A_REGISTER_STRUCTS_H
#define AS5048A_REGISTER_STRUCTS_H

#include "stdint.h"



struct ErrFlags_s {
  uint16_t framing_error:1U;
  uint16_t command_inv:1U;
  uint16_t parity_error:1U;
  uint16_t ErrFlags_unused:11U;
  uint16_t ErrFlags_RWn_or_Error_Flag:1U;
  uint16_t ErrFlags_PAR:1U;
};
union ErrFlags_u {
    uint16_t raw;
    ErrFlags_s bit; 
};

/** See the first section of the 
 * "application information" chapter for information on 
 * programming the zero position on the AS5048 */
struct ProgCtrl_s {
  uint16_t programming_enable:1U;
  uint16_t ProgCtrl_unused_1:2U;
  uint16_t burn:1U;
  uint16_t ProgCtrl_unused_2:2U;
  uint16_t verify:1U;
  uint16_t ProgCtrl_unused_3:7U;
  uint16_t ProgCtrl_RWn_or_Error_Flag:1U;
  uint16_t ProgCtrl_PAR:1U;
};
union ProgCtrl_u {
    uint16_t raw;
    ProgCtrl_s bit; 
};

struct OTP_RegZeroPosHigh_s {
  uint16_t zero_position_bits_6_to_13:8U;
  uint16_t OTP_RegZeroPosHigh_unused:6U;
  uint16_t OTP_RegZeroPosHigh_RWn_or_Error_Flag:1U;
  uint16_t OTP_RegZeroPosHigh_PAR:1U;
};
union OTP_RegZeroPosHigh_u {
    uint16_t raw;
    OTP_RegZeroPosHigh_s bit; 
};

struct OTP_RegZeroPosLow_s {
  uint16_t zero_position_bits_0_to_5:6U;
  uint16_t OTP_RegZeroPosLow_unused:8U;
  uint16_t OTP_RegZeroPosLow_RWn_or_Error_Flag:1U;
  uint16_t OTP_RegZeroPosLow_PAR:1U;
};
union OTP_RegZeroPosLow_u {
    uint16_t raw;
    OTP_RegZeroPosLow_s bit; 
};

struct DiagAndAutoGainCtrl_s {
  uint16_t AGC_value:8U;
  uint16_t offset_comp_finished_OCF:1U;
  uint16_t cordis_ovf_COF:1U;
  uint16_t comp_low:1U;
  uint16_t comp_high:1U;
  uint16_t DiagAndAutoGainCtrl_unused:2U;
  uint16_t DiagAndAutoGainCtrl_RWn_or_Error_Flag:1U;
  uint16_t DiagAndAutoGainCtrl_PAR:1U;
};
union DiagAndAutoGainCtrl_u {
    uint16_t raw;
    DiagAndAutoGainCtrl_s bit; 
};

struct Magnitude_s {
  uint16_t magnitude_data:14U;
  uint16_t Magnitude_RWn_or_Error_Flag:1U;
  uint16_t Magnitude_PAR:1U;
};
union Magnitude_u {
    uint16_t raw;
    Magnitude_s bit; 
};

struct Angle_s {
  uint16_t angle_data:14U;
  uint16_t Angle_RWn_or_Error_Flag:1U;
  uint16_t Angle_PAR:1U;
};
union Angle_u {
    uint16_t raw;
    Angle_s bit; 
};


struct GenericRegister_s {
  uint16_t generic_data:14U;
  uint16_t Generic_RWn_or_Error_Flag:1U;
  uint16_t Generic_PAR:1U;
};
union GenericRegister_u {
    uint16_t raw;
    GenericRegister_s bit; 
};

#endif
