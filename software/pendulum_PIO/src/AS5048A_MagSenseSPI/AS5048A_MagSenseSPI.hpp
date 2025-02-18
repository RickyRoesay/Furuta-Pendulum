#ifndef AS5048A_MYMAGSENSE_LIB_H
#define AS5048A_MYMAGSENSE_LIB_H

/** NOTE: 
 * This is basically copy and paste from SimpleFOC's MagneticSensorSPI
 * with the addition of AS5048A specific functions and diagnostics. */

#include "Arduino.h"
#include <SPI.h>
#include "../lib/Arduino-FOC/src/common/base_classes/Sensor.h"
#include "../lib/Arduino-FOC/src/common/foc_utils.h"
#include "../lib/Arduino-FOC/src/common/time_utils.h"
#include "AS5048A_Registers.hpp"



class AS5048A_MagSenseSPI: public Sensor{
public:

    /** MagneticSensorSPI class constructor
     * @param cs  SPI chip select pin, kept as an "int" to match the arduino library pin type. */
    AS5048A_MagSenseSPI(int cs);

    /** sensor initialise pins, reads all register data and  */
    bool init(SPIClass* _spi);

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;

    /** Returns the filtered velocity.  This will apply a filter to the 
     * most recently calculated velocity and return the value as a unit
     * of rad/s */
    float getFiltVelocity();

  



    /******* SENSOR: STATUS: ENUM: *******/
    /** This state will be updated at each function 
     * call of the base class's "update" function to signal 
     * if there hase been a fault  */
    typedef enum : word {
      AS5048A_MagSense_State__OK,
      AS5048A_MagSense_State__Comm_Fault,
      AS5048A_MagSense_State__Mag_Fault,
    } AS5048A_MagSenseSPI_State_e;




  private:
    
    /******* SPI: VARIABLES: *******/
    SPISettings settings; //!< SPI settings variable
    SPIClass* spi; 
    int chip_select_pin; //!< SPI chip select pin


    /******* FAULT: TYPES: AND: DEBUG: INFO: *******/
    typedef enum : uint16_t {
      AS5048A_FLT__NO_ERRORS = 0x0000,
      AS5048A_FLT__PARITY_ERROR = 0x0001, // this could be either errFlags->PAR bit high or a SW detected RX parity error
      AS5048A_FLT__COMMAND_INVALID = 0x0002, // command invalid flag was high in the err flags reg
      AS5048A_FLT__FRAMING_ERROR = 0x0004, // The details of what cause this are unclear in the DS
      AS5048A_FLT__OFFSET_COMP_UNFINISHED = 0x0008,  // offset compensation never finished
      AS5048A_FLT__CORDIC_OVERFLOW = 0x0010,         // overflow fault in the "computer" on the ic
      AS5048A_FLT__COMP_ERR_HIGH_MAG_FIELD = 0x0020, // comp LOW (mag field strength too high)
      AS5048A_FLT__COMP_ERR_LOW_MAG_FIELD = 0x0040,  // comp HIGH (mag field strength too low)
      AS5048A_FLT__WRITE_READ_MISMATCH = 0x0080, // readback of a register doesn't match what was written to it (SW detected fault)
      AS5048A_FLT__INVALID_SW_FUNC_PARAM = 0x0100,// an invalid parameter was passed to a low level driver (SW related fault)
      AS5048A_FLT__ERROR_FLAG_ACTIVE = 0x0200,// for when the error flag is active but we don't know which specific fault caused it
    } AS5048A_Faults_e;
    struct AS5048A_Faults_s {
      uint16_t parity_error:1U;
      uint16_t command_invalid_errFlag:1U; // command invalid bit on the AS5048 was returned (i.e. due to a signal issue)
      uint16_t framing_error:1U;
      uint16_t offset_comp_unfinished:1U;     // offset compensation never finished
      uint16_t cordic_overflow:1U;            // overflow fault in the "computer" on the ic
      uint16_t comp_err_high_mag_field:1U;    // comp LOW (mag field strength too high)
      uint16_t comp_err_low_mag_field:1U;     // comp HIGH (mag field strength too low)
      uint16_t write_value_mismatch:1U; // readback of a register doesn't match what was written to it (SW detected fault)
      uint16_t invalid_function_parameters:1U; // an invalid parameter was passed to a low level driver (SW related fault)
      uint16_t error_flag_active:1U; // for when the error flag is active but we don't know which specific fault caused it
      uint16_t AS5048A_Fault_unused:6U;
    };
    union AS5048A_Faults_u {
        uint16_t raw;
        AS5048A_Faults_s bit; 
        AS5048A_Faults_e all; 
    };
    AS5048A_Faults_u fault_status_on_startup; // this will mainly be used for internal debugging
    AS5048A_Faults_u fault_status; // this will mainly be used for internal debugging
    AS5048A_Faults_u fault_memory; // this will mainly be used for internal debugging

    uint32_t num_of_err_flags = 0;
    uint32_t num_of_mag_errors = 0;
    

    /******* SENSOR: STATUS: ENUM: *******/
    AS5048A_MagSenseSPI_State_e sensor_state; 
    

    /******* REGISTER: TYPES: AND: DATA: *******/
    /** Register Descriptions can be found on figure 22 of the datasheet */
    /** Struct/union declarations can be found in AS5048_Registers.hpp */

    /** Register address values.  These take up the 14 out 16 bits [13:0]
     * bits in the SPI Write command.  The other bits are the 
     * parity bit and the write/read flag as bit 15 and bit 14 respectively */
     typedef enum : word {
      AS5048_REG__NOP                  = 0x0000,
      AS5048_REG__ErrFlags             = 0x0001,
      AS5048_REG__ProgCtrl             = 0x0003,
      AS5048_REG__OTP_RegZeroPosHigh   = 0x0016,
      AS5048_REG__OTP_RegZeroPosLow    = 0x0017,
      AS5048_REG__DiagAndAutoGainCtrl  = 0x3FFD,
      AS5048_REG__Magnitude            = 0x3FFE,
      AS5048_REG__Angle                = 0x3FFF,
    } AS5048A_Register_e;

    struct AS5048A_Reg_Data_s {
      ErrFlags_u err_flags;
      ProgCtrl_u prog_ctrl;
      OTP_RegZeroPosHigh_u otp_high;
      OTP_RegZeroPosLow_u otp_low;
      DiagAndAutoGainCtrl_u diag_ags;
      Magnitude_u magnitude;
      Angle_u angle;
    };
    AS5048A_Reg_Data_s reg_data_on_startup;

    ErrFlags_u reg_data_err_flags;
    Angle_u reg_data_angle;
    DiagAndAutoGainCtrl_u reg_data_diag_ags;
    Magnitude_u reg_data_magnitude;


    /******* FUNCTIONS: *******/

    /** Deinit SPI handle in the ardunio/HAL driver */
    void deinitSPI(); 
    
    /** Calculate parity value and shift the output so the 
     * parity bit is in the correct location (bit 15 for AS5048) */
    uint16_t inline spiCalcEvenParityAndShift(uint16_t val_to_write);

    bool inline isRxParityBitCorrect(uint16_t rx_data_frame);

    float inline getAngleInRadiansFromRawCounts(uint16_t raw_counts);


    /** Takes raw data from the errFlags and DiagAGS registers and 
     * sets the applicable faults encoded as type AS5048A_Faults_u -> raw.
     * 
     * NOTE: The return type is encoded to be the value of AS5048A_Faults_u -> raw.  To prevent
     * compilation issues/warnings we will use a uint16_t to pass the data. 
     * 
     * Example:  AS5048A_Faults_u_variable.raw = getFaultTypeFromRegData(var1, var2)  
     * 
     * This function is responsible for setting all of the fault bits in the 
     * AS5048A_Faults_s bitfield/struct that are NOT set in the communication functions
     * such as "read", "readAngleAndAdditionalRegister", and "writeRegAndVerify."
     * 
     * Possible errors that can be output from this function are: 
     *   - Parity Error (AS5048A_FLT__PARITY_ERROR) 
     *   - Invalid Command as response from AS5048A (AS5048A_FLT__COMMAND_INVALID) 
     *   - Framing Error (AS5048A_FLT__FRAMING_ERROR)
     *   - Offset compensation unfinished at startup (AS5048A_FLT__OFFSET_COMP_UNFINISHED)
     *   - IC's internal computer/"cordic" overflow (AS5048A_FLT__CORDIC_OVERFLOW)
     *   - Magnetic Field too high (AS5048A_FLT__COMP_ERR_HIGH_MAG_FIELD)
     *   - Magnetic Field too low (AS5048A_FLT__COMP_ERR_LOW_MAG_FIELD)
     * */
    uint16_t inline getFaultTypeFromRegData(uint16_t err_flags_reg_data, uint16_t diag_ags_reg_data);
    

    /** Read one SPI register value.  
     * 
     * NOTE: The return type is encoded to be the value of AS5048A_Faults_u -> raw.  To prevent
     * compilation issues/warnings we will use a uint16_t to pass the data.
     * 
     * Example:  AS5048A_Faults_u_variable.raw = read(var1, &var2)    
     * 
     * Possible errors that can be output from this function are: 
     *   - Parity Error (AS5048A_FLT__PARITY_ERROR)
     *   - Error Frame (AS5048A_FLT__ERROR_FLAG_ACTIVE)
     *   - Invalid Command (AS5048A_FLT__INVALID_SW_FUNC_PARAM) This will be returned 
     *        in the context of this fault only if the command passed as a 
     *        parameter was not a valid command. */
    uint16_t read(AS5048A_Register_e reg_addr_name, uint16_t * data_ptr);

    
    /** Read Angle, Error Flags, Diag/AGS, and Magnitude registers. This function 
     * sends 4 words over SPI.  
     * 
     * NOTE: The @param prev_err_flag_reg is data that was returned from a read 
     * command that was sent the last time the function was called.  If 
     * the last command was a NOP, the function will behave normally.
     * 
     * Each of the 4 words will be doing the following:
     * TX0: Read angle command
     * RX0: clock out error flag data from last transmission's final command
     * 
     * TX1: Read diag_AGS command
     * RX1: clock out angle data
     * 
     * TX2: Read magnitude command
     * RX2: clock out diag_AGS data
     * 
     * TX3: Clear Error flags command (read request)
     * RX3: clock out magnitude data
     * 
     * NOTE: The @return type is encoded to be the value of AS5048A_Faults_u -> raw.  To prevent
     * compilation issues/warnings we will use a uint16_t to pass the data.
     * 
     * Example:  AS5048A_Faults_u_variable.raw = read(var1, &var2)    
     * 
     * Possible errors that can be output from this function are: 
     *   - Parity Error (AS5048A_FLT__PARITY_ERROR)
     *   - Error Frame (AS5048A_FLT__ERROR_FLAG_ACTIVE)
     * The calling function will need to analyze faults in more 
     * detail depending on if the recieved data can be trusted as valid
     *  (ie comm faults would prevent use from knowing what specific faults are set)
     * 
     * NOTE: If any one of the signals has an error during transmission the entire 
     * data set is considered invalid.  This will simplify the code and we likely 
     * don't need any more granularity.  It's a pendulum, not an airplane :)
     * 
     * */
    uint16_t readAngleDiagMagnitudeAndErrors(uint16_t * prev_err_flag_reg,
                                             uint16_t * angle_data_ptr,
                                             uint16_t * diag_ags_data_ptr,
                                             uint16_t * magnitude_data_ptr); 


    /** Write one SPI register value and verify the write results
     * 
     * NOTE: The return type is encoded to be the value of AS5048A_Faults_u -> raw.  To prevent
     * compilation issues/warnings we will use a uint16_t to pass the data.
     * 
     * Example:  AS5048A_Faults_u_variable.raw = read(var1, &var2)    
     * 
     * Possible errors that can be output from this function are: 
     *   - Parity Error (AS5048A_FLT__PARITY_ERROR)
     *   - Error Frame (AS5048A_FLT__ERROR_FLAG_ACTIVE)
     *   - Invalid Command (AS5048A_FLT__INVALID_SW_FUNC_PARAM) This will be returned 
     *        in the context of this fault only if the command passed as a 
     *        parameter was not a valid command. 
     *   - read/write mismatch, i.e. the write didn't take effect (AS5048A_FLT__WRITE_READ_MISMATCH) */
    uint16_t writeRegAndVerify(AS5048A_Register_e reg_addr_name, uint16_t data_to_write);
  
};

#endif
