


#include "Arduino.h"
#include <SPI.h>

#include "AS5048A_MagSenseSPI.hpp"
#include "AS5048A_Registers.hpp"

#define SPI_CLOCK_SPEED_HZ_MAG 8000000
#define SPI_MODE__CPOL_0__CPHA_1  SPI_MODE1

/** NOTE: Register Descriptions can be found on figure 22 of the datasheet */
/** NOTE: Struct/union declarations can be found in AS5048_Registers.hpp */

#define PARITY_BIT_MASK       0x8000  // parity bit is always the MSB for read and writes
#define ERROR_FLAG_BIT_MASK   0x4000  // error flag bit is bit 14 on messages recieved from AS5048
#define READ_nWRITE_BIT_MASK  0x4000  // READ/nWRITE bit is bit 14 on messages sent to the AS5048
#define DATA_MASK             0x3FFF  // the last 14 bits are for data

#define NOP_COMMAND_DATA                      0x0000  // an NOP command has all 0's for the full word
#define READ_ANGLE_COMMAND_DATA               0xFFFF // with read an parity bit, its all 1's for reading angle command
#define READ_DIAG_AGS_COMMAND_DATA            0x7FFD 
#define READ_MAGNITUDE_COMMAND_DATA           0x7FFE 
#define CLEAR_READ_ERR_FLAGS_COMMAND_DATA     0x4001 


/** Used to check if the diag_ags register has COF and comp error bits LOW */
#define DIAG_AGS_COF_AND_COMP_ERR_ANTI_BIT_MASK   0x0E00

/** Used to check if the diag_ags register hasn't finished offset compensation (OFC flag)*/
#define DIAG_AGS_OFC_BIT_MASK   0x0100


#define DIAG_AGS_WARNINGS_BIT_MASK 0x0F00
#define DIAG_AGS_REG_ALIGN_TO_FAULT_R_SHIFT_NUM   5
#define ERROR_FLAGS_REG_BIT_MASK 0x0007



#define MAG_SNS_PARITY_BIT_IDX 15
#define MAG_SNS_nW_R_EF_BIT_IDX 14



#define MAG_SNS_AS5048A_BIT_RES 14
#define COUNTS_PER_REVOLUTION_INT  16384  // 14 bits of resolution, 2^14 = 16384
#define COUNTS_PER_REVOLUTION_F32  16384.0f  // 14 bits of resolution, 2^14 = 16384



#define WRITE_CMD_FRAME_IDX_0_TX         0 //TX
#define PREV_CMD_RESP_FRAME_IDX_0_RX     0 //RX
#define DATA_FRAME_IDX_1_TX              1 //TX
#define PREV_DATA_IN_REG_FRAME_IDX_1_RX  1 //RX
#define NOP_FRAME_IDX_2_TX               2 //TX
#define NEW_DATA_IN_REG_FRAME_IDX_2_RX   2 //RX



/** For the AS5048A, a minimum time of 350ns is needed between 
 * the rising and falling edge of the chip select pin between each 
 * 16 bit data frame.  To speed up consecutive transmissions, we will
 * burn a select number of clock cycles instead of using the STM32duino's 
 * delayMicroseconds function. 
 * 
 * As per figure 15 in the AS5048's datasheet, the minimum wait time is 
 * 350 nanoseconds. We are safe to wait the minimum time of 350ns since 
 * the digitalWrite function of the chip select pin takes roughly 200ns 
 * which should add an appropriate amount of margin.  
 * 
 * For a clock speed of 168MHz, the wait time in cycles is calculated 
 * as follows: 
 * wait_time_in_cycles = 350ns / SYSCLK_periodicity = 350e-9 * SYSCLK
 * wait_time_in_cycles = 58.8 = 59
 * 
 * With the current implmenetation of consecutive frames (1 word = frame)
 * the on time of the nCS pin is around 750ns */
#define NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS 59




/** this function has been adapted from STM32duino's delayMicroseconds
 * function to wait for a specific number of clock cycles as opposed to
 * a unit of microseconds.  */
static inline void stm32_delay_clock_cycles(int32_t cycles)
{
  int32_t start  = dwt_getCycles();
  while ((int32_t)dwt_getCycles() - start < cycles);
}



AS5048A_MagSenseSPI::AS5048A_MagSenseSPI(int cs)
{
  chip_select_pin = cs;
}



float inline AS5048A_MagSenseSPI::getAngleInRadiansFromRawCounts(uint16_t raw_counts)
{
  return (float)(raw_counts & DATA_MASK) / COUNTS_PER_REVOLUTION_F32 * _2PI;
}



uint16_t inline AS5048A_MagSenseSPI::getFaultTypeFromRegData(uint16_t err_flags_reg_data, 
                                                            uint16_t diag_ags_reg_data)
{
  AS5048A_Faults_u tmp_fault_rtn_val;
  
  /** These are doing the same thing, but the bottom section of this
   * "#if" code should be faster unless the optimized compiler is alarmingly good.
   * I haven't looked at the compiled assembly of the both implementations
   * so I don't know for sure. */
  #if 0
  ErrFlags_u tmp_err_flags;
  tmp_err_flags.raw = err_flags_reg_data;
  
  DiagAndAutoGainCtrl_u tmp_diag_ags;
  tmp_diag_ags.raw = diag_ags_reg_data;

  tmp_fault_rtn_val.bit.command_invalid_errFlag = tmp_err_flags.bit.command_inv;
  tmp_fault_rtn_val.bit.parity_error = tmp_err_flags.bit.ErrFlags_PAR;
  tmp_fault_rtn_val.bit.framing_error = tmp_err_flags.bit.framing_error;
  
  tmp_fault_rtn_val.bit.comp_err_high_mag_field = tmp_diag_ags.bit.comp_low;
  tmp_fault_rtn_val.bit.comp_err_low_mag_field = tmp_diag_ags.bit.comp_high;
  tmp_fault_rtn_val.bit.cordic_overflow = tmp_diag_ags.bit.cordis_ovf_COF;
  tmp_fault_rtn_val.bit.offset_comp_unfinished = !tmp_diag_ags.bit.offset_comp_finished_OCF;
  #else 
  tmp_fault_rtn_val.raw = (diag_ags_reg_data & ERROR_FLAGS_REG_BIT_MASK);
  tmp_fault_rtn_val.raw |= (err_flags_reg_data & DIAG_AGS_WARNINGS_BIT_MASK) >> DIAG_AGS_REG_ALIGN_TO_FAULT_R_SHIFT_NUM;
  #endif 

  return tmp_fault_rtn_val.raw;
}



bool AS5048A_MagSenseSPI::init(SPIClass* _spi)
{
  bool tmp_rtn_val;

  spi = _spi;

	settings = SPISettings(SPI_CLOCK_SPEED_HZ_MAG, MSBFIRST, SPI_MODE__CPOL_0__CPHA_1);

	//setup pins
	pinMode(chip_select_pin, OUTPUT);
	digitalWrite(chip_select_pin, HIGH);

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();

  fault_status_on_startup.all = AS5048A_FLT__NO_ERRORS;

  /** wait 10ms since the sensor seems to output incorrect 
   * angle measurements right after it starts up after a power up, even through there 
   * are no faults being reported and the offset calibration finished flag 
   * is set HIGH when reading registers immediately after startup. */
  delayMicroseconds(10000);

  /** simply clear the faults in case the AS5048A is faulted 
   * on startup.  If this does not clear the fault successfully 
   * then the fault_status variable will be set accordingly to the output 
   * of the succeeding "read" function calls. */
  (void)read(AS5048_REG__ErrFlags, &reg_data_on_startup.err_flags.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

  fault_status_on_startup.raw |= read(AS5048_REG__ProgCtrl, &reg_data_on_startup.prog_ctrl.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

  fault_status_on_startup.raw |= read(AS5048_REG__OTP_RegZeroPosHigh, &reg_data_on_startup.otp_high.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

  fault_status_on_startup.raw |= read(AS5048_REG__OTP_RegZeroPosLow, &reg_data_on_startup.otp_low.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

  fault_status_on_startup.raw |= read(AS5048_REG__DiagAndAutoGainCtrl, &reg_data_on_startup.diag_ags.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

  fault_status_on_startup.raw |= read(AS5048_REG__Magnitude, &reg_data_on_startup.magnitude.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);
  
  fault_status_on_startup.raw |= read(AS5048_REG__Angle, &reg_data_on_startup.angle.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

  #if 0
  vel_angle_prev = getAngleInRadiansFromRawCounts(reg_data_on_startup.angle.raw); 
  vel_angle_prev_ts = _micros();
  delay(1);
  fault_status.raw |= read(AS5048_REG__Angle, &reg_data_angle.raw);
  stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);
  angle_prev = getAngleInRadiansFromRawCounts(reg_data_angle.raw); 
  angle_prev_ts = _micros();
  #else
  this->Sensor::init(); // call base class init
  #endif 

  /** the init function will call getSensorAngle which sets 
   * fault_status.  Store the most recent fault_status in the fault_status_on_startup
   * variable to keep a list of all fault bits that were set at startup. 
   * Since Sensor::init calls getSensorAngle twice, we are only looking 
   * at the most recent of the 2 times fault_status is set by getSensorAngle */
  fault_status_on_startup.raw |= fault_status.raw;
  
  /** If there are diagnostic warnings or and error flag is set, this function
   * will return a raw uint16 encoded as AS5048A_Faults_u so we have more granularity 
   * of a specific issue.  */
  if(fault_status_on_startup.raw == AS5048A_FLT__NO_ERRORS)
  {
    /** only apply the diag_ags values and the error flags specific faults
     * if there has been no comm errors.  Otherwise we may be setting 
     * flags high when in reality it was simply that the SPI transmission was cooked. */
    fault_status_on_startup.raw |= getFaultTypeFromRegData(reg_data_on_startup.err_flags.raw, 
                                                          reg_data_on_startup.diag_ags.raw);
  }

  fault_memory.raw = fault_status_on_startup.raw;

  if(fault_status_on_startup.raw == AS5048A_FLT__NO_ERRORS)
    tmp_rtn_val = true;
  else
    tmp_rtn_val = false;

  return tmp_rtn_val;
}





//  Shaft angle calculation
//  updates fault_status
//  angle is in radians [rad]
float AS5048A_MagSenseSPI::getSensorAngle()
{
  /** The base class's (sensor) "update" function that is called on each 
   * execution of a control loop uses a retun value of less than 0
   * as a way to signal that an error has happened during transmission.  Start
   * out by initializing the return value to an invalid value. */
  float tmp_angle_rtn_val = -1.0f;

  
  /** If any of the 4 messages have an error flag or parity error, all of the data 
   * is "thrown out" even if there was only a parity error on the final message
   * recieved. 
   * NOTE: See function prototype for more information on this function */
  fault_status.raw = readAngleDiagMagnitudeAndErrors(&reg_data_err_flags.raw,// requested at last func call
                                                    &reg_data_angle.raw,
                                                    &reg_data_diag_ags.raw,
                                                    &reg_data_magnitude.raw); 

  /** the only faults checked for so far are for error flags and parity bits: */
  if(fault_status.all == AS5048A_FLT__NO_ERRORS)
  {
    /** statement condition is true if there are any diagnostic warnings */
    if((reg_data_diag_ags.raw & DIAG_AGS_COF_AND_COMP_ERR_ANTI_BIT_MASK)
    || !(reg_data_diag_ags.raw & DIAG_AGS_OFC_BIT_MASK))
    {
      sensor_state = AS5048A_MagSense_State__Mag_Fault;
      num_of_mag_errors++;
      
      // keep the return value the same, -1 = invalid.
    }
    else
    {
      sensor_state = AS5048A_MagSense_State__OK;
      tmp_angle_rtn_val = getAngleInRadiansFromRawCounts(reg_data_angle.bit.angle_data);
    }
      
    fault_status.raw |= getFaultTypeFromRegData(reg_data_err_flags.raw, reg_data_diag_ags.raw);
  }
  else
  {
    num_of_err_flags++;
    sensor_state = AS5048A_MagSense_State__Comm_Fault;

    /** don't update fault_status with specific errFlags or Diag warnings since comm failed
     * and we don't know if any of the data is valid. */

    // keep the return value the same, -1 = invalid.
  }

  return tmp_angle_rtn_val;
}



// SPI functions 
/**
 * Utility function used to calculate even parity of word
 */
uint16_t inline AS5048A_MagSenseSPI::spiCalcEvenParityAndShift(uint16_t value)
{
	uint16_t cnt = 0;
	uint16_t i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1) cnt++;
		value >>= 1;
	}
	return (cnt & 0x1) << MAG_SNS_PARITY_BIT_IDX;
}


bool inline AS5048A_MagSenseSPI::isRxParityBitCorrect(uint16_t rx_data_frame)
{
  bool tmp_rtn_val;

  /** remove the parity bit before calculating the data's parity */
  uint16_t tmp_data_to_check_parity_on = ~PARITY_BIT_MASK & rx_data_frame;
  uint16_t tmp_shifted_parity_bit_expected = spiCalcEvenParityAndShift(tmp_data_to_check_parity_on);
  
  /** set the parity error if the parity bit is not what we expect it to be: */
  if((rx_data_frame & PARITY_BIT_MASK) == tmp_shifted_parity_bit_expected)
    tmp_rtn_val = true;
  else
    tmp_rtn_val = false;

  return tmp_rtn_val;
}


/** NOTE: The return type is encoded to be the value of AS5048A_Faults_u -> raw.  To prevent
 * compilation issues/warnings we will use a uint16_t to pass the data.
 * 
 * Example:  AS5048A_Faults_u_variable.raw = read(var1, &var2)    */
uint16_t   AS5048A_MagSenseSPI::read(   AS5048A_MagSenseSPI::AS5048A_Register_e reg_addr_name, 
                                        uint16_t * data_ptr)
{
  uint16_t tmp_tx_data;
  AS5048A_Faults_u tmp_rtn_flt;
  tmp_rtn_flt.all = AS5048A_FLT__INVALID_SW_FUNC_PARAM;

  switch(reg_addr_name)
  {
    case AS5048_REG__ErrFlags:
    case AS5048_REG__ProgCtrl:
    case AS5048_REG__OTP_RegZeroPosHigh:
    case AS5048_REG__OTP_RegZeroPosLow:
    case AS5048_REG__DiagAndAutoGainCtrl:
    case AS5048_REG__Magnitude:
    case AS5048_REG__Angle:
      if(data_ptr)
      {
        GenericRegister_u tmp_recieved_data;

        /** clear the "invalid sw function parameter" fault 
         * from the return variable since valid parameters were used */
        tmp_rtn_flt.all = AS5048A_FLT__NO_ERRORS;

        // set the READ/nWRITE bit to 1 for read commands
        tmp_tx_data = reg_addr_name | READ_nWRITE_BIT_MASK;
        
        //Add a parity bit on the the MSB
        tmp_tx_data |= spiCalcEvenParityAndShift(tmp_tx_data);

        //SPI - begin transaction
        spi->beginTransaction(settings);

        //Send the register address we want to read + parity + Read/Write bit
        digitalWrite(chip_select_pin, LOW);
        (void)spi->transfer16(tmp_tx_data);
        digitalWrite(chip_select_pin, HIGH);
        
        /** Wait >=350ns for the command to take effect.  
         * See "NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS" macro declaration for more info */
        stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

        //Now read the response
        digitalWrite(chip_select_pin, LOW);
        tmp_recieved_data.raw = spi->transfer16(NOP_COMMAND_DATA); // send all 0's as a NOP command
        digitalWrite(chip_select_pin, HIGH);

        spi->endTransaction();

        /** set fault status as Error flag if the recieved data has the error flag bit HIGH
         * 
         * NOTE: we are only checking the most recent byte.  If an error has occurred anywhere 
         * within this data transmission, we will see in in the error flag until we read 
         * from the "Error Flag" register.  This will also allow us to clear the error 
         * flag using this read function since we'll be ignoring the error flag 
         * sent before the "Error flag" register is cleared. */
        tmp_rtn_flt.bit.error_flag_active = tmp_recieved_data.bit.Generic_RWn_or_Error_Flag;

        /** set the parity error if the recieved parity bit is not correct */
        tmp_rtn_flt.bit.parity_error = !isRxParityBitCorrect(tmp_recieved_data.raw);

        *data_ptr = tmp_recieved_data.raw;
      }
      else
      {
        /** Do nothing.  After this switch case we will simply return 
         * the fault status variable tmp_rtn_flt that has already been 
         * set to invalid SW function parameter */
      }
    break;

    /** NOP and any other value are not valid read addresses */
    case AS5048_REG__NOP:
    default:
      /** Do nothing.  After this switch case we will simply return 
       * the fault status variable tmp_rtn_flt that has already been 
       * set to invalid SW function parameter */
    break;
  }

	return tmp_rtn_flt.raw;  // return the fault status as a union between an enum, bitmask'd stuct and uint16_t
}




/** NOTE: see function prototype for more info. */
uint16_t AS5048A_MagSenseSPI::readAngleDiagMagnitudeAndErrors(uint16_t * prev_err_flag_reg,
                                          uint16_t * angle_data_ptr,
                                          uint16_t * diag_ags_data_ptr,
                                          uint16_t * magnitude_data_ptr)
{
  uint16_t tmp_second_tx_word;
  AS5048A_Faults_u tmp_rtn_flt;
  tmp_rtn_flt.all = AS5048A_FLT__INVALID_SW_FUNC_PARAM;

  if(prev_err_flag_reg && angle_data_ptr && diag_ags_data_ptr && magnitude_data_ptr)
  {
    GenericRegister_u tmp_recieved_data[4];

    /** clear the "invalid sw function parameter" fault 
     * from the return variable since valid parameters were used */
    tmp_rtn_flt.all = AS5048A_FLT__NO_ERRORS;

    //SPI - begin transaction
    spi->beginTransaction(settings);

    //Send the register address we want to read + parity + Read/Write bit
    digitalWrite(chip_select_pin, LOW);
    tmp_recieved_data[0].raw = spi->transfer16(READ_ANGLE_COMMAND_DATA);
    digitalWrite(chip_select_pin, HIGH);
    
    /** Wait >=350ns for the command to take effect.  
     * See "NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS" macro declaration for more info */
    stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

    //Now read the response
    digitalWrite(chip_select_pin, LOW);
    tmp_recieved_data[1].raw = spi->transfer16(READ_DIAG_AGS_COMMAND_DATA); // send all 0's as a NOP command
    digitalWrite(chip_select_pin, HIGH);
    
    /** Wait >=350ns for the command to take effect.  
     * See "NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS" macro declaration for more info */
    stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

    //Now read the response
    digitalWrite(chip_select_pin, LOW);
    tmp_recieved_data[2].raw = spi->transfer16(READ_MAGNITUDE_COMMAND_DATA); // send all 0's as a NOP command
    digitalWrite(chip_select_pin, HIGH);
    
    /** Wait >=350ns for the command to take effect.  
     * See "NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS" macro declaration for more info */
    stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);

    //Now read the response
    digitalWrite(chip_select_pin, LOW);
    tmp_recieved_data[3].raw = spi->transfer16(CLEAR_READ_ERR_FLAGS_COMMAND_DATA); // send all 0's as a NOP command
    digitalWrite(chip_select_pin, HIGH);

    spi->endTransaction();

    /** set fault status as Error flag if the recieved data has the error flag bit HIGH
     * 
     * NOTE: we are only checking the most recent byte.  If an error has occurred anywhere 
     * within this data transmission, we will see in in the error flag until AFTER read 
     * from the "Error Flag" register.  This will also allow us to clear the error 
     * flag using this read function since we'll be ignoring the error flag 
     * sent before the "Error flag" register is cleared. */
    tmp_rtn_flt.bit.error_flag_active = tmp_recieved_data[3].bit.Generic_RWn_or_Error_Flag;

    /** set the parity error if the recieved parity bit is not correct on both recieved words. */
    tmp_rtn_flt.bit.parity_error = !isRxParityBitCorrect(tmp_recieved_data[0].raw);
    tmp_rtn_flt.bit.parity_error |= !isRxParityBitCorrect(tmp_recieved_data[1].raw);
    tmp_rtn_flt.bit.parity_error |= !isRxParityBitCorrect(tmp_recieved_data[2].raw);
    tmp_rtn_flt.bit.parity_error |= !isRxParityBitCorrect(tmp_recieved_data[3].raw);

    *prev_err_flag_reg = tmp_recieved_data[0].raw;
    *angle_data_ptr = tmp_recieved_data[1].raw;
    *diag_ags_data_ptr = tmp_recieved_data[2].raw;
    *magnitude_data_ptr = tmp_recieved_data[3].raw;
  }
  else
  {
    /** Do nothing.  After this switch case we will simply return 
     * the fault status variable tmp_rtn_flt that has already been 
     * set to invalid SW function parameter */
  }

	return tmp_rtn_flt.raw;  // return the fault status as a union between an enum, bitmask'd stuct and uint16_t
}



/** NOTE: The return type is encoded to be the value of AS5048A_Faults_u -> raw.  To prevent
 * compilation issues/warnings we will use a uint16_t to pass the data.
 * 
 * Example:  AS5048A_Faults_u_variable.raw = writeRegAndVerify(var1, var2)    */
uint16_t   AS5048A_MagSenseSPI::writeRegAndVerify(AS5048A_MagSenseSPI::AS5048A_Register_e reg_addr_name, 
                                                                  uint16_t data_to_write)
{
  /** NOTE: See figure 22 in the datasheet for more information on the write command */

  AS5048A_Faults_u tmp_rtn_flt;
  tmp_rtn_flt.all = AS5048A_FLT__INVALID_SW_FUNC_PARAM;

  uint16_t tmp_tx_data[3];
  GenericRegister_u tmp_rx_data[3];

  bool tmp_does_new_reg_val_match_tx_data;

  switch(reg_addr_name)
  {
    case AS5048_REG__ErrFlags:
    case AS5048_REG__ProgCtrl:
    case AS5048_REG__OTP_RegZeroPosHigh:
    case AS5048_REG__OTP_RegZeroPosLow:
    case AS5048_REG__DiagAndAutoGainCtrl:
    case AS5048_REG__Magnitude:
    case AS5048_REG__Angle:

      /** clear the "invalid sw function parameter" fault 
       * from the return variable since valid parameters were used */
      tmp_rtn_flt.all = AS5048A_FLT__NO_ERRORS;

      /** build the write command, set the address and add parity bit to the write command */
      tmp_tx_data[WRITE_CMD_FRAME_IDX_0_TX] = reg_addr_name;
      tmp_tx_data[WRITE_CMD_FRAME_IDX_0_TX] |= spiCalcEvenParityAndShift(tmp_tx_data[WRITE_CMD_FRAME_IDX_0_TX]);
      
      /** ensure READ/nWRITE bit is cleared and write the correct parity bit */
      tmp_tx_data[DATA_FRAME_IDX_1_TX] = data_to_write & DATA_MASK;
      tmp_tx_data[DATA_FRAME_IDX_1_TX] |= spiCalcEvenParityAndShift(tmp_tx_data[WRITE_CMD_FRAME_IDX_0_TX]);

      tmp_tx_data[NOP_FRAME_IDX_2_TX] = NOP_COMMAND_DATA;

      //SPI - begin transaction
      spi->beginTransaction(settings);

      //Send the register address we want to read + parity + Read/Write bit
      digitalWrite(chip_select_pin, LOW);
      tmp_rx_data[PREV_CMD_RESP_FRAME_IDX_0_RX].raw = spi->transfer16(tmp_tx_data[WRITE_CMD_FRAME_IDX_0_TX]);
      digitalWrite(chip_select_pin, HIGH);
        
      /** delay 1us, the minimum time possible in plain arduino. 
       * 350ns is the required time for AMS sensors, 80ns for MA730, MA702 */
      #if 0
      delayMicroseconds(1); 
      #else 
      stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);
      #endif 

      digitalWrite(chip_select_pin, LOW);
      tmp_rx_data[PREV_DATA_IN_REG_FRAME_IDX_1_RX].raw = spi->transfer16(tmp_tx_data[DATA_FRAME_IDX_1_TX]); // send all 0's as a NOP command
      digitalWrite(chip_select_pin, HIGH);
        
      /** delay 1us, the minimum time possible in plain arduino. 
       * 350ns is the required time for AMS sensors, 80ns for MA730, MA702 */
      #if 0
      delayMicroseconds(1); 
      #else 
      stm32_delay_clock_cycles(NUM_OF_CLK_CYCLES_TO_WAIT_FOR_350NS);
      #endif 

      digitalWrite(chip_select_pin, LOW);
      tmp_rx_data[NEW_DATA_IN_REG_FRAME_IDX_2_RX].raw = spi->transfer16(tmp_tx_data[NOP_FRAME_IDX_2_TX]); // send all 0's as a NOP command
      digitalWrite(chip_select_pin, HIGH);

      spi->endTransaction();

      /** set fault status as Error flag if the recieved data has the error flag bit HIGH
       * 
       * NOTE: we are only checking the most recent byte.  If an error has occurred anywhere 
       * within this data transmission, we will see in in the error flag until we clear 
       * the "Error Flag" register by reading its value. */
      tmp_rtn_flt.bit.error_flag_active = tmp_rx_data[NEW_DATA_IN_REG_FRAME_IDX_2_RX].bit.Generic_RWn_or_Error_Flag;

      /** set the parity error if the recieved parity bit is not correct */
      tmp_rtn_flt.bit.parity_error =  !isRxParityBitCorrect(tmp_rx_data[PREV_CMD_RESP_FRAME_IDX_0_RX].raw);
      tmp_rtn_flt.bit.parity_error |= !isRxParityBitCorrect(tmp_rx_data[PREV_DATA_IN_REG_FRAME_IDX_1_RX].raw);
      tmp_rtn_flt.bit.parity_error |= !isRxParityBitCorrect(tmp_rx_data[NEW_DATA_IN_REG_FRAME_IDX_2_RX].raw);
      
      tmp_does_new_reg_val_match_tx_data = (tmp_rx_data[NEW_DATA_IN_REG_FRAME_IDX_2_RX].bit.generic_data 
                                                == (data_to_write & DATA_MASK));
      tmp_rtn_flt.bit.write_value_mismatch = !tmp_does_new_reg_val_match_tx_data;
    break;

    /** NOP and any other value are not valid read addresses */
    case AS5048_REG__NOP:
    default:
      /** Do nothing.  After this switch case we will simply return 
       * the fault status variable tmp_rtn_flt that has already been 
       * set to invalid SW function parameter */
    break;
  }

	return tmp_rtn_flt.raw;  // return the fault status as a union between an enum, bitmask'd stuct and uint16_t
}


/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time */
void AS5048A_MagSenseSPI::deinitSPI()
{
	spi->end();
}


