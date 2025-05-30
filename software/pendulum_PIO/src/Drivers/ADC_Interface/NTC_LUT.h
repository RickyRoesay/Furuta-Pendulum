#include "stdint.h"

#ifndef NTC_LUT_H
#define NTC_LUT_H

/** NTC lookup table for the ODrive/ODESC4.2 motor and aux NTC temp sensors. 
 * 
 * I have not been able to find documention specifically for the ODESC 4.2, 
 * but assuming they copied literally everything from the ODrive, the NTC 
 * part number is probably NXP15XH103F03RC.
 * 
 * The latest ODrive BOM was found here: 
 * https://github.com/odriverobotics/ODriveHardware/blob/master/v3/v3.4docs/BOM_v3.4_48V.csv
 * 
 * More NTC info can be found in the ODESC4p2 pinout excel sheet.  */

#define TABLE_SIZE 23

// Fast lookup function using linear interpolation
float get_NXP15XH103_temp_via_LUT(uint32_t adc_count);

#endif 