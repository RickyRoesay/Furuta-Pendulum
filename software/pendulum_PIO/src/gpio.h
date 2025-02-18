#ifndef __GPIO_H
#define __GPIO_H

#define	GPIO1	        PA0 // use this as uart4 tx
#define	GPIO2	        PA1 // use this as uart4 rx
#define	GPIO3_UART_TX	PA2
#define	GPIO4_UART_RX	PA3

#define	SPI_MOSI      PC12
#define	SPI_MISO	    PC11
#define	SPI_SCK	      PC10
#define	SPI_nCS_IO6	  PB2
#define	SPI_nCS_DRV	  PC13  // not confirmed

#define	CAN_TX	      PB9   // not confirmed
#define	CAN_RX	      PB8   // not confirmed

#define	ENC_A	        PB4
#define	ENC_B	        PB5

/** this won't be used as ENC_Z, but on the board's silkscreen it's labeled 
 * as "Z" on the encoder connector.  The name of this macro will 
 * reflect the board's label as well as it's indended use. */
#define	ENC_Z__GPIO5	PC9  

#define	USB_DM	      PA11  // not confirmed
#define	USB_DP	      PA12  // not confirmed

#define	nFAULT	      PD2   // not confirmed
#define	EN_GATE	      PB12
#define	AH	          PA8
#define	AL	          PB13
#define	BH	          PA9
#define	BL	          PB14
#define	CH	          PA10
#define	CL	          PB15

#define	AUX_H	        PB11
#define	AUX_L	        PB10

/** Analog Inputs, not confirmed: */
#define	SO1	          PC0
#define	SO2	          PC1

/** the "_ALT1" is for ADC2, and additionally "_ALT2" is for ADC3 */
#define	M_TEMP	      PC5_ALT1
#define	AUX_TEMP	    PA5_ALT1
#define	VBUS_SNS	    PA6_ALT1

/** this is an unused pin that corresponds to TIM3 when used as a 
 * timer pin via the HardwareTimer class.  We only need this to 
 * act as an interrupt generation source so we chose a pin that
 * is not connected to anything on the ODESC4.2 
 * 
 * NOTE: It is assumed that all "Motor1" (second instance of motors)
 * in the odrive 3.5 schematic have pins that are not connected to anything
 * on the ODESC4.2 */
#define FLOATING_M1_CH_TIM3  PC8


#endif // __BIQUAD_HPP
