#ifndef __GPIO_H
#define __GPIO_H

#define	GPIO1	        PA0 
#define	GPIO2	        PA1
#define	GPIO3_UART_TX	PA2 // use this as uart4 tx
#define	GPIO4_UART_RX	PA3 // use this as uart4 rx

#define	SPI_MOSI      PC12
#define	SPI_MISO	    PC11
#define	SPI_SCK	      PC10
#define	SPI_nCS_IO6	  PB2
#define	SPI_nCS_DRV	  PC13 

#define	CAN_TX	      PB9 
#define	CAN_RX	      PB8 

#define	ENC_A	        PB4
#define	ENC_B	        PB5
#define	ENC_Z	        PC9  

#define	USB_DM	      PA11  // yellow wire
#define	USB_DP__LED_DO	      PA12  // green wire

#define	nFAULT	      PD2   
#define	EN_GATE	      PB12
#define	AH	          PA8
#define	AL	          PB13
#define	BH	          PA9
#define	BL	          PB14
#define	CH	          PA10
#define	CL	          PB15

#define	AUX_H	        PB11
#define	AUX_L	        PB10

/** Analog Inputs: */
#define	SO1	          PC0
#define	SO2	          PC1

/** the "_ALT1" is for ADC2, and additionally "_ALT2" is for ADC3 */
#define	M_TEMP	      PC5_ALT1
#define	AUX_TEMP	    PA5_ALT1
#define	VBUS_SNS	   PA6  // PA6_ALT1



#endif // __GPIO_H
