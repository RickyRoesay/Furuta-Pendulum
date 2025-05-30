#ifndef RGB_OLED_SCREEN_SETTINGS_H
#define	RGB_OLED_SCREEN_SETTINGS_H





/**
 * Keeping consistent with the datasheet's terminology, a "Row" is 
 * the line of pixels all connected to a single "common" driver. 
 * 
 * A "Column" is a line of pixels connected to a set of 3x segments, one for each 
 * color (RGB).  
 * 
 * common index scales with row number.
 * 
 * segment pairs (in 3, RGB) scale with columns. 
 * 
 * 
 * 
 * For coordinates aligning with the screens intended orientation, width = x, and 
 * height = y will be used. */
//


//#define WIDTH_IS_PROPORTIONAL_TO_COLUMNS // for example, for a width of 128, there would be 128 columns.


#define OLED_START_ROW_IDX	0
#define OLED_STOP_ROW_IDX 	0x7F 

/** because the only remapping that can be done 
 * on the columns is inverting them, (0-127  vs 127-0),
 * the screen's populated segments are center aligned in memory 
 * to remap correctly when there are <128 segment pairs */
#define OLED_START_COL_IDX	0x20
#define OLED_STOP_COL_IDX	0x5F


/** Second bit of payload remap/color depth command */
#define OLED_COLUMN_ADDR_NONINVERTED    false
#define OLED_COLUMN_ADDR_INVERTED       true

#define OLED_ROW_COM_SCAN_NONINVERTED    false
#define OLED_ROW_COM_SCAN_INVERTED       true


#define OLED_HORIZONTAL_ADDR_INCREMENT  false
#define OLED_VERTICAL_ADDR_INCREMENT    true

#ifdef WIDTH_IS_PROPORTIONAL_TO_COLUMNS
    #define OLED_START_X_IDX    OLED_START_COL_IDX
    #define OLED_STOP_X_IDX     OLED_STOP_COL_IDX

    #define OLED_START_Y_IDX    OLED_START_ROW_IDX
    #define OLED_STOP_Y_IDX     OLED_STOP_ROW_IDX


    #define OLED_ADDRESS_INCREMENT_ORDER    OLED_HORIZONTAL_ADDR_INCREMENT
    #define OLED_COM_SCAN_INV_SETTING       OLED_ROW_COM_SCAN_INVERTED
#else
    #define OLED_START_X_IDX    OLED_START_ROW_IDX
    #define OLED_STOP_X_IDX     OLED_STOP_ROW_IDX

    #define OLED_START_Y_IDX    OLED_START_COL_IDX 
    #define OLED_STOP_Y_IDX     OLED_STOP_COL_IDX 

    #define OLED_ADDRESS_INCREMENT_ORDER    OLED_VERTICAL_ADDR_INCREMENT
    #define OLED_COM_SCAN_INV_SETTING       OLED_ROW_COM_SCAN_NONINVERTED
#endif

#define OLED_X_HEIGHT_m1    (OLED_STOP_X_IDX - OLED_START_X_IDX)
#define OLED_X_HEIGHT       (OLED_X_HEIGHT_m1 + 1)

#define OLED_Y_HEIGHT_m1    (OLED_STOP_Y_IDX - OLED_START_Y_IDX)
#define OLED_Y_HEIGHT       (OLED_Y_HEIGHT_m1 + 1)

#define OLED_MUX_NUM    (OLED_STOP_ROW_IDX)

#define OLED_WIDTH 	    OLED_X_HEIGHT
#define OLED_HEIGHT 	OLED_Y_HEIGHT 






/** 
 * 
 * ring buffer example:
 * uint8[]  raw data buffer
 * uint16   raw data buffer head idx
 * uint16   raw data buffer tail idx
 * 
 * 
 * ring_buffer[]     num of bytes to send while D# is high or low
 * ring_buffer[]     D# high or low flag corresponding to same index as the above ring buffer
 * 
 * ring_buffer[]     num of bytes to send before deasserting chip select
 * 
 */




#endif /* RGB_OLED_SCREEN_SETTINGS_H */