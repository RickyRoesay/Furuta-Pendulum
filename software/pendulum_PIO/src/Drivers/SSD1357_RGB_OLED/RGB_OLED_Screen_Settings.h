#ifndef RGB_OLED_SCREEN_SETTINGS_H
#define	RGB_OLED_SCREEN_SETTINGS_H





/**
 * Keeping consistent with the datasheet's terminology, a "Row" is 
 * the line of pixels all connected to a single "common" driver. 
 * 
 * A "Column" is a line of pixels connected to a set of 3x segments, one for each 
 * color (RGB).  
 * 
 * common index scales with row number
 * segment pairs (in 3, RGB)  
 * 
 * 
 * 
 * For coordinates aligning with the screens intended orientation, width = x, and 
 * height = y will be used. */
//


//#define WIDTH_IS_PROPORTIONAL_TO_COLUMNS // for example, for a width of 128, there would be 128 columns.

#define OLED_64x64_WIDTH 	64
#define OLED_64x64_HEIGHT 	128 

#define OLED_64x64_START_ROW	0
#define OLED_64x64_STOP_ROW 	127 

#define OLED_64x64_START_COL	0x20
#define OLED_64x64_STOP_COL		0x5F




#endif /* SF_RGB_OLED_64X64_H */