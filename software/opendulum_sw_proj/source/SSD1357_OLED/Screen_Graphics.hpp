/* 

A libary to use the SSD1357 driver in conjuction with a particular OLED display that is 64x64

*/

#ifndef OLED_GRAPHICS_H
#define	OLED_GRAPHICS_H

// #include "screen65k.h"				// This is a method of storing arbitrary RGB images in 16-bit depth where two colors are 5 bits and the last is 6 bits
#include "SparkFun_SSD1357_OLED.h"	// This is a driver that takes screens and displays them on a physical device



#define OLED_SCROLL_NORMAL 0x01
#define OLED_SCROLL_SLOW 0x02
#define OLED_SCROLL_SLOWEST 0x03

class OLED_Graphics : public SSD1357 {
private:
protected:

	bool _isInverted, _isFlippedH, _isFlippedV, _incV, _coSwapped, _scanReversed;

	uint8_t _colorMode;


public:

	OLED_Graphics();

	void begin(uint8_t dcPin, uint8_t rstPin, uint8_t csPin, SPIClass &spiInterface = SPI, uint32_t spiFreq = SSD1357_SPI_MAX_FREQ) override;
	void defaultConfigure( void );

    void clearDisplay(uint8_t mode = 0x00);
    void fillDisplay(uint16_t value);

    void invert(bool inv);
    void flipHorizontal(bool flip);

    


    uint16_t getDisplayWidth(void);
    uint16_t getDisplayHeight(void);
    void setDisplayHeight(uint16_t height);
    void setFillColor(uint16_t value);
    

    // LCD Rotate Scroll functions
    void scrollRight(uint8_t start, uint8_t stop, uint8_t speed);
    void scrollLeft(uint8_t start, uint8_t stop, uint8_t speed);
    //TODO Add 0x29/0x2A vertical scrolling commands
    //void scrollUp(uint8_t start, uint8_t stop);
    //void scrollVertLeft(uint8_t start, uint8_t stop);
    void scrollStop(void);




};








#endif /* SF_RGB_OLED_64X64_H */