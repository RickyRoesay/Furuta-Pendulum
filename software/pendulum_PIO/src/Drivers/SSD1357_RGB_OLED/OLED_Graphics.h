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

	// LCD Draw functions
    void clearDisplay(uint8_t mode = 0x00);
    void fillDisplay(uint16_t value);
    // void setCursor(uint8_t x, uint8_t y);

    void invert(bool inv);
    // void setContrast(uint8_t contrast);
    // void flipVertical(bool flip);
    void flipHorizontal(bool flip);

    

    // void drawChar(uint8_t x, uint8_t y, uint8_t c);
    // void drawChar(uint8_t x, uint8_t y, uint8_t c, uint8_t color, uint8_t mode);

    // void drawBitmap(uint8_t *bitArray);

    uint16_t getDisplayWidth(void);
    uint16_t getDisplayHeight(void);
    void setDisplayWidth(uint16_t width);
    void setDisplayHeight(uint16_t height);
    void setFillColor(uint16_t value);
    // void setDrawMode(uint8_t mode);
    // uint8_t *getScreenBuffer(void);

    //Font functions
    // uint8_t getFontWidth(void);
    // uint8_t getFontHeight(void);
    // uint8_t getTotalFonts(void);
    // uint8_t getFontType(void);
    // bool setFontType(uint8_t type);
    // uint8_t getFontStartChar(void);
    // uint8_t getFontTotalChar(void);
    void setCursor(uint8_t x, uint8_t y);

    // LCD Rotate Scroll functions
    void scrollRight(uint8_t start, uint8_t stop, uint8_t speed);
    void scrollLeft(uint8_t start, uint8_t stop, uint8_t speed);

    //TODO Add 0x29/0x2A vertical scrolling commands
    //void scrollUp(uint8_t start, uint8_t stop);
    //void scrollVertLeft(uint8_t start, uint8_t stop);

    void scrollStop(void);




};








#endif /* SF_RGB_OLED_64X64_H */