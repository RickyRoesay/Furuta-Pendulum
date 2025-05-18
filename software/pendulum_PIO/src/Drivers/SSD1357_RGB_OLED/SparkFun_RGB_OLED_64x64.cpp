#include "SparkFun_RGB_OLED_64x64.h"


		


RGB_OLED_64x64::RGB_OLED_64x64()
{
}


void RGB_OLED_64x64::begin(uint8_t dcPin, uint8_t rstPin, uint8_t csPin, SPIClass &spiInterface, uint32_t spiFreq)
{
	// Associate 
	_dc = dcPin;
	_rst = rstPin;
	_cs = csPin;
	_spi = &spiInterface;

	_spiFreq = spiFreq;

	linkDefaultFont();

	// Set pinmodes
	pinMode(_cs, OUTPUT);
	pinMode(_rst, OUTPUT);
	pinMode(_dc, OUTPUT);

	// Set pins to default positions
	digitalWrite(_cs, HIGH);
	digitalWrite(_rst, HIGH);
	digitalWrite(_dc, HIGH);

	// Transmit just one byte without a target to 'set' the spi hardware
	uint8_t temp_buff[1];
	_spi->beginTransaction(SPISettings(_spiFreq, SSD1357_SPI_DATA_ORDER, SSD1357_SPI_MODE));
	_spi->transfer(temp_buff, 1);
	_spi->endTransaction();

	// Perform the startup procedure
	toggleResetPin();
	defaultConfigure();

	/** Specific to the 64x64 display, these must be set after
	 * calling "defaultConfigure()" */
	_width = OLED_WIDTH;
	_height = OLED_HEIGHT;

	_fillColor = 0xFFFF;
	
}



void RGB_OLED_64x64::defaultConfigure( void )
{
	// This is the suggested initialization routine from WiseChip (pg. 9 of the datasheet)
	setCommandLock(false);
  	setSleepMode(true);

  	// Initial settings configuration
  	setClockDivider(0xB0);

  	setMUXRatio(OLED_MUX_NUM); 
	
  	setDisplayOffset(0x00);
  	setDisplayStartLine(0x00);
  	setRemapColorDepth(OLED_ADDRESS_INCREMENT_ORDER,  
						OLED_COLUMN_ADDR_INVERTED, // OLED_COLUMN_ADDR_NONINVERTED    OLED_COLUMN_ADDR_INVERTED
						true, // color swap 
						OLED_COM_SCAN_INV_SETTING, 
						true, // split odd even
						SSD1357_COLOR_MODE_65k);
  	_colorMode = SSD1357_COLOR_MODE_65k;

  	setContrastCurrentABC(0x88, 0x32, 0x88);
  	setMasterContrastCurrent(0x0F);
  	setResetPrechargePeriod(0x02, 0x03);
  
  	uint8_t MLUT[63] = {0x02, 0x03, 0x04, 0x05,
                      	0x06, 0x07, 0x08, 0x09,
                      	0x0A, 0x0B, 0x0C, 0x0D,
                      	0x0E, 0x0F, 0x10, 0x11,
                      	0x12, 0x13, 0x15, 0x17,
                      	0x19, 0x1B, 0x1D, 0x1F,
                      	0x21, 0x23, 0x25, 0x27,
                      	0x2A, 0x2D, 0x30, 0x33,
                      	0x36, 0x39, 0x3C, 0x3F,
                      	0x42, 0x45, 0x48, 0x4C,
                      	0x50, 0x54, 0x58, 0x5C,
                      	0x60, 0x64, 0x68, 0x6C,
                      	0x70, 0x74, 0x78, 0x7D,
                      	0x82, 0x87, 0x8C, 0x91,
                      	0x96, 0x9B, 0xA0, 0xA5,
                      	0xAA, 0xAF, 0xB4};
  	setMLUTGrayscale(MLUT);

  	setPrechargeVoltage(0x17);
  	setVCOMH(0x05);
  	setColumnAddress(OLED_START_COL_IDX, OLED_STOP_COL_IDX);
  	setRowAddress(OLED_START_ROW_IDX, OLED_STOP_ROW_IDX);
  	setDisplayMode(SSD1357_CMD_SDM_RESET);

  	setWidth(OLED_WIDTH);
  	setHeight(OLED_HEIGHT);

  	setSleepMode(false);

  	delay(200);

  	setFontCursorValues(OLED_START_X_IDX, OLED_START_Y_IDX, OLED_START_X_IDX, OLED_START_Y_IDX, OLED_STOP_X_IDX, OLED_STOP_Y_IDX);
}

void RGB_OLED_64x64::clearDisplay(uint8_t mode)
{
	fillDisplay(0x0000);
}

void RGB_OLED_64x64::fillDisplay(uint16_t value)
{
	/** once "fast_filled_rectangle" has the WIDTH_IS_PROPORTIONAL_TO_COLUMNS incorporated 
	 * into it's "writeRam" function call (like is the case with the "write" function), 
	 * we will be able to pass x and y coordinates relative to the screens indended orientation.  
	 * 
	 * Until then, simply pass the hard coded column and row indexes into this function... */
	fast_filled_rectangle(OLED_START_COL_IDX, OLED_START_ROW_IDX, OLED_STOP_COL_IDX, OLED_STOP_ROW_IDX, value);
}




void RGB_OLED_64x64::invert(bool inv)
{
	if(inv)
	{
		setDisplayMode(SSD1357_CMD_SDM_INVERSE);
	}
	else
	{
		setDisplayMode(SSD1357_CMD_SDM_RESET);
	}
}

// void RGB_OLED_64x64::setContrast(uint8_t contrast)
// {

// }

// void RGB_OLED_64x64::flipVertical(bool flip)		// No support yet


void RGB_OLED_64x64::flipHorizontal(bool flip)
{
	_isFlippedH = flip;
	setRemapColorDepth(_incV, flip, _coSwapped, _scanReversed, true, _colorMode);
}




void RGB_OLED_64x64::setCursor(uint8_t x, uint8_t y)
{
	setCursorRAM(OLED_START_COL_IDX + x, OLED_START_ROW_IDX + y);
}


uint16_t RGB_OLED_64x64::getDisplayWidth(void)
{
	return _width;
}

uint16_t RGB_OLED_64x64::getDisplayHeight(void)
{
	return _height;
}

void RGB_OLED_64x64::setDisplayWidth(uint16_t width)
{

}

void RGB_OLED_64x64::setDisplayHeight(uint16_t height)
{
	setMUXRatio(height);
}

void RGB_OLED_64x64::setFillColor(uint16_t value)
{
	_fillColor = value;
}


void RGB_OLED_64x64::scrollRight(uint8_t start, uint8_t stop, uint8_t speed)
{
	uint8_t scrollParameter = 0xFF;
	setupHorizontalScroll(scrollParameter, start, stop, speed);
	startScrolling();
}

void RGB_OLED_64x64::scrollLeft(uint8_t start, uint8_t stop, uint8_t speed)
{
	uint8_t scrollParameter = 0x01;
	setupHorizontalScroll(scrollParameter, start, stop, speed);
	startScrolling();
}

//TODO Add 0x29/0x2A vertical scrolling commands
// void RGB_OLED_64x64::scrollUp(uint8_t start, uint8_t stop);
//void scrollVertLeft(uint8_t start, uint8_t stop);

void RGB_OLED_64x64::scrollStop(void)
{
	stopScrolling();
}




