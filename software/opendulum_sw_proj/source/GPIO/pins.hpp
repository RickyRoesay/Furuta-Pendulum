#include "GPIO_Pin.hpp"

#pragma once

/** The "GPIO" software component/source files act 
 * as a way to configure all of the GPIO pins on the 
 * board in one location, in a way that is not super painful 
 * to make changes. */

extern GPIO_Pin gpio_led_data_out;
extern GPIO_Pin gpio_led1;
extern GPIO_Pin gpio_led2;

bool pins_configure_gpio(void);

