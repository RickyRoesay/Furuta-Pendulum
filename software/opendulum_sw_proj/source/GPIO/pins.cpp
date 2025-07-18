#include "pins.hpp"


/** These are great C++ practices and I won't be taking any questions :)    */

GPIO_Pin gpio_led_data_out = GPIO_Pin();
GPIO_Pin gpio_led1 = GPIO_Pin();
GPIO_Pin gpio_led2 = GPIO_Pin();


bool pins_configure_gpio(void)
{
    bool tmp_ret_val__has_config_failed = false;

    LL_GPIO_InitTypeDef tmp_GPIO_InitStruct = {0};

    /////////////////////// RGB: LED: DATA: OUT: ///////////////////////
    /** PB3, either for use as SSD1357__D# or LED_Data_out.
     * Right now we are using it as the rgb led data out pin. */
    tmp_GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
    tmp_GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    tmp_GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    tmp_GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    tmp_GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    tmp_GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    tmp_ret_val__has_config_failed |= gpio_led_data_out.config_pin(GPIOB, &tmp_GPIO_InitStruct);

    /////////////////////// LED 1 ///////////////////////
    /** PB15, used as "LED1" on the board. */
    tmp_GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    tmp_GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    tmp_GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    tmp_GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    tmp_GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    tmp_GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    tmp_ret_val__has_config_failed |= gpio_led1.config_pin(GPIOB, &tmp_GPIO_InitStruct);

    /////////////////////// LED 2 ///////////////////////
    /** PC6, used as "LED2" on the board. */
    tmp_GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
    tmp_GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    tmp_GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    tmp_GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    tmp_GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    tmp_GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    tmp_ret_val__has_config_failed |= gpio_led2.config_pin(GPIOC, &tmp_GPIO_InitStruct);

    return tmp_ret_val__has_config_failed;
}