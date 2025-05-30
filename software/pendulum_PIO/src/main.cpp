#include <Arduino.h>
#include "STM32_CAN.h"
#include "../lib/Arduino-FOC/src/SimpleFOC.h"


#include "Drivers/DRV8301_Gate_Driver/DRV8301_Gate_Driver.hpp" 
#include "Drivers/AS5048A_MagSenseSPI/AS5048A_MagSenseSPI.hpp" // mag sensor
#include "Drivers/WS2812B_RGB_LED_Strip/WS2812B_RGB_LED_Strip.hpp" // RGB LED's
#include "Drivers/ADC_Interface/ADC_Interface.hpp" 
#include "Drivers/SSD1357_RGB_OLED/OLED_Graphics.h" 

#include "../lib/Arduino-FOC/src/common/foc_utils.h" // sin, cosine, pi, 2pi, etc

#include "CAN_TP/can_tp.hpp"

#include "Biquad.hpp"

#include "gpio.h"
#include "config.hpp"
#include "Gimbal/gimbal.hpp"
#include "RGB_Wrapper/RGB_Wrapper.hpp"

#include "Pendulum/pendulum.hpp"





////////////////////////// DEFINITIONS: ////////////////////////////

#if !defined(_STM32_DEF_)
#define _STM32_DEF_
#endif 

#if !defined(SIMPLEFOC_STM32_DEBUG)
#define SIMPLEFOC_STM32_DEBUG
#endif 





//////////////////////////// CLASS: DECLARATIONS: ////////////////////////////


HardwareTimer ctrl_loop_5kHz_timer = HardwareTimer(TIM4);
//HardwareTimer test_tmr = HardwareTimer(TIM8);

/************* DRV8301: DRIVER: *************/
/** NOTE: both sw_bldc_driver and hw_bldc_driver classes have control over 
 * the EN_GATE, but the only time hw_bldc_driver uses it is on initialization. 
 * 
 * after the hw_bldc_driver's init function is called only the "sw_bldc_driver" class
 * has control of the EN_GATE pin. */
DRV8301_Gate_Driver hw_bldc_driver = DRV8301_Gate_Driver(SPI_nCS_DRV, EN_GATE, nFAULT);
void on_nFAULT() { hw_bldc_driver.fault_pin_asserted_isr_callback(); }


/************* MAG: SENSE: *************/
AS5048A_MagSenseSPI mag_sense = AS5048A_MagSenseSPI(SPI_nCS_IO6);

/** Params are: MOSI, MISO, SCLK, Chip select (optional). Chip select will be controlled 
 * by the calling function. */
SPIClass SPI_2(SPI_MOSI, SPI_MISO, SPI_SCK); 


/************* COMM: *************/
HardwareSerial hw_serial(GPIO4_UART_RX, GPIO3_UART_TX);

Commander command = Commander(hw_serial);// commander interface
void on_motor(char* cmd){ command.motor(&motor, cmd); }


WS2812B_RGB_LED_Strip led_strip = WS2812B_RGB_LED_Strip(USB_DP__LED_DO);
RGB_Wrapper rgb_wrapper = RGB_Wrapper(40);

OLED_Graphics oled = OLED_Graphics();











////////////////////////////////// 5kHz: CONTROL: LOOP: ISR: /////////////////////////////////

void Update_IT_callback(void)
{
  //digitalWrite(GPIO2__SSD_nRST, HIGH); 
  
  pdm_run_control_loop();
  
  /** pack and send CAN msg */
  send_can_Debug1_message(motor.current_sp,
                          motor.current.q,
                          pdm.pdm_theta_dot_filt,
                          motor.shaft_velocity,
                          pdm.pdm_phi,
                          encoder.getMechanicalAngle(),
                          0);
  
  //digitalWrite(GPIO2__SSD_nRST, LOW); 
}







////////////////////////////////// SETUP: /////////////////////////////////

void setup() 
{
  hw_serial.begin(250000); 

  /** only used for timing analysis for now */
  //pinMode(GPIO1__SSD_nCS, OUTPUT);
  //pinMode(GPIO2__SSD_nRST, OUTPUT);
  //digitalWrite(GPIO1__SSD_nCS, LOW);
  //digitalWrite(GPIO2__SSD_nRST, LOW);
  
  rgb_wrapper.link_ws2812b_rgb_led_driver_class(&led_strip);
  rgb_wrapper.init_periphs_for_WS2812B();
  
  RGB_Wrapper_HV_Color_s tmp_led_init_color_blink;
  tmp_led_init_color_blink.hue = 150.0f;  // static hue
  tmp_led_init_color_blink.value = 128;  // static value
  
  RGB_Wrapper_HV_Color_s tmp_led_init_color_pulse;
  tmp_led_init_color_pulse.hue = 330.0f;  //static hue
  tmp_led_init_color_pulse.value = 255;  // max value
  
  RGB_Wrapper_HV_Color_s tmp_led_init_color_circle;
  tmp_led_init_color_circle.hue = 30.0f;
  tmp_led_init_color_circle.value = 128.0f; //30;
  
  
  /** initialize pixel and led circle configuration: */
  //rgb_wrapper.set_rainbow_pixel_settings(0, 1, 30, 120.0f, 0.1f);
  //rgb_wrapper.set_pulse_pixel_settings(2, 2, 80.0, 0.3f, false, tmp_led_init_color_pulse);
  //rgb_wrapper.set_blink_pixel_settings(3, 3, 0.0f, 0.025f, false, tmp_led_init_color_blink);
  rgb_wrapper.set_circle_pixel_settings(0, 39, \
                                        RGB_Wrapper_Circle__Phi_Rainbow_Theta_Q, \
                                        -0.307f, \
                                        0.001f, \
                                        6, \
                                        tmp_led_init_color_circle);
  //
  rgb_wrapper.set_theta_offset_in_radians(-1.264767f, RGB_Wrapper_Theta_Tracking_Direction__INVERSE);
  
  if(rgb_wrapper.finish_initializing_pixel_types() == RGB_Wrapper_Status_UPDATING_LED_BUF)
    hw_serial.println("LED types done initializing.");
  else  
    hw_serial.println("Failed to initialize LED's types/driver!");



  #if 1
  oled.begin(USB_DM__SSD_DS, GPIO2__SSD_nRST, GPIO1__SSD_nCS, SPI_2, 8000000);
  oled.fillDisplay(0); 
  oled.write('B');
  oled.write('O');
  oled.write('O');
  oled.write('T');
  oled.write('I');
  oled.write('N');
  oled.write('G');
  oled.write(' ');
  oled.write('U');
  oled.write('P');
  oled.write('.');
  oled.write('.');
  oled.write('.');
  
  #else 
  /** For RGB LED timing analysis: */
  pinMode(GPIO2__SSD_nRST, OUTPUT);
  digitalWrite(GPIO2__SSD_nRST, LOW);
  pinMode(GPIO1__SSD_nCS, OUTPUT);
  digitalWrite(GPIO1__SSD_nCS, LOW);
  pinMode(USB_DM__SSD_DS, OUTPUT);
  digitalWrite(USB_DM__SSD_DS, LOW);
  #endif 


  /** Prescaler is automatically set when setting overflow with format != tick */
  ctrl_loop_5kHz_timer.setOverflow(200UL, MICROSEC_FORMAT);
  ctrl_loop_5kHz_timer.attachInterrupt(Update_IT_callback);

  pinMode(AUX_H, OUTPUT); 
  pinMode(AUX_L, OUTPUT);
  digitalWrite(AUX_H, LOW);
  digitalWrite(AUX_L, LOW);

  /** Set as inputs to be configured in the simpleFOC's hardware specific
   * "_adc_init" in stm32f4_hal.cpp file, so we can measure bus voltage 
   * at 20kHz just like with current sensors so control loops can run on 
   * data. We don't use a brake resistor but future uses for the ODESC 
   * might. */
  pinMode(VBUS_SNS, INPUT);
  
  // init magnetic angle sensor
  if(mag_sense.init(&SPI_2) != true)
  {
    hw_serial.println("Failed to initialize AS5048A Mag Sensor!");

    /** return early, do not control the motor! */
    return;
  }
  
  mag_sense.update();
  
  pdm_set_theta_offset_when_pointed_down();
  
  if(hw_bldc_driver.init(DRV8301_GAIN_SETTING, &SPI_2, on_nFAULT) != true)
  {
    hw_serial.println("Failed to set gain settings for DRV8301!");

    /** return early, do not control the motor! */
    return;
  }

  can_tp_init();

  adc_if_init(&ctrl_loop_5kHz_timer);

  gimbal_init(hw_serial);

  oled.reconfigureSpiSettings(); 
  oled.write('D');
  oled.write('O');
  oled.write('N');
  oled.write('E');
  oled.write('\n');
  oled.write(' '); // '\n' character is ignored if the font's x position is 0.
  oled.write('\n'); 
  oled.write('M');
  oled.write('O');
  oled.write('T');
  oled.write('O');
  oled.write('R');
  oled.write(' ');
  oled.write('C');
  oled.write('O');
  oled.write('N');
  oled.write('F');
  oled.write('I');
  oled.write('G');
  oled.write('U');
  oled.write('R');
  oled.write('E');
  oled.write('D');

  pdm_torque_setpoint_biquad_c.set_steady_state_val(0.0f);

  command.add('M', on_motor,"my motor motion");
  command.add('C', pdm_on_constants,"Control Constants");

  command.run();
  motor.monitor();
  
  ctrl_loop_5kHz_timer.resume();

  motor.enable();
}








float hue = 0.0f;
uint8_t value = 30;

void loop() 
{
  #if 0
  led_strip.modify_pixel_buffer_all_leds(hue, value);

  hue += 0.01f;
  if(hue >= 360.0f)
  {
    hue = 0.0f;
  }

  if(led_strip.process_bitfield_array(55) == WS2812B_READY_TO_UPLOAD_BITSTREAM)
  {
    (void)led_strip.write_bitfield_array_via_dma();
  }
  #else
    //digitalWrite(USB_DM__SSD_DS, HIGH);
    rgb_wrapper.update_pixels(pdm.pdm_theta /** pdm.pdm_phi */, encoder.getMechanicalAngle());
    //digitalWrite(USB_DM__SSD_DS, LOW);
  #endif 

  
  //oled.write(value);
  if(value > 255)
  {
    value = 55;
    //oled.write(value);
  }
  else
    value++;

  command.run();
  motor.monitor();

  if(pdm.D == 1.0f)
  {
    hw_serial.print(pdm.pdm_phi, 6); //
    hw_serial.print("    ");
    hw_serial.print(pdm.pdm_theta_dot_filt, 6);
    hw_serial.print("    ");
    hw_serial.print(encoder.getVelocity(), 6);
    hw_serial.print("    ");
    hw_serial.print(encoder.getMechanicalAngle(), 6);
    hw_serial.print("    ");
    hw_serial.println(pdm.offset.median_phi_in_revolution, 6);
  }
  else if(pdm.D == 2.0f)
  {
    uint32_t test = adc_if_get_phB_LS_CS_SO1_counts();
    hw_serial.print(test); //
    hw_serial.print("    ");
    test = adc_if_get_phC_LS_CS_SO2_counts();
    hw_serial.print(test);
    hw_serial.print("    ");
    test = adc_if_get_vbus_counts();
    hw_serial.println(test);
  }
  else if(pdm.D == 3.0f)
  {
    float test = adc_if_get_vbus_v();
    hw_serial.print(test); //
    hw_serial.print("    ");
    test = adc_if_get_pcb_temp_near_motor_fets_C();
    hw_serial.print(test);
    hw_serial.print("    ");
    test = adc_if_get_pcb_temp_near_aux_fets_C();
    hw_serial.println(test);
  }

  pdm_run_phi_offset_correction_checks();
}


