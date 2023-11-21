
// https://playground.arduino.cc/Main/RotaryEncoders/
// https://forum.arduino.cc/t/mouse-scroll-wheel-encoder-to-arduino/68935/4
//  as possibly useful references

#include "Adafruit_TinyUSB.h"

/* This sketch demonstrates USB HID mouse
 * Press button pin will move
 * - mouse toward bottom right of monitor
 * 
 * Depending on the board, the button pin
 * and its active state (when pressed) are different
 */
// #if defined ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS
//   const int pin = 4; // Left Button
//   bool activeState = true;
// #elif defined ARDUINO_NRF52840_FEATHER
//   const int pin = 7; // UserSw
//   bool activeState = false;
// #else
//   const int pin = 0;
//   bool activeState = true;
// #endif
const int pinA = 0 ;
const int pinB = 5 ;   
int delta = 0 ; 
// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] =
{
  TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID object
Adafruit_USBD_HID usb_hid;
int state_pinA; 
int state_pinB;
int rotState;
// the setup fu;nction runs once when you press reset or power the board
void setup()
{
  // Set up button, pullup opposite to active state
  // pinMode(pin, activeState ? INPUT_PULLDOWN : INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(pinA), pinChange, CHANGE);
attachInterrupt(digitalPinToInterrupt(pinB), pinChange, CHANGE);
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  // Notes: following commented-out functions has no affect on ESP32
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
  // usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setStringDescriptor("Carter Scroller");

  usb_hid.begin();
  pinMode(pinA, INPUT); // need to find out if these are INPUT or INPUT_PULLUP
  pinMode(pinB, INPUT); // input seems to work

  state_pinA = digitalRead(pinA);
  state_pinB = digitalRead(pinB);
  rotState =  state_pinA + 2*state_pinB;
  // Serial.begin(115200); 

  // wait until device mounted
  while( !USBDevice.mounted() ) delay(1);

  Serial.println("Adafruit TinyUSB HID Mouse example");
}

void loop()
{
  // poll gpio once each 10 ms
  delay(1);

  // Whether button is pressed
  bool any_change = (delta != 0);
  // nothing to do if button is not pressed
  if (!any_change) return;

  // state_
  // Remote wakeup
  if ( USBDevice.suspended() )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    USBDevice.remoteWakeup();
  }

  if ( usb_hid.ready() )
  {
 usb_hid.mouseScroll(0, delta/2,0);
    delta = 0 ; 
  }
  // rotState = new_rotState; 
  return ;
}

void pinChange(){
  int new_A = digitalRead(pinA);
  int new_B = digitalRead(pinB);
  int new_rotState = new_A + 2 * new_B;
  int isFwd = (rotState == 3 && new_rotState == 1) 
      || (rotState ==1 && new_rotState ==0)
      || (rotState==0 && new_rotState==2 ) || (rotState==2 && new_rotState==3) ;
    if (isFwd){delta +=1;}else{delta -=1;};
  rotState= new_rotState;
  return ;
}