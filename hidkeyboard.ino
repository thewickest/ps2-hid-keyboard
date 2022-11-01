/*********************************************************************
  This is an example for our nRF51822 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include "ps2_Keyboard.h"
#include "ps2_AnsiTranslator.h"
#include "ps2_UsbTranslator.h"
#include "ps2_SimpleDiagnostics.h"

#define DATA_PIN 2
#define CLOCK_PIN 3

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
//Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/**********************Use for keyboard Input******************************/
typedef ps2::SimpleDiagnostics<254> Diagnostics_;
static Diagnostics_ diagnostics;
static ps2::AnsiTranslator<Diagnostics_> keyMapping(diagnostics);
static ps2::UsbTranslator<Diagnostics_> usbMapping(diagnostics);
static ps2::Keyboard<DATA_PIN, CLOCK_PIN, 1, Diagnostics_> ps2Keyboard(diagnostics);
static ps2::KeyboardLeds lastLedSent = ps2::KeyboardLeds::none;
/**************************************************************************/

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Keyboard Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit Keyboard': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Bluefruit Keyboard" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  } else
  {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  /*Setting Up Keyboard*/
  setUpKeyboard();

  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts keyboard input"));

  Serial.println();
  Serial.println(F("Enter the character(s) to send:"));
  Serial.println(F("- \\r for Enter"));
  Serial.println(F("- \\n for newline"));
  Serial.println(F("- \\t for tab"));
  Serial.println(F("- \\b for backspace"));

  Serial.println();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
uint8_t lastKeyCode;
uint8_t sumKey; //the sum of the HID mod key.
uint8_t arrKeys[6];
String sixKeys; //string of arrKeys join with "-". The HID keys to be sent.
bool isDown = false;

void loop(void)
{
  diagnostics.setLedIndicator<LED_BUILTIN>();
  ps2::KeyboardOutput scanCode = ps2Keyboard.readScanCode();
  if (scanCode == ps2::KeyboardOutput::garbled) {
    keyMapping.reset();
  }
  else if (scanCode != ps2::KeyboardOutput::none)
  {
    ps2::UsbKeyAction usbCode = usbMapping.translatePs2Keycode(scanCode); //get the last keycode pressed
    if (usbCode.gesture != 2 ) {
      sendScanCode(usbCode);
    }

    if (usbCode.gesture == 1) {
      ps2::KeyboardLeds newLeds =
        (usbCode.hidCode == 0x39 ? ps2::KeyboardLeds::capsLock : ps2::KeyboardLeds::none)
        | (usbCode.hidCode == 0x53 ? ps2::KeyboardLeds::numLock : ps2::KeyboardLeds::none)
        | (usbCode.hidCode == 0x47 ? ps2::KeyboardLeds::scrollLock : ps2::KeyboardLeds::none);

      /*XOR because I want to store the leds everytime it is 1 on that key*/
      uint8_t ledsValue = (uint8_t)newLeds ^ (uint8_t)lastLedSent;
      /*Cast into KeyboardLeds struct*/
      newLeds = (ps2::KeyboardLeds)ledsValue;
      ps2Keyboard.sendLedStatus(newLeds);
      lastLedSent = newLeds;
    }
  }
}

void sendScanCode(ps2::UsbKeyAction usbCode) {

  if (usbCode.gesture == 1) {
    uint8_t pressedKey = (uint8_t)usbCode.hidCode; //ej.: 0xE0, 0xE1, etc...
    /*Esto es para los modificadores. En la tabla de USB/HID codes los mods tienen los valores del 0xE0 hasta el 0xE7*/
    if (pressedKey >= 0xE0) {     /*Modificadores*/
      uint8_t desp = ~0xE0 & pressedKey; //ej.: With modkey == 0xE1 should give 0x01
      uint8_t modKey = 0x01 << desp; //ej.: 0x01, 0x02, etc...
      sumKey = modKey ^ lastKeyCode;
      /*ble.print("AT+BleKeyboardCode=");
      ble.print(sumKey, HEX);
      ble.print("-00");
      ble.println(sixKeys);*/
      sendKeys();
      lastKeyCode = sumKey;
    } else {                     /*Teclas normales*/
      pushKey(pressedKey); //concat the pressedKey into the array of pressed Keys. It contains 6 keys maximum
      concatArray(); //returns a concat string into the string sixKeys;
      /*ble.print("AT+BleKeyboardCode=");
      ble.print(sumKey,HEX);
      ble.print("-00");
      ble.println(sixKeys);*/
      sendKeys();
    }
  } else if (usbCode.gesture == 0) {
    uint8_t pressedKey = (uint8_t) usbCode.hidCode;
    if (pressedKey >= 0xE0) {
      uint8_t desp = ~0xE0 & pressedKey; //ej.: With modkey == 0xE1 should give 0x01
      uint8_t modKey = 0x01 << desp; //ej.: 0x01, 0x02, etc...
      sumKey = modKey ^ lastKeyCode;
    }
    lastKeyCode = sumKey;
    popKey(pressedKey);
    concatArray();
    if(sixKeys == "" && sumKey == 0x00){
      ble.println("AT+BleKeyboardCode=00-00"); 
    }else{
      sendKeys();
    }
  }

  /*No esperamos al OK porque sino cada tecla se envía muy lenta*/
  /*Ver la forma de cómo enviar chunks de información si queremos incluir esto*/
  /*if( ble.waitForOK() )
    {
    Serial.println( F("OK!") );
    }else
    {
    Serial.println( F("FAILED!") );
    }*/
}

void setUpKeyboard() {
  ps2Keyboard.begin();
  keyMapping.setNumLock(true);
  ps2Keyboard.awaitStartup();

  // see the docs for awaitStartup - TL;DR <- when we reset the board but not the keyboard, awaitStartup
  //  records an error because it thinks the keyboard didn't power-up correctly.  When debugging, that's
  //  true - but only because it never powered down.
  diagnostics.reset();

  ps2Keyboard.sendLedStatus(ps2::KeyboardLeds::numLock);
  lastLedSent = ps2::KeyboardLeds::numLock;
}

void sendKeys(){
  ble.print("AT+BleKeyboardCode=");
  ble.print(sumKey,HEX);
  ble.print("-00");
  ble.println(sixKeys);
}

void pushKey(uint8_t key) {
  for(int i=0; i<sizeof(arrKeys);i++){ //the number of keys that can be sent
    if(arrKeys[i]==NULL){
      arrKeys[i] = key;
      break;
    }
  }
}

void popKey(uint8_t key){
  for(int i=0; i<sizeof(arrKeys);i++){ //the number of keys that can be sent
    if(arrKeys[i]==key){
      arrKeys[i]=NULL;
    }
  }
  //memset(arrKeys, 0, sizeof(arrKeys));
}

void concatArray(){
  //String sixKeys
  String conc = "";
  for(int i=0;i <sizeof(arrKeys);i++){
    if(arrKeys[i]!=NULL){
      conc.concat("-");
      conc.concat(String(arrKeys[i],HEX));
    }
  }
  sixKeys = conc;
}

void blePrintArr(uint8_t keys[]) {
  for (int i = 0; i < 6; i++) {//the number of keys that can be sent
    ble.print("-");
    ble.print(keys[1],HEX);    
  }
  ble.println();
}
