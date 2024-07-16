#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#include <Adafruit_MotorShield.h>

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE 1
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);





// A small helper
void error(const __FlashStringHelper *err) {
  Serial.println(err);
  while (1)
    ;
}

uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t *data, const uint32_t numBytes);

extern uint8_t packetbuffer[];

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *Right_Motor = AFMS.getMotor(3);  //Right Motor
Adafruit_DCMotor *Left_Motor = AFMS.getMotor(4);   //Left Motor

void setup() {

  cli();

  sei();


  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if (!ble.begin(VERBOSE_MODE)) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if (FACTORYRESET_ENABLE) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (!ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println(F("Switching to DATA mode!"));
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  //Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {

    Serial.println("Could not find Motor Shield. Check Wiring.");
    while (1)
      ;
  }
  Serial.println("Motor Shield found.");
}

void loop() {

  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;


  uint8_t buttnum;
  boolean pressed;
  
  // Buttons
  if (packetbuffer[1] == 'B') {
    buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    // Serial.print ("Button "); Serial.print(buttnum);
  
if (pressed) {
      Serial.println(" pressed");
    switch (buttnum) {
      case 1:
        Right_Motor->setSpeed(0);
        Left_Motor->setSpeed(0);
        Serial.println ("Stop");
        break;
      case 2:
        Right_Motor->setSpeed(85);
        Left_Motor->setSpeed(85);
        break;
      case 3:
        Right_Motor->setSpeed(170);
        Left_Motor->setSpeed(170);
        break;
      case 4:
        Left_Motor->setSpeed(255);
        Right_Motor->setSpeed(255);
        break;
      case 5:
        Right_Motor->run(FORWARD);
        //Right_Motor->setSpeed(125);
        Left_Motor->run(FORWARD);
       //Left_Motor->setSpeed(125);
       Serial.println ("Forward");
       break;
      case 6:
        Right_Motor->run(BACKWARD);
       //Right_Motor->setSpeed(125);
        Left_Motor->run(BACKWARD);
       //Left_Motor->setSpeed(125);
        Serial.println ("Backward");
        break;
      case 7:
        Right_Motor->run(FORWARD);
        //Right_Motor->setSpeed(125);
        Left_Motor->setSpeed(0);
        Serial.println ("Left Turn");
        break;
      case 8:
        Left_Motor->run(FORWARD);
        //Left_Motor->setSpeed(125);
        Right_Motor->setSpeed(0);
        Serial.println ("Right Turn");
        break;
      }
    } 

    else {
      Serial.println(" released");
      Left_Motor->run(RELEASE);
      Right_Motor->run(RELEASE);
      Serial.println ('Left_Motor->run()');
    }
    
  }

}
