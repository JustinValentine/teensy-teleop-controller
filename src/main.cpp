#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"

// --- Hardware Pin Definitions ---
const int motorPinA = 6; 
const int motorPinB = 7;
const int thumbXPin  = 15;  // URX
const int thumbYPin  = 14;  // URY
// const int triggerPin = 16;  // FDBK 
const int triggerPin      = 17;  // R1
const int bumperPin = 20; // bumper

// --- Objects & Network ---
FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;
Adafruit_DRV2605 drv;

// --- Haptic Spring Variables ---
const int triggerRestPos = 200;  // The ADC value when the trigger is untouched
float dynamicStiffness = 0.2;    // Starts light (kP multiplier)
const float minStiffness = 0.2;  // Base spring force for moving through empty air
const float maxStiffness = 2.0;  // Max pushback when gripper is stalled/crushing
int springForce = 0;


// Timing
unsigned long lastCanSend = 0;
const int canInterval = 10; 

void setup() {
  Serial.begin(115200);
  
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(triggerPin, INPUT);
  pinMode(bumperPin, INPUT_PULLUP);
  
  Wire.begin();
  if (drv.begin()) {
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_INTTRIG); 
  }

  canFD.begin();

  CANFD_timings_t config;
  config.clock = CLK_60MHz;
  config.baudrate = 1000000;    // 1Mbps Nominal speed
  config.baudrateFD = 5000000;  // 5Mbps Data speed
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 70;
  
  canFD.setBaudRate(config);
}

void loop() {
  // ---------------------------------------------------------
  // 1. LISTEN TO THE PC (Update Haptic State)
  // ---------------------------------------------------------
  CANFD_message_t rxMsg;
  while (canFD.read(rxMsg)) {
    // TODO: This is gross. I hate it.
    if (rxMsg.id != 0x11 && rxMsg.len >= 1) {
    // if (rxMsg.id == 0x22) {
      // Expecting Byte 0 to be a 0-255 "stiffness command"

      springForce = rxMsg.buf[0]; 
    }
  }

  // ---------------------------------------------------------
  // 2. READ LOCAL SENSORS
  // ---------------------------------------------------------
  int triggerPos = analogRead(triggerPin);
  bool bumperPressed = !digitalRead(bumperPin);
  int thumbX = analogRead(thumbXPin);
  int thumbY = analogRead(thumbYPin);
  // Serial.print("Trigger: ");
  // Serial.print(triggerPos);
  // Serial.print("\tThumb X: ");
  // Serial.print(thumbX);
  // Serial.print("\tThumb Y: ");
  // Serial.print(thumbY);
  // Serial.print("\tBumper: ");
  // Serial.println(bumperPressed ? "PRESSED" : "Open");

  // ---------------------------------------------------------
  // 3. Trigger Smart Spring
  // ---------------------------------------------------------
  
  if (springForce > 255) springForce = 255; // Hardware PWM limit
  
  // Drive motor to push finger back toward rest
  analogWrite(motorPinA, springForce);
  analogWrite(motorPinB, 0);

  // ---------------------------------------------------------
  // 4. I2C BUMPER HAPTICS (State Change Feedback)
  // ---------------------------------------------------------
  static bool lastBumper = false;
  if (bumperPressed && !lastBumper) {
    drv.setWaveform(0, 1);  // Heavy click for state toggle
    drv.setWaveform(1, 0);  
    drv.go();
  }
  lastBumper = bumperPressed;

  // ---------------------------------------------------------
  // 5. BROADCAST TO PC
  // ---------------------------------------------------------
  if (millis() - lastCanSend >= canInterval) {
    lastCanSend = millis();
    
    CANFD_message_t txMsg;
    txMsg.id = 0x11;
    txMsg.flags.extended = 0;
    txMsg.edl = 1;
    txMsg.brs = 1;
    txMsg.len = 8;

    txMsg.buf[0] = triggerPos >> 8;
    txMsg.buf[1] = triggerPos & 0xFF;
    txMsg.buf[2] = thumbX >> 8;
    txMsg.buf[3] = thumbX & 0xFF;
    txMsg.buf[4] = thumbY >> 8;
    txMsg.buf[5] = thumbY & 0xFF;
    txMsg.buf[6] = bumperPressed ? 1 : 0;
    txMsg.buf[7] = springForce;

    canFD.write(txMsg);
  }
}
