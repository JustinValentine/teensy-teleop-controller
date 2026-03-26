#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"

// --- Hardware Pin Definitions ---
const int motorPinA = 2; 
const int motorPinB = 3;
const int triggerPin = 14;  
const int thumbXPin  = 15;  
const int bumperPin  = 16;  
const int thumbYPin  = 17;  

// --- Objects & Network ---
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;
Adafruit_DRV2605 drv;

// --- Haptic Spring Variables ---
const int triggerRestPos = 130;  // The ADC value when the trigger is untouched
float dynamicStiffness = 0.2;    // Starts light (kP multiplier)
const float minStiffness = 0.2;  // Base spring force for moving through empty air
const float maxStiffness = 2.0;  // Max pushback when gripper is stalled/crushing

// Joystick Deadzone Settings
const int joyCenter = 512;   
const int joyDeadzone = 40;  

// Timing
unsigned long lastCanSend = 0;
const int canInterval = 10; 

int applyDeadzone(int rawValue) {
  if (abs(rawValue - joyCenter) < joyDeadzone) {
    return joyCenter; 
  }
  return rawValue;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(bumperPin, INPUT_PULLUP);
  
  Wire.begin();
  if (drv.begin()) {
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_INTTRIG); 
  }

  canFD.begin();
  canFD.setBaudRate(1000000);       
  canFD.setBaudRateData(5000000);   
  canFD.setClock(CLK_60MHz);
}

void loop() {
  // ---------------------------------------------------------
  // 1. LISTEN TO THE PC (Update Haptic State)
  // ---------------------------------------------------------
  CANFD_message_t rxMsg;
  if (canFD.read(rxMsg)) {
    // If the PC sends a message on ID 0x22
    if (rxMsg.id == 0x22) {
      // Expecting Byte 0 to be a 0-255 "stiffness command"
      int incomingStiffness = rxMsg.buf[0]; 
      
      // Map the 0-255 command to our actual kP multiplier bounds
      dynamicStiffness = incomingStiffness * ((maxStiffness - minStiffness) / 255.0) + minStiffness;
    }
  }

  // ---------------------------------------------------------
  // 2. READ LOCAL SENSORS
  // ---------------------------------------------------------
  int triggerPos = analogRead(triggerPin);
  bool bumperPressed = !digitalRead(bumperPin); 
  int thumbX = applyDeadzone(analogRead(thumbXPin));
  int thumbY = applyDeadzone(analogRead(thumbYPin));

  // ---------------------------------------------------------
  // 3. Trigger Smart Spring
  // ---------------------------------------------------------
  // A spring always wants to return to its resting position.
  // The further you pull it, and the higher the PC sets the stiffness, the harder it fights.
  
  if (triggerPos > triggerRestPos) {
    int displacement = triggerPos - triggerRestPos;
    
    // Calculate spring pushback (Hooke's Law: F = kx)
    int springForce = displacement * dynamicStiffness;
    
    if (springForce > 255) springForce = 255; // Hardware PWM limit
    
    // Drive motor to push finger back toward rest
    analogWrite(motorPinA, springForce);
    analogWrite(motorPinB, 0);
  } else {
    // Trigger is completely released
    analogWrite(motorPinA, 0);
    analogWrite(motorPinB, 0);
  }

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
    txMsg.flags.fd = 1;      
    txMsg.flags.brs = 1;     
    txMsg.len = 8;           

    txMsg.buf[0] = triggerPos >> 8;       
    txMsg.buf[1] = triggerPos & 0xFF;     
    txMsg.buf[2] = thumbX >> 8;           
    txMsg.buf[3] = thumbX & 0xFF;         
    txMsg.buf[4] = thumbY >> 8;           
    txMsg.buf[5] = thumbY & 0xFF;         
    txMsg.buf[6] = bumperPressed ? 1 : 0; 
    txMsg.buf[7] = 0;                     

    canFD.write(txMsg);
  }
}