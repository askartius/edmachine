/*  
    Program to control EDM machine via ESP and hardware buttons
    
    Authors: Askar Idrisov (ESP communications), Timur Idrisov (machine control & electrical discharge process)
*/

#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd (0x27, 16, 2);

const int DIR_PIN = "YOUR_DIRECTION_PIN";
const int STEP_PIN = "YOUR_STEP_PIN";
const int LED = LED_BUILTIN; // Builtin LED on Arduino
const int START_PIN = "YOUR_START_BUTTON_PIN"; // Start button

const int REVOLUTION = "YOUR_LEAD"; // Change in coordinate after one revolution in μm
const int STEPS_PER_REVOLUTION = "YOUR_STEPS_PER_REVOLUTION"; // Change this to fit the number of steps per revolution
const float STEP = (float) REVOLUTION / STEPS_PER_REVOLUTION;
const int JOG = "YOUR_JOG"; // Change in coordinate after one jog in μm
const int STEPS_PER_JOG = STEPS_PER_REVOLUTION * JOG / REVOLUTION; // Properties of SMC LEY16C
const int STEP_DIVIDER = "YOUR_STEP_DIVIDER";

int gapVoltage = "YOUR_GAP_VOLTAGE"; // Voltage "limit" in a spark gap in V
int pulseTime = "YOUR_PULSE_TIME"; // in μs
int pauseTime = "YOUR_PAUSE_TIME"; // in μs
int pulsePeriod = pulseTime + pauseTime; // in μs
int dutyCycle = 1024 *  pulseTime / pulsePeriod;
int nGroupPulses = "YOUR_NUMBER_OF_PULSES_IN_GROUP"; // Number of cycles
int washGapReal = "YOUR_WASH_GAP"; // in μm
int washGap = washGapReal / 50; // in μm, luft = 40..70μm - a coefficient
long zCoordinate = 0; // in μm
long zSteps = 0; // Steps from zero coordinate
int dir = 0; // Jog direction for calculating coordinate (1 - UP, -1 - DOWN)
bool runProcess = false;
volatile bool stopProcess = false; // STOP button

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(START_PIN, INPUT);
  pinMode(LED, OUTPUT);
  pinMode("YOUR_STOP_BUTTON_PIN", INPUT_PULLUP);
  pinMode("YOUR_TIMER_PIN", OUTPUT); // TIMER - Generator of pulse
  
  attachInterrupt(digitalPinToInterrupt("YOUR_STOP_BUTTON_PIN"), stop_interrupt, FALLING); // STOP button
  attachInterrupt(digitalPinToInterrupt("YOUR_ESP_STOP_PIN"), stop_interrupt, CHANGE); // ESP STOP Pin
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(7, 0);
  lcd.print("|");
  lcd.setCursor(0, 1);
  lcd.print("TI=");
  lcd.print(pulseTime);
  lcd.setCursor(7, 1);
  lcd.print("|");
  lcd.setCursor(8, 1);
  lcd.print("TP=");
  lcd.print(pauseTime);
  
  Serial.begin(115200);
  Serial.print("*# Pulse period: ");
  Serial.print(pulsePeriod);
  Serial.print('\n');
  Serial.print("*# Duty cycle: ");
  Serial.print(dutyCycle);
  Serial.print('\n');
}

void loop() {
  // Display voltage on the LCD
  lcd.setCursor(8, 0);
  lcd.print("V ");
  lcd.print(analogRead(A0) / 10);
  lcd.print('/');
  lcd.print(gapVoltage);
  lcd.print("   ");

  if (digitalRead(START_PIN) == HIGH) {
    runProcess = true;
    lcd.setCursor(0, 0);
    lcd.print("RUN ");
    Serial.print("*Z\n"); // Send proccess state to the app
  }
  
  if (runProcess) {
    Serial.print("*V ");
    Serial.print(analogRead(A0) / 10);
    Serial.print('\n');
    
    Timer1.initialize(pulsePeriod);
    Timer1.pwm(9, dutyCycle); // pulsePeriod - period, dutyCycle - fill factor 0-1023
    
    if (analogRead(A0) > gapVoltage * 10) { // Analog input signal gapVoltage
      for (int pulseCount = 0; pulseCount < nGroupPulses; pulseCount++) {
        digitalWrite(DIR_PIN, LOW); // Jog DOWN
        dir = -1;
        
        if (stopProcess) { // STOP button
          onProcessStopped();
          break;
        }
        
        while (analogRead(A0) > gapVoltage * 10) { // One step is ~6.25 μm, jog DOWN and control the gap voltage
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(50);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(50);
          zSteps += dir * 1;
          //digitalWrite(LED, LOW); delay(00); digitalWrite(LED, HIGH);
          
          if (stopProcess) { // STOP button
            onProcessStopped();
            break;
          }
        }

        digitalWrite(DIR_PIN, HIGH); // Jog UP
        dir = 1;
        for (long stepCount = STEPS_PER_REVOLUTION / 10; stepCount > 0; stepCount--) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(25);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(25);
        }
        zSteps += dir * STEPS_PER_REVOLUTION / 10;
        sendCoordinates();
      }
    }
    
    Timer1.stop();
    digitalWrite(9, LOW);
    digitalWrite(LED, LOW);
    
    digitalWrite(DIR_PIN, HIGH); // Jog UP
    dir = 1;
    for (long stepCount = STEPS_PER_REVOLUTION * washGap; stepCount > 0; stepCount--) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(50);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(50);
    }
    zSteps += dir * STEPS_PER_REVOLUTION * washGap;
    sendCoordinates();
    //delay(500); // Delay for cooling
    digitalWrite(DIR_PIN, LOW); // Jog DOWN
    dir = -1;
    for (long stepCount = STEPS_PER_REVOLUTION * washGap - 8; stepCount > 0; stepCount--) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(50);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(50);
    }
    zSteps += dir * STEPS_PER_REVOLUTION * washGap - 8;
    sendCoordinates();
  }
  else if (Serial.available()) { // Process stopped, wait for incoming data from the Serial port
    String data = Serial.readStringUntil('\n');

    if (data.charAt(0) == '*') {
      char command = data.charAt(1);

      if (command == 'C' && data.charAt(2) == '1') { // Client connected, send data
        Serial.print("*A\n");
        sendCoordinates();
        sendParameters();
      }
      
      else if (command == 'Z') { // Start the process
        runProcess = true;
        lcd.setCursor(0, 0);
        lcd.print("RUN ");
        Serial.print("*Z\n");
      }
      
      else if (command == 'P') { // Parameters
        data = data.substring(3);

        int newPulseTime = data.substring(0, data.indexOf(' ')).toInt();
        int newPauseTime = data.substring(data.indexOf(' ') + 1, data.lastIndexOf(' ')).toInt();
        int newGapVoltage = data.substring(data.lastIndexOf(' ') + 1).toInt();

        if (newPulseTime > 0) {
          pulseTime = newPulseTime;
        }
        if (newPauseTime > 0) {
          pauseTime = newPauseTime;
        }
        if (newGapVoltage > 0) {
          gapVoltage = newGapVoltage;
        }
        pulsePeriod = pulseTime + pauseTime;
        dutyCycle = 1024 *  pulseTime / pulsePeriod;

        //  Update data on the LCD
        lcd.clear();
        lcd.setCursor(7, 0);
        lcd.print("|");
        lcd.setCursor(0, 1);
        lcd.print("TI=");
        lcd.print(pulseTime);
        lcd.setCursor(7, 1);
        lcd.print("|");
        lcd.setCursor(8, 1);
        lcd.print("TP=");
        lcd.print(pauseTime);
        
        sendParameters();
      }
      
      else if (command == 'J') { // Jog to
        data = data.substring(3);

        long targetZCoordinate = data.substring(0, data.indexOf(' ')).toInt() * STEP_DIVIDER;
        int speedMultiplier = data.substring(data.indexOf(' ') + 1).toInt();

        if (targetZCoordinate > zCoordinate) {
          digitalWrite(DIR_PIN, HIGH); // Jog UP
          dir = 1;
        }
        else {
          digitalWrite(DIR_PIN, LOW); // Jog DOWN
          dir = -1;
        }

        zCoordinate = zSteps * STEP;
        for (long stepCount = STEPS_PER_JOG * abs(zCoordinate - targetZCoordinate) / JOG; stepCount > 0; stepCount--) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(625 / speedMultiplier);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(625 / speedMultiplier);
          if (stepCount % (STEPS_PER_JOG * STEP_DIVIDER) == 0) {
            zSteps += dir * STEPS_PER_JOG * STEP_DIVIDER;
            sendCoordinates();
          }
        }
      }
      
      else if (command == 'R') { // Reset coordinate
        zCoordinate = 0;
        zSteps = 0;
        Serial.print("*R\n");
      }
      
      else if (command == '*') { // Message
        Serial.print("*#");
        Serial.print(data.substring(2));
        Serial.print('\n');
      }
    }
  }
  
  if (stopProcess) {
    onProcessStopped();
    
    digitalWrite(DIR_PIN, HIGH); // Jog UP
    dir = 1;
    for (long stepCount = STEPS_PER_REVOLUTION * 20; stepCount > 0; stepCount--) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(125);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(125);
      if (stepCount % STEPS_PER_REVOLUTION == 0) {
        zSteps += dir * STEPS_PER_REVOLUTION;
        sendCoordinates();
      }
    }
    
    stopProcess = false;
  }
}

void stop_interrupt() {
  stopProcess = true;
}

void sendCoordinates() {
  zCoordinate = zSteps * STEP;
  Serial.print("*J ");
  Serial.print(zCoordinate / STEP_DIVIDER);
  Serial.print('\n');
}

void sendParameters() {
  Serial.print("*P ");
  Serial.print(pulseTime);
  Serial.print(' ');
  Serial.print(pauseTime);
  Serial.print(' ');
  Serial.print(gapVoltage);
  Serial.print('\n');
}

void onProcessStopped() {
  Timer1.stop();
  digitalWrite(9, LOW);
  digitalWrite(LED, LOW);
  
  runProcess = false;
  Serial.print("*A\n");
  Serial.print("*V ?\n");
  
  lcd.setCursor(0, 0);
  lcd.print("STOP ");
}
