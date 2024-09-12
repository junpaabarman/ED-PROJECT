#include <LiquidCrystal.h>

// Constants for accelerometer (accident detection)
const int accelerometerZPin = A2;
const float impactThreshold = 15.0;
const unsigned long recoveryTime = 5000;

// Constants for IR sensor (overspeed detection) and LCD
const int irSensorPin = 2;
const int lcdRsPin = 8;
const int lcdEnablePin = 9;
const int lcdD4Pin = 10;
const int lcdD5Pin = 11;
const int lcdD6Pin = 12;
const int lcdD7Pin = 13;
const int maxSpeedThreshold = 6300;
const int irPulseInterval = 1000;

// Variables for accelerometer (accident detection)
int prevZValue;
bool accidentDetected = false;
unsigned long lastAccidentTime = 0;

// Variables for IR sensor (overspeed detection) and LCD
LiquidCrystal lcd(lcdRsPin, lcdEnablePin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
volatile int wheelRotations = 0;
unsigned long lastIrPulseTime = 0;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  lcd.begin(16, 2);     // Initialize LCD display

  // Attach interrupt to IR sensor pin for wheel rotation counting
  attachInterrupt(digitalPinToInterrupt(irSensorPin), countWheelRotation, RISING);
}

void loop() {
  // Read accelerometer value along Z-axis
  int zValue = analogRead(accelerometerZPin);
  float deltaZ = abs(zValue - prevZValue);
  prevZValue = zValue;

  // Check for accident event
  if (!accidentDetected && deltaZ > impactThreshold) {
    // Accident detected
    Serial.println("Accident detected!");
    accidentDetected = true;
    lastAccidentTime = millis(); // Record the time of the accident
  }

  // Check if recovery time has elapsed since the last accident
  if (accidentDetected && (millis() - lastAccidentTime >= recoveryTime)) {
    // Reset accident detection after recovery time
    accidentDetected = false;
  }

  // Calculate RPM every irPulseInterval milliseconds
  if (millis() - lastIrPulseTime >= irPulseInterval) {
    // Calculate RPM from wheel rotations in the last interval
    float rpm = (wheelRotations * (60000.0 / irPulseInterval)); // Convert rotations per millisecond to RPM

    // Display current speed on LCD
    lcd.clear();  // Clear LCD display
    lcd.setCursor(0, 0);
    lcd.print("Speed: ");
    lcd.print((int)rpm); // Display RPM as integer

    // Check for overspeed condition
    if (rpm > maxSpeedThreshold) {
      lcd.setCursor(0, 1);
      lcd.print("Overspeed!");
    }

    // Reset wheel rotations count
    wheelRotations = 0;
    lastIrPulseTime = millis(); // Update last IR pulse time
  }
}

// Interrupt service routine to count wheel rotations
void countWheelRotation() {
  wheelRotations++; // Increment wheel rotations count
}



