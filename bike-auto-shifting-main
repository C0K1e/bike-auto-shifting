#include <Servo.h> // Include the Servo library

// Create a Servo object for the actuator
Servo actuator;

// Pin Definitions
const int servoPin = 9;             // Servo control pin connected to pin 9
const int shiftUpButtonPin = 11;    // Shift up button connected to pin 5
const int shiftDownButtonPin = 12;  // Shift down button connected to pin 3
const int calibrateButtonPin = 13;  // Calibration button connected to pin 4
const int sensorPin = 3;            // Hall sensor connected to pin 3

// RGB LED Pins
const int redPin = 5;    // Red LED pin
const int greenPin = 2;  // Green LED pin
const int bluePin = 6;   // Blue LED pin

// Actuator Specifications
const float actuatorStroke_mm = 50.0;  // Total stroke length in mm
const int actuatorMaxMicros = 2000;    // Maximum servo pulse width in microseconds
const int actuatorMinMicros = 1000;    // Minimum servo pulse width in microseconds
const float microsPerMm = (actuatorMaxMicros - actuatorMinMicros) / actuatorStroke_mm; // Microseconds per mm

// Configurable Parameters
int chainringTeeth = 42;            // Number of teeth on the chainring (adjustable)
int cassetteTeeth[] = {11, 13, 15, 17, 19, 22, 25, 28, 32, 32, 36}; // Teeth on cassette sprockets
int totalGears = sizeof(cassetteTeeth) / sizeof(cassetteTeeth[0]);  // Total number of gears
float maxSpeed = 40.0;              // Maximum speed for the highest gear (km/h)
float pullFactor_mm = 2.8;          // Cable pull per gear shift in mm
float pullFactor_micros = pullFactor_mm * microsPerMm; // Microseconds per gear shift
float wheelCircumference = 2.6;    // Wheel circumference in meters

// Speed and Hall Sensor Variables
volatile unsigned long lastPulseTime = 0; // Time of the last pulse
volatile unsigned long pulseInterval = 0; // Time between pulses
const unsigned long debounceTime = 100000; // Debounce time in microseconds (100 ms)
float speed = 0;                          // Speed in km/h

// Variables to Track Actuator State
int currentGear = totalGears;                   // Current gear (starting at the lowest gear)
int actuatorPositionMicros = actuatorMinMicros; // Current actuator position in microseconds
int startingPositionMicros = actuatorMaxMicros; // Position after pre-tensioning

// Timing Variables
unsigned long gearChangeStartTime = 0;         // Time when speed entered a new range
int previousGear = totalGears;                 // Last evaluated gear for stability check
unsigned long buttonPressStartTime = 0;        // Start time for button press

// Modes
bool isSetupMode = true;                       // Start in Setup Mode by default

// Function prototypes
void setLED(int red, int green, int blue);
void blinkLED(int red, int green, int blue, int times, int delayMs);
void checkButtonPress();
void calibrateActuator();
void calculateSpeed();
float calculateCadence();
void autoShiftGears(unsigned long currentTime);
void shiftUp();
void shiftDown();

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging

  // Attach the actuator servo to the servo pin
  actuator.attach(servoPin);

  // Configure button pins with internal pull-up resistors
  pinMode(shiftUpButtonPin, INPUT_PULLUP);
  pinMode(shiftDownButtonPin, INPUT_PULLUP);
  pinMode(calibrateButtonPin, INPUT_PULLUP);
  pinMode(sensorPin, INPUT_PULLUP); // Set up Hall sensor

  // Configure RGB LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Attach an interrupt for the Hall sensor
  attachInterrupt(digitalPinToInterrupt(sensorPin), calculateSpeed, FALLING);

  // Start in Setup Mode
  setLED(255, 0, 0); // Red LED for Setup Mode
  calibrateActuator(); // Ensure actuator is in calibration position
  Serial.println("Starting in Setup Mode.");
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time

  // Handle button presses for mode switching and status checks
  checkButtonPress();

  if (isSetupMode) {
    // Setup Mode: Actuator stays in calibration position
    setLED(255, 0, 0); // Red LED for Setup Mode
  } else {
    // Auto Shift Mode
    setLED(0, 0, 0); // Turn off LED during normal operation

    // Set speed to 0 if no pulse for more than 2 seconds
    if (currentTime - lastPulseTime / 1000 > 2000) {
      speed = 0;
    }

    // Calculate cadence
    float cadence = calculateCadence();

    // Automatically shift gears based on speed
    autoShiftGears(currentTime);

    // Output speed, cadence, and gear for debugging
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print(" km/h, Cadence: ");
    Serial.print(cadence);
    Serial.print(" RPM, Gear: ");
    Serial.println(currentGear);

    delay(500);
  }
}

// ISR for speed calculation
void calculateSpeed() {
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime > debounceTime) {
    pulseInterval = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
    if (pulseInterval > 0) {
      speed = 3.6 * (wheelCircumference / (pulseInterval / 1e6));
    }
  }
}

// Function to calculate cadence based on speed and gear ratio
float calculateCadence() {
  if (speed == 0) return 0;

  // Calculate wheel RPM
  float wheelRPM = (speed * 1000) / (60 * wheelCircumference) / 3.6;

  // Calculate gear ratio based on current gear
  float gearRatio = (float)chainringTeeth / cassetteTeeth[currentGear - 1];

  // Calculate and return cadence
  return wheelRPM * gearRatio;
}

// Function to automatically shift gears based on speed
void autoShiftGears(unsigned long currentTime) {
  int newGear;

  if (speed >= maxSpeed) {
    newGear = totalGears; // Above maxSpeed, highest gear
  } else {
    newGear = map(speed, 0, maxSpeed, 1, totalGears);
  }

  newGear = constrain(newGear, 1, totalGears);

  if (newGear != currentGear) {
    if (newGear > currentGear) {
      shiftDown();
    } else if (newGear < currentGear) {
      shiftUp();
    }
  }
}

void shiftUp() {
  if (currentGear > 1) {
    currentGear--;
    actuatorPositionMicros -= pullFactor_micros;
    actuator.writeMicroseconds(actuatorPositionMicros);
    Serial.print("Shifted down to gear ");
    Serial.println(currentGear);
  }
}

void shiftDown() {
  if (currentGear < totalGears) {
    currentGear++;
    actuatorPositionMicros += pullFactor_micros;
    actuator.writeMicroseconds(actuatorPositionMicros);
    Serial.print("Shifted up to gear ");
    Serial.println(currentGear);
  }
}

// Calibration routine to initialize the actuator
void calibrateActuator() {
  actuatorPositionMicros = actuatorMaxMicros;
  actuator.writeMicroseconds(actuatorPositionMicros);
  Serial.println("Calibrating actuator to maximum position.");
}

void checkButtonPress() {
  int buttonState = digitalRead(calibrateButtonPin);
  if (buttonState == LOW) {
    if (buttonPressStartTime == 0) {
      buttonPressStartTime = millis(); // Start timing
    } else if (millis() - buttonPressStartTime > 3000) {
      // Long press: Toggle mode
      isSetupMode = !isSetupMode;
      buttonPressStartTime = 0; // Reset timing

      if (isSetupMode) {
        setLED(255, 0, 0); // Red LED for Setup Mode
        calibrateActuator();
        Serial.println("Entered Setup Mode");
      } else {
        blinkLED(0, 255, 0, 3, 300); // Blink green 3 times
        Serial.println("Entered Auto Shift Mode");
      }
    }
  } else if (buttonPressStartTime > 0) {
    // Short press: Status check
    if (millis() - buttonPressStartTime < 3000) {
      if (isSetupMode) {
        blinkLED(255, 0, 0, 3, 300); // Blink red 3 times
        Serial.println("Status: Setup Mode");
      } else {
        blinkLED(0, 255, 0, 3, 300); // Blink green 3 times
        Serial.println("Status: Auto Shift Mode");
      }
    }
    buttonPressStartTime = 0; // Reset timing
  }
}

// Function to set the RGB LED color
void setLED(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

// Function to blink the RGB LED
void blinkLED(int red, int green, int blue, int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    setLED(red, green, blue);
    delay(delayMs);
    setLED(0, 0, 0);
    delay(delayMs);
  }
}
