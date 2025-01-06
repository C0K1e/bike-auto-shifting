#include <Servo.h> // Include the Servo library
#include <float.h> // Für FLT_MAX
#include <Wire.h> // Für Gyor Auslesen

// Create a Servo object for the actuator
Servo actuator;

// Pin Definitions
const int servoPin = 9;             // Servo control pin connected to pin 9
const int shiftUpButtonPin = 11;    // Shift up button connected to pin 5
const int shiftDownButtonPin = 12;  // Shift down button connected to pin 3
const int calibrateButtonPin = 2;  // Calibration button connected to pin 4
const int sensorPin = 3;            // Hall sensor connected to pin 3
const int analogPinBattery = A0;           // 1) Pin für Spannungsmessung

// RGB LED Pins
const int redPin = 4;    // Red LED pin
const int greenPin = 5;  // Green LED pin
const int bluePin = 6;   // Blue LED pin

// Actuator Specifications
const float actuatorStroke_mm = 50.0;  // Total stroke length in mm
const int actuatorMaxMicros = 2000;    // Maximum servo pulse width in microseconds
const int actuatorMinMicros = 1000;    // Minimum servo pulse width in microseconds
const float microsPerMm = (actuatorMaxMicros - actuatorMinMicros) / actuatorStroke_mm; // Microseconds per mm

// Configurable Parameters
int chainringTeeth = 42;            // Number of teeth on the chainring (adjustable)
int cassetteTeeth[] = {11, 13, 15, 17, 19, 22, 25, 28, 32, 36}; // Teeth on cassette sprockets
int totalGears = sizeof(cassetteTeeth) / sizeof(cassetteTeeth[0]);  // Total number of gears
float maxSpeed = 40.0;              // Maximum speed for the highest gear (km/h)
float pullFactor_mm = 2.8;          // Cable pull per gear shift in mm
float pullFactor_micros = pullFactor_mm * microsPerMm; // Microseconds per gear shift
float wheelCircumference = 2.6;    // Wheel circumference in meters
float optimalCadence = 85;           // Optimal Cadence Setup
float criticalBattery = 20.0;         // Percentage for critical Battery

// Speed and Hall Sensor Variables
volatile unsigned long lastPulseTime = 0; // Time of the last pulse
volatile unsigned long pulseInterval = 0; // Time between pulses
const unsigned long debounceTime = 100000; // Debounce time in microseconds (100 ms)
float speed = 0;                          // Speed in km/h
float wheelRPM = 0;                       // Wheel RPM global

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

//Gyro Variables
float tiltPerc;

// MPU-6050 I2C address
const int MPU_ADDR = 0x68;

// Variables to store sensor data
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// Calibration constants
const float accelScale = 2.0 / 32768.0; // For ±2g range
const float gyroScale = 250.0 / 32768.0; // For ±250°/s range

// Variables to track the tilt angle
float angleX = 0;         // Integrated angle in degrees
float baseAngle = 0;      // Reference angle (calibrated to 0%)
float gyroDrift = 0;      // Measured drift of the gyroscope
unsigned long prevTime;   // For delta time calculation

// Constants for tilt scaling
const float maxTiltDegrees = 45.0; // 100% corresponds to 45 degrees
const float alpha = 0.95;          // Complementary filter constant (increased sensitivity)

// Battery variables
float batteryLevel;
// 2) Widerstände deines Teilers
const float R1 = 10000.0;  // 10k (oben)
const float R2 = 15000.0;  // 15k (unten)
// 3) Minimal- und Maximalspannung deines 2S-LiPo (anpassen nach Bedarf)
const float minVoltage = 6.0;   // z.B. 6.0 V = 3.0 V pro Zelle (fast leer)
const float maxVoltage = 8.4;   // 8.4 V = 4.2 V pro Zelle (voll)


// Function prototypes
void setLED(int red, int green, int blue);
void blinkLED(int red, int green, int blue, int times, int delayMs);
void checkSetupButtonPress();
void calibrateActuator();
void calculateSpeed();
float calculateCadence();
int getOptimalGear(float wheelSpeed, float slopePercent, float desiredCadence);
void autoShiftGears(unsigned long currentTime);
void shiftDown();
void shiftUp();
void calibrateZero();
float getCurrentTiltPercentage();
float getBatteryPercentage();

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging

  // Attach the actuator servo to the servo pin
  actuator.attach(servoPin);

  // Configure button pins with internal pull-up resistors
  pinMode(shiftUpButtonPin, INPUT_PULLUP);
  pinMode(shiftDownButtonPin, INPUT_PULLUP);
  pinMode(calibrateButtonPin, INPUT_PULLUP);
  pinMode(sensorPin, INPUT_PULLUP); // Set up Hall sensor
  pinMode(analogPinBattery, INPUT); //Set up BatteryLevel Pin // ggf löschen da analog auslesen

  // Configure RGB LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Attach an interrupt for the Hall sensor
  attachInterrupt(digitalPinToInterrupt(sensorPin), calculateSpeed, FALLING);

  // Start in Setup Mode
  setLED(255, 0, 0); // Red LED for Setup Mode
  calibrateZero(); // Calibrate Gyro to Zero Leveling
  calibrateActuator(); // Ensure actuator is in calibration position
  Serial.println("Starting in Setup Mode.");
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time
  unsigned long lastBlinkMillis = 0; // Globale Variable

  //Akkustand Auslesen
  batteryLevel = getBatteryPercentage();
  
  // Handle button presses for mode switching and status checks
  checkSetupButtonPress();

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

  if (batteryLevel < criticalBattery) {
    unsigned long currentMillis = millis();

    // Prüfen, ob seit dem letzten Blinken >= 2000ms vergangen sind
    if (currentMillis - lastBlinkMillis >= 2000) {
      lastBlinkMillis = currentMillis;

      // Farbton für Orange/gelb-orange
      // Beispiel: (255, 140, 0) oder (255, 100, 0)
      // Um "soft" orange zu bekommen, kann man rot=255, grün=130..150, blau=0 wählen
      blinkLED(255, 130, 0, 1, 500);
      // => blinkLED(..., 1, 500) bedeutet:
      //    - 500 ms an
      //    - 500 ms aus
      // => insgesamt 1000 ms. Danach geht loop() weiter.
    }

  } 

    // Calculate cadence
    float cadence = calculateCadence();

    // Get Tilt Angle
    float tiltPerc = getCurrentTiltPercentage();  // -100..+100%

    // Automatically shift gears based on speed
    autoShiftGears(currentTime);

    // Output speed, cadence, and gear for debugging
    Serial.print("Batterie-Ladestand: ");
    Serial.print(batteryLevel, 1);
    Serial.println("%");

    Serial.print("Speed: ");
    Serial.print(speed);

    Serial.print(" km/h, Cadence: ");
    Serial.print(cadence);

    Serial.print("Aktuelle Neigung: ");
    Serial.print(tiltPerc, 2);
    Serial.println(" %");

    Serial.print(" RPM, Gear: ");
    Serial.println(currentGear);

    delay(1000);
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
  wheelRPM = (speed * 1000) / (60 * wheelCircumference) / 3.6;

  // Calculate gear ratio based on current gear
  float gearRatio = (float)chainringTeeth / cassetteTeeth[currentGear - 1];

  // Calculate and return cadence
  return wheelRPM * gearRatio;
}

// Funktion, um den optimalen Gang basierend auf Drehgeschwindigkeit und gewünschter Trittfrequenz zu berechnen
int getOptimalGear(float wheelSpeed, float slopePercent, float desiredCadence)
{
    // 1) Steigung auf ±15% begrenzen
    if (slopePercent > 15.0) {
        slopePercent = 15.0;
    } else if (slopePercent < -15.0) {
        slopePercent = -15.0;
    }

    // 2) Trittfrequenz an Steigung anpassen
    //    Beispiel: +15% Steigung => +15 RPM, -15% Steigung => -15 RPM
    float adjustedCadence = desiredCadence + slopePercent;

    // Schutz gegen zu niedrige (oder negative) Trittfrequenz
    if (adjustedCadence < 1.0) {
        adjustedCadence = 1.0;
    }

    // 3) Ungültige Eingaben (z. B. Stillstand) abfangen
    if (wheelSpeed <= 0 || adjustedCadence <= 0) {
        Serial.println("Ungültige Eingaben oder Stillstand.");
        return currentGear;
    }

    // 4) Optimalen Gang finden
    float optimalGearRatio = wheelSpeed / adjustedCadence;
    int optimalGear = currentGear;
    float smallestDelta = FLT_MAX;  // braucht #include <float.h>

    // Schleife über alle verfügbaren Gänge
    for (int i = 1; i <= totalGears; i++) {
        float gearRatio = (float)chainringTeeth / cassetteTeeth[i - 1];
        float delta = fabs(optimalGearRatio - gearRatio); // braucht #include <math.h>
        if (delta < smallestDelta) {
            smallestDelta = delta;
            optimalGear = i;
        }
    }

    // Debug-Ausgabe
    Serial.print(" => Optimaler Gang: ");
    Serial.println(optimalGear);

  // Ausgabe des optimalen Gangs und des Deltas für Debugging
  Serial.print("Optimaler Gang: ");
  Serial.println(optimalGear);
 
  return optimalGear;
}

// Function to automatically shift gears based on speed
void autoShiftGears(unsigned long currentTime) {
  int newGear;

  newGear = getOptimalGear(wheelRPM,tiltPerc,optimalCadence);

  if (newGear != currentGear) {
    if (newGear > currentGear) {
      shiftUp();
    } else if (newGear < currentGear) {
      shiftDown();
    }
  }
}

void shiftDown() {
  if (currentGear > 1) {
    currentGear--;
    actuatorPositionMicros -= pullFactor_micros;
    actuator.writeMicroseconds(actuatorPositionMicros);
    Serial.print("Shifted down to gear ");
    Serial.println(currentGear);
  }
}

void shiftUp() {
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

void checkSetupButtonPress() {
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


// Function to calibrate the sensor to set zero position and gyro drift
void calibrateZero() {
  Serial.println("Calibrating Zero-Lage and Gyro Drift... Please keep the sensor steady.");
  float sumAccelAngle = 0;
  float sumGyro = 0;
  const int sampleCount = 200; // More samples for better drift measurement

  // Average multiple readings to find the initial tilt angle and gyro drift
  for (int i = 0; i < sampleCount; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (size_t)14, true); // 14 Bytes (Accel + Gyro)

    // Read accelerometer data
    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();

    // Read gyroscope data
    gyroX = Wire.read() << 8 | Wire.read();
    Wire.read(); // Skip gyroY and gyroZ

    // Convert raw accelerometer values to g
    float ax = accelX * accelScale;
    float ay = accelY * accelScale;
    float az = accelZ * accelScale;

    // Calculate tilt angle from accelerometer
    float accelAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180 / PI;

    // Accumulate tilt angle and gyroscope drift
    sumAccelAngle += accelAngle;
    sumGyro += gyroX * gyroScale;

    delay(5); // Faster sampling for sensitivity
  }

  // Calculate average base angle and gyro drift
  baseAngle = sumAccelAngle / sampleCount;
  gyroDrift = sumGyro / sampleCount;

  // Initialize gyroscope angle to match the base angle
  angleX = baseAngle;

  Serial.print("Calibration complete. Zero angle set to: ");
  Serial.println(baseAngle, 2);
  Serial.print("Gyro drift compensated: ");
  Serial.println(gyroDrift, 2);

  prevTime = millis(); // Initialize time tracking
}


// Function zur Neigngsauslesung
/**
 * @brief  Liest die aktuelle Neigung aus dem MPU-6050, wendet den Complementary Filter an
 *         und gibt eine Neigung in Prozent (±100%) zurück.
 *
 *   - Vor der ersten Nutzung unbedingt einmal calibrateZero() in setup() aufrufen!
 *   - 45° Neigung entsprechen ±100% (intern).
 *   - Werte jenseits ±45° werden gekappt, sodass das Ergebnis immer im Bereich -100..+100 liegt.
 *
 * @return float  Der aktuelle Tilt in Prozent, -100..+100
 */

float getCurrentTiltPercentage()
{
  // 1) Sensordaten anfordern
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (size_t)14, true); // 14 Bytes (Accel + Gyro)

  // 2) Beschleunigungswerte einlesen
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  // 3) Gyro-Werte einlesen
  gyroX  = Wire.read() << 8 | Wire.read();
  gyroY  = Wire.read() << 8 | Wire.read();
  gyroZ  = Wire.read() << 8 | Wire.read();

  // 4) Skalieren in "g" bzw. °/s
  float ax = accelX * accelScale;              // z.B. ±2 g
  float ay = accelY * accelScale;
  float az = accelZ * accelScale;
  float gx = (gyroX * gyroScale) - gyroDrift;  // in °/s, Drift subtrahiert

  // 5) Zeitdifferenz berechnen für Gyro-Integration
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // 6) Winkel aus Beschleunigung (Pitch) berechnen
  //    atan2(X-Achse, YZ-Kombination)
  float accelAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // 7) Gyro-Integration
  angleX += gx * deltaTime; // integriert den Gyro-Wert über die Zeit

  // 8) Complementary Filter
  //    alpha = 0.95 (z.B.) mischt Gyro- und Beschleunigungswerte
  angleX = alpha * angleX + (1.0 - alpha) * accelAngle;

  // 9) Relativer Winkel zur Baseline + Kappen auf ±45°
  float relativeAngle = angleX - baseAngle;
  if (relativeAngle >  maxTiltDegrees) relativeAngle =  maxTiltDegrees;
  if (relativeAngle < -maxTiltDegrees) relativeAngle = -maxTiltDegrees;

  // 10) Aus dem Winkel => "Tilt-Percentage" (±100% bei ±45°)
  float tiltPercentage = (relativeAngle / maxTiltDegrees) * 100.0;

  // 11) Rückgabe: Nur ±100% (durch das Kappen oben haben wir das schon sichergestellt)
  return tiltPercentage;
}


// Akkustand auslesen
/**
 * @brief   Misst die Akkuspannung (2S-LiPo) über einen Spannungsteiler
 *          (R1=10k, R2=15k) und gibt grob 0..100% Kapazität zurück.
 *
 * Beschaltung:
 *   Akku+ ---> [ R1=10k ] --->+---> (A0 / ADC)
 *                             |
 *                             [ R2=15k ]
 *                             |
 *                            GND
 *
 * WARNUNG:
 *   Bei vollem 2S-LiPo (8,4 V) -> ADC-Spannung ~ 8,4 * (15/25) = 5,04 V
 *   Das überschreitet die 5,0 V-Grenze des Arduino-ADC minimal.
 */
float getBatteryPercentage()
{
    // --- ADC auslesen ---
    int rawValue = analogRead(analogPinBattery);          // [0..1023]
    float voltageAtPin = (rawValue / 1023.0) * 5.0; // 0..5.0 V

    // --- Teilerfaktor (Achtung: R2 ist unten) ---
    float dividerFactor = R2 / (R1 + R2); // = 15000 / (10000 + 15000) = 0.6
    // => Akku-Spannung = SpannungAmPin / dividerFactor
    float batteryVoltage = voltageAtPin / dividerFactor;
    // Für 8.4 V Akku -> voltageAtPin ~ 5.04 V (drüber!)

    // --- Lineare Schätzung des Ladezustands ---
    float percentage = (batteryVoltage - minVoltage)
                     / (maxVoltage - minVoltage) * 100.0;

    // --- Begrenzen auf 0..100% ---
    if (percentage < 0.0)   percentage = 0.0;
    if (percentage > 100.0) percentage = 100.0;

    /*
    // (Optional) Debug-Ausgabe:
    Serial.print("ADC raw: ");
    Serial.print(rawValue);
    Serial.print(" => PinVoltage: ");
    Serial.print(voltageAtPin, 2);
    
    Serial.print("V => BatteryVoltage: ");
    Serial.print(batteryVoltage, 2);
    Serial.print("V => Capacity: ");
    Serial.print(percentage, 1);
    Serial.println("%");
    */

    return percentage;
}