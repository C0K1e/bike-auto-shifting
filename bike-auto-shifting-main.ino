// ---------------------------------------------------------------------
// AUTOMATIC ELECTRICAL BICYCLE SHIFTING CODE
// 
// Copyight by
// Jasper Bartel
// Moritz Bühl
// Conrad Kieselberger
// Chat ;)
// ---------------------------------------------------------------------

#include <Servo.h>   // Include the Servo library
#include <float.h>   // Für FLT_MAX
#include <Wire.h>    // Für Gyro-Auslesen
#include <math.h>    // Für fabs()

// ---------------------------------------------------------------------
// SERVO & HARDWARE
// ---------------------------------------------------------------------
Servo actuator;

// Pin Definitions
const int servoPin          = 9;   // Servo control pin
const int shiftUpButtonPin  = 11;  // Shift up button
const int shiftDownButtonPin= 12;  // Shift down button
const int calibrateButtonPin= 2;   // Calibration button
const int sensorPin         = 3;   // Hall sensor
const int analogPinBattery  = A0;  // Pin für Spannungsmessung

// RGB LED Pins
const int redPin   = 4;
const int greenPin = 5;
const int bluePin  = 6;

// ---------------------------------------------------------------------
// AKTUATOR & GANG-KONFIG
// ---------------------------------------------------------------------
const float actuatorStroke_mm  = 50.0;   // Total stroke length in mm
const int   actuatorMaxMicros  = 2000;   // Maximum servo pulse width in microseconds
const int   actuatorMinMicros  = 1000;   // Minimum servo pulse width in microseconds
const float microsPerMm        = (actuatorMaxMicros - actuatorMinMicros) / actuatorStroke_mm; 

// ---------------------------------------------------------------------
// KASSETTE MIT INDIVIDUELLEN PULL-FAKTOREN
// ---------------------------------------------------------------------
struct Gear {
  int teeth;
  float pullFactor;  // individueller Pull-Faktor in mm für diesen Gang
};

// Ritzelzahlen aufsteigend, Pull-Faktor-Apex 1 11-Speed Shifter
// nach https://drivetrainbuilder.com/SB-APX-B1.htm
Gear cassette[] = {
  {11, 3.75},
  {13, 2.80},
  {15, 2.78},
  {17, 2.80},
  {19, 2.82},
  {22, 2.84},
  {25, 2.95},
  {28, 3.03},
  {32, 3.03},
  {36, 3.27},
  {42, 0.00} // Kein Pull-Faktor für Gang 11 notwendig
};

int totalGears = sizeof(cassette) / sizeof(cassette[0]);

// Hilfsfunktionen für Gang <-> Array-Index
int cassetteIndexForGear(int gear) {
  // Gang 1 => Index=10 (42 Zähne), ... Gang 11 => Index=0 (11 Zähne)
  int idx = totalGears - gear;
  if (idx < 0) idx = 0;
  if (idx >= totalGears) idx = totalGears - 1;
  return idx;
}

int getCassetteTeeth(int gear) {
  return cassette[cassetteIndexForGear(gear)].teeth;
}

float getCassettePullFactor(int gear) {
  return cassette[cassetteIndexForGear(gear)].pullFactor;
}

// ---------------------------------------------------------------------
// PARAMETER
// ---------------------------------------------------------------------
float maxSpeed          = 40.0;
float wheelCircumference= 2.6;
float optimalCadence    = 85;
float criticalBattery   = 20.0;

const int maxGearJump      = 3;
const int startGearAuto    = 2;

// ---------------------------------------------------------------------
// SPEED & HALL SENSOR
// ---------------------------------------------------------------------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
const unsigned long debounceTime      = 100000;
float speed = 0;
float wheelRPM= 0;

// ---------------------------------------------------------------------
// AKTUATOR-STATE
// ---------------------------------------------------------------------
int currentGear            = totalGears;
int actuatorPositionMicros = actuatorMinMicros;
int startingPositionMicros = actuatorMaxMicros;

// ---------------------------------------------------------------------
// TIMING & MODES
// ---------------------------------------------------------------------
unsigned long gearChangeStartTime = 0;
int previousGear           = totalGears;
unsigned long buttonPressStartTime = 0;
bool isSetupMode = true;

// Battery Button Check
unsigned long lastButtonPressTime = 0;   // Zeit der letzten Tastenbetätigung
unsigned long doubleClickInterval = 500; // Max. Zeit in ms zwischen zwei Klicks für Doppelklick
bool isWaitingForSecondClick = false;    // Status, ob ein zweiter Klick erwartet wird


// ---------------------------------------------------------------------
// GYRO
// ---------------------------------------------------------------------
float tiltPerc;
const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ;
int16_t gyroX,  gyroY,  gyroZ;
const float accelScale = 2.0 / 32768.0;
const float gyroScale  = 250.0 / 32768.0;
float angleX    = 0;
float baseAngle = 0;
float gyroDrift = 0;
unsigned long prevTime;
const float maxTiltDegrees = 45.0; 
const float alpha          = 0.95;

// ---------------------------------------------------------------------
// BATTERY
// ---------------------------------------------------------------------
float batteryLevel;
const float R1 = 10000.0;
const float R2 = 15000.0;
const float minVoltage = 6.0;
const float maxVoltage = 8.4;


// ---------------------------------------------------------------------
// FUNKTIONSPROTOTYPEN
// ---------------------------------------------------------------------
void setLED(int red, int green, int blue);
void blinkLED(int red, int green, int blue, int times, int delayMs);
void checkSetupButtonPress();
void checkBatteryStatus();
void calibrateActuator();
void calculateSpeed();
float calculateCadence();
int   getOptimalGear(float wheelSpeed, float slopePercent, float desiredCadence);
void  autoShiftGears();
void  shiftDown();
void  shiftUp();
void  calibrateZero();
float getCurrentTiltPercentage();
float getBatteryPercentage();

// ---------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(9600); 
  actuator.attach(servoPin);

  pinMode(shiftUpButtonPin,   INPUT_PULLUP);
  pinMode(shiftDownButtonPin, INPUT_PULLUP);
  pinMode(calibrateButtonPin, INPUT_PULLUP);
  pinMode(sensorPin,          INPUT_PULLUP);
  pinMode(analogPinBattery,   INPUT);

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  attachInterrupt(digitalPinToInterrupt(sensorPin), calculateSpeed, FALLING);

  setLED(255, 0, 0); 
  calibrateZero();       
  calibrateActuator();   
  Serial.println("Starting in Setup Mode.");

  currentGear = totalGears;  
  actuatorPositionMicros = actuatorMinMicros + (getCassettePullFactor(currentGear) * microsPerMm * (currentGear - 1));
  actuator.writeMicroseconds(actuatorPositionMicros);
  Serial.println("SetupMode: High gear forced (Gang 11, kleinstes Ritzel).");
}

// ---------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------
void loop() {
  unsigned long currentTime = millis();
  static unsigned long lastBlinkMillis = 0;

  batteryLevel = getBatteryPercentage();
  checkSetupButtonPress();

  if (isSetupMode) {
    setLED(255, 0, 0);
  } 
  else {
    setLED(0, 0, 0);

    if ((currentTime - (lastPulseTime / 1000)) > 2000) {
      speed = 0;
    }

    if (batteryLevel < criticalBattery) {
      if (currentTime - lastBlinkMillis >= 2000) {
        lastBlinkMillis = currentTime;
        blinkLED(255, 130, 0, 1, 500);
      }
    }

    float cadence = calculateCadence();
    tiltPerc = getCurrentTiltPercentage();

    autoShiftGears();

    Serial.print("Batterie-Ladestand: ");
    Serial.print(batteryLevel, 1);
    Serial.println("%");

    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print(" km/h, Cadence: ");
    Serial.print(cadence);
    Serial.print(" RPM, Neigung: ");
    Serial.print(tiltPerc, 2);
    Serial.print(" %, Gear: ");
    Serial.println(currentGear);

    delay(1000);
  }
}

// ---------------------------------------------------------------------
// ISR: Geschwindigkeitsberechnung
// ---------------------------------------------------------------------
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

// ---------------------------------------------------------------------
// Trittfrequenz
// ---------------------------------------------------------------------
float calculateCadence() {
  if (speed == 0) return 0.0;
  wheelRPM = (speed * 1000.0) / (60.0 * wheelCircumference) / 3.6;
  float gearRatio = (float)chainringTeeth / (float)getCassetteTeeth(currentGear);
  return wheelRPM * gearRatio;
}

// ---------------------------------------------------------------------
// Optimaler Gang
// ---------------------------------------------------------------------
int getOptimalGear(float wheelSpeed, float slopePercent, float desiredCadence) {
  if (slopePercent > 15.0) slopePercent = 15.0;
  if (slopePercent < -15.0) slopePercent = -15.0;

  float adjustedCadence = desiredCadence + slopePercent;
  if (adjustedCadence < 1.0) adjustedCadence = 1.0;

  if (wheelSpeed <= 0 || adjustedCadence <= 0) {
    Serial.println("Ungültige Eingaben oder Stillstand.");
    return currentGear;
  }

  float optimalGearRatio = wheelSpeed / adjustedCadence;
  float smallestDelta     = FLT_MAX;
  int   optimalGear       = currentGear;

  for (int g = 1; g <= totalGears; g++) {
    float gearRatio = (float)chainringTeeth / (float)getCassetteTeeth(g);
    float delta     = fabs(optimalGearRatio - gearRatio);
    if (delta < smallestDelta) {
      smallestDelta  = delta;
      optimalGear    = g;
    }
  }

  Serial.print(" => Optimaler Gang: ");
  Serial.println(optimalGear);
  return optimalGear;
}

// ---------------------------------------------------------------------
// MEHRERE GÄNGE IN EINEM SCHRITT
// ---------------------------------------------------------------------
void autoShiftGears() {
  int newGear = getOptimalGear(wheelRPM, tiltPerc, optimalCadence);
  if (newGear == currentGear) return;

  int diff = newGear - currentGear;
  if (abs(diff) > maxGearJump) {
    diff = (diff > 0) ? maxGearJump : -maxGearJump;
  }

  if (diff > 0) {
    for (int i = 0; i < diff; i++) {
      shiftUp();
    }
  } else {
    for (int i = 0; i < abs(diff); i++) {
      shiftDown();
    }
  }
}

// ---------------------------------------------------------------------
// SHIFT-FUNKTIONEN
// ---------------------------------------------------------------------
void shiftUp() {
  // Hochschalten: Verwende den Pull-Faktor des aktuellen Gangs,
  // bevor der Gang erhöht wird.
  if (currentGear < totalGears) {
    float pf = getCassettePullFactor(currentGear);  // Pull-Faktor des aktuellen Gangs
    currentGear++;  // Gang erhöhen
    actuatorPositionMicros += pf * microsPerMm;
    actuator.writeMicroseconds(actuatorPositionMicros);
    Serial.print("Shifted up to gear ");
    Serial.println(currentGear);
  }
  // Wenn currentGear == totalGears (Gang 11), passiert nichts.
}

void shiftDown() {
  // Runterschalten: Verwende den Pull-Faktor des Gangs unterhalb des aktuellen.
  if (currentGear > 1) {
    int targetGear = currentGear - 1;              // Neuer (niedrigerer) Gang
    float pf = getCassettePullFactor(targetGear);  // Pull-Faktor des darunterliegenden Gangs
    currentGear--;  // Gang verringern
    actuatorPositionMicros -= pf * microsPerMm;
    actuator.writeMicroseconds(actuatorPositionMicros);
    Serial.print("Shifted down to gear ");
    Serial.println(currentGear);
  }
  // Wenn currentGear == 1, passiert nichts, da kein Gang darunter existiert.
}

// ---------------------------------------------------------------------
// SETUP-MODE: Kalibrierung
// ---------------------------------------------------------------------
void calibrateActuator() {
  actuatorPositionMicros = actuatorMaxMicros;
  actuator.writeMicroseconds(actuatorPositionMicros);
  Serial.println("Calibrating actuator to maximum position.");
}

// ---------------------------------------------------------------------
// CHECK BUTTON PRESS
// ---------------------------------------------------------------------
void checkSetupButtonPress() {
    int buttonState = digitalRead(calibrateButtonPin);

    if (buttonState == LOW) {
        unsigned long currentTime = millis();

        // Prüfen, ob ein Doppelklick erkannt wurde
        if (isWaitingForSecondClick && (currentTime - lastButtonPressTime <= doubleClickInterval)) {
            isWaitingForSecondClick = false;  // Zweiter Klick erkannt
            Serial.println("Doppelklick erkannt: Akkustand wird überprüft.");
            checkBatteryStatus();             // Akkustatus prüfen und LED-Signal ausgeben
        } else {
            // Erstes Drücken, Doppelklick starten
            isWaitingForSecondClick = true;
            lastButtonPressTime = currentTime;
        }
    } else if (isWaitingForSecondClick) {
        // Prüfen, ob Zeit für den zweiten Klick abgelaufen ist
        if (millis() - lastButtonPressTime > doubleClickInterval) {
            isWaitingForSecondClick = false; // Warten beenden
        }
    }

    // Langer Klick für Moduswechsel
    if (buttonState == LOW && buttonPressStartTime == 0) {
        buttonPressStartTime = millis();
    } else if (buttonState == LOW && millis() - buttonPressStartTime > 3000) {
        isSetupMode = !isSetupMode;
        buttonPressStartTime = 0;

        if (isSetupMode) {
            setLED(255, 0, 0);  // Setup-Mode LED
            calibrateActuator();
            Serial.println("Entered Setup Mode");

            currentGear = totalGears; 
            actuatorPositionMicros = actuatorMinMicros + (getCassettePullFactor(currentGear) * microsPerMm * (currentGear - 1));
            actuator.writeMicroseconds(actuatorPositionMicros);
            Serial.println("SetupMode: Forced highest gear (11).");
        } else {
            blinkLED(0, 255, 0, 3, 300);  // Auto-Mode LED
            Serial.println("Entered Auto Shift Mode");

            if (startGearAuto < 1)  return; 
            if (startGearAuto > totalGears) return;

            int diff = startGearAuto - currentGear;
            if (diff > 0) {
                for (int i = 0; i < diff; i++) shiftUp();
            } else if (diff < 0) {
                for (int i = 0; i < abs(diff); i++) shiftDown();
            }
        }
    } else if (buttonState == HIGH && buttonPressStartTime > 0) {
        if (millis() - buttonPressStartTime < 3000) {
            if (isSetupMode) {
                blinkLED(255, 0, 0, 3, 300);  // Setup Mode Status
                Serial.println("Status: Setup Mode");
            } else {
                blinkLED(0, 255, 0, 3, 300);  // Auto Shift Status
                Serial.println("Status: Auto Shift Mode");
            }
        }
        buttonPressStartTime = 0;
    }
}

// ---------------------------------------------------------------------
// LED
// ---------------------------------------------------------------------
void setLED(int red, int green, int blue) {
  analogWrite(redPin,   red);
  analogWrite(greenPin, green);
  analogWrite(bluePin,  blue);
}

void blinkLED(int red, int green, int blue, int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    setLED(red, green, blue);
    delay(delayMs);
    setLED(0, 0, 0);
    delay(delayMs);
  }
}

// ---------------------------------------------------------------------
// GYRO-Kalibrierung
// ---------------------------------------------------------------------
void calibrateZero() {
  Serial.println("Calibrating Zero-Lage and Gyro Drift... Keep sensor steady.");
  float sumAccelAngle = 0;
  float sumGyro       = 0;
  const int sampleCount = 200;

  for (int i = 0; i < sampleCount; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();

    gyroX  = Wire.read() << 8 | Wire.read();
    Wire.read(); 
    Wire.read(); 

    float ax = accelX * accelScale;
    float ay = accelY * accelScale;
    float az = accelZ * accelScale;

    float accelAngle = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

    sumAccelAngle += accelAngle;
    sumGyro += gyroX * gyroScale;

    delay(5);
  }

  baseAngle = sumAccelAngle / sampleCount;
  gyroDrift = sumGyro / sampleCount;
  angleX    = baseAngle;

  Serial.print("Calibration complete. Zero angle set to: ");
  Serial.println(baseAngle, 2);
  Serial.print("Gyro drift compensated: ");
  Serial.println(gyroDrift, 2);

  prevTime = millis();
}

// ---------------------------------------------------------------------
// AKTUELLE NEIGUNG
// ---------------------------------------------------------------------
float getCurrentTiltPercentage() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  gyroX  = Wire.read() << 8 | Wire.read();
  gyroY  = Wire.read() << 8 | Wire.read();
  gyroZ  = Wire.read() << 8 | Wire.read();

  float ax = accelX * accelScale;
  float ay = accelY * accelScale;
  float az = accelZ * accelScale;
  float gx = (gyroX * gyroScale) - gyroDrift;

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  float accelAngle = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

  angleX += gx * deltaTime;
  angleX = alpha * angleX + (1.0 - alpha) * accelAngle;

  float relativeAngle = angleX - baseAngle;
  if (relativeAngle >  maxTiltDegrees) relativeAngle =  maxTiltDegrees;
  if (relativeAngle < -maxTiltDegrees) relativeAngle = -maxTiltDegrees;

  float tiltPercentage = (relativeAngle / maxTiltDegrees) * 100.0;
  return tiltPercentage;
}

// ---------------------------------------------------------------------
// AKKUSTAND
// ---------------------------------------------------------------------
float getBatteryPercentage() {
  int rawValue = analogRead(analogPinBattery); 
  float voltageAtPin = (rawValue / 1023.0) * 5.0;

  float dividerFactor = R2 / (R1 + R2); 
  float batteryVoltage = voltageAtPin / dividerFactor;

  float percentage = (batteryVoltage - minVoltage)
                   / (maxVoltage - minVoltage) * 100.0;

  if (percentage < 0.0)   percentage = 0.0;
  if (percentage > 100.0) percentage = 100.0;

  return percentage;
}

// ---------------------------------------------------------------------
// CHECK BATTERY STATUS
// ---------------------------------------------------------------------
void checkBatteryStatus() {
    float battery = getBatteryPercentage();

    Serial.print("Akkustand: ");
    Serial.print(battery, 1);
    Serial.println("%");

    if (battery >= 20.0 && battery < 45.0) {
        blinkLED(255, 130, 0, 1, 300); // 1x Orange blinken
    } else if (battery >= 45.0 && battery < 75.0) {
        blinkLED(255, 130, 0, 2, 300); // 2x Orange blinken
    } else if (battery >= 75.0 && battery <= 100.0) {
        blinkLED(255, 130, 0, 3, 300); // 3x Orange blinken
    } else if (battery < 20.0) {
        Serial.println("Kritischer Akkustand!"); // Keine Blinksignale unter 20 %
    }
}
