

// ---------------------------------------------------------------------
// AUTOMATIC ELECTRICAL BICYCLE SHIFTING CODE (Extended Debug Version)
//
// Copyright:
//   - Jasper Bartel
//   - Moritz Bühl
//   - Conrad Kieselberger
//   - Chat ;)
//
// Hinweis:
// Dieser Code enthält erweiterte Debug-Ausgaben. Bei Bedarf kannst du
// weitere Ausgaben hinzufügen oder entfernen.
// ---------------------------------------------------------------------


#include <Servo.h>
#include <float.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>  // Für EEPROM-Speicherung

// ---------------------------------------------------------------------
// SERVO & HARDWARE
// ---------------------------------------------------------------------
Servo actuator;

// Pin Definitions
const int servoPin           = 9;
const int shiftUpButtonPin   = 11;
const int shiftDownButtonPin = 12;
const int calibrateButtonPin = 2;
const int sensorPin          = 3;
const int analogPinBattery   = A0;

// RGB LED Pins
const int redPin   = 4;
const int greenPin = 5;
const int bluePin  = 6;

// ---------------------------------------------------------------------
// AKTUATOR & GANG-KONFIG
// ---------------------------------------------------------------------
const float actuatorStroke_mm = 46.0;
const int   actuatorMaxMicros = 2000;
const int   actuatorMinMicros = 1000;
const float microsPerMm       = (actuatorMaxMicros - actuatorMinMicros) / actuatorStroke_mm;

// ---------------------------------------------------------------------
// KASSETTE MIT INDIVIDUELLEN PULL-FAKTOREN
// ---------------------------------------------------------------------
const int chainringTeeth = 42;

struct Gear {
  int teeth;
  float pullFactor;
};

Gear cassette[] = {
  {11, 3.75},
  {13, 2.80},
  {15, 2.78},
  {17, 2.80},
  {19, 2.82},
  {22, 2.84},
  {25, 2.95},
  {28, 3.03},
  {32, 3.07},
  {36, 3.75},
  {42, 0.00}
};

int totalGears = sizeof(cassette) / sizeof(cassette[0]);

int cassetteIndexForGear(int gear) {
  int idx = totalGears - gear;
  if (idx < 0) idx = 0;
  if (idx >= totalGears) idx = totalGears - 1;
  return idx;
}

int getCassetteTeeth(int gear) {
  int idx = cassetteIndexForGear(gear);
  Serial.print("[getCassetteTeeth] Gear: ");
  Serial.print(gear);
  Serial.print(" => Index: ");
  Serial.print(idx);
  Serial.print(" => Teeth: ");
  Serial.println(cassette[idx].teeth);
  return cassette[idx].teeth;
}

float getCassettePullFactor(int gear) {
  int idx = cassetteIndexForGear(gear);
  Serial.print("[getCassettePullFactor] Gear: ");
  Serial.print(gear);
  Serial.print(" => Index: ");
  Serial.print(idx);
  Serial.print(" => Pull-Factor: ");
  Serial.println(cassette[idx].pullFactor, 3);
  return cassette[idx].pullFactor;
}

// ---------------------------------------------------------------------
// PARAMETER
// ---------------------------------------------------------------------
float maxSpeed           = 40.0;
float wheelCircumference = 2.2;  // in Metern
float optimalCadence     = 60.0; // RPM
float criticalBattery    = 20.0;

const int maxGearJump   = 3;
const int startGearAuto = 2;

// ---------------------------------------------------------------------
// SPEED & HALL SENSOR
// ---------------------------------------------------------------------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
const unsigned long debounceTime     = 100000; // 100 ms in Microsekunden
float speed    = 0;
float wheelRPM = 0;

// ---------------------------------------------------------------------
// AKTUATOR-STATE
// ---------------------------------------------------------------------
// --> Wir wollen letzten Gang aus EEPROM laden:
const int EEPROM_ADDR_LAST_GEAR = 8;  // int belegt 2 Byte (ggf. 4 Byte, hier aber Standard-Arduino => 2 Byte)
int currentGear            = 11;  
int actuatorPositionMicros = 1000;
int startingPositionMicros = 2000;

// ---------------------------------------------------------------------
// TIMING & MODES
// ---------------------------------------------------------------------
unsigned long gearChangeStartTime   = 0;
int previousGear                    = 11;
unsigned long buttonPressStartTime  = 0;
bool isSetupMode                    = false;  // Standard: Code startet im Setup-Mode

// Battery Button Check
unsigned long lastButtonPressTime   = 0;   
unsigned long doubleClickInterval   = 500;
bool isWaitingForSecondClick        = false;    

// ---------------------------------------------------------------------
// GYRO / NEIGUNG
// ---------------------------------------------------------------------
const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ;
int16_t gyroX,  gyroY,  gyroZ;

const float accelScale = 2.0 / 32768.0;
const float gyroScale  = 250.0 / 32768.0; 

float angleX      = 0;     // Akkumulierte Neigung (Pitch)
float gyroDrift   = 0;     // X-Gyro Drift
unsigned long prevTime;

const float maxTiltDegrees = 45.0; 
const float alpha          = 0.95; 

// ---------------------------------------------------------------------
// BATTERY
// ---------------------------------------------------------------------
float batteryLevel;
const float R1        = 10000.0;
const float R2        = 15000.0;
const float minVoltage = 6.0;
const float maxVoltage = 8.4;

// ---------------------------------------------------------------------
// FORWARD-KALIBRIERUNG
// ---------------------------------------------------------------------
// Wir speichern den "Forward-Winkel" direkt in Grad (nicht Bogenmaß),
// damit wir ihn in getCurrentTiltPercentage() einfach abziehen können.
const int EEPROM_ADDR_FORWARD_ANGLE = 0;   
float forwardCalibrationAngleDeg = 0.0;  // in Grad

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

// Null-Drift-Kalibrierung (Gyro):
void calibrateZero();

// Neigungsberechnung mit Forward-Korrektur:
float getCurrentTiltPercentage();

// Akku:
float getBatteryPercentage();
void  checkBatteryStatus();

// EEPROM-Helfer:
void storeEepromFloat(int address, float value);
float readEepromFloat(int address);
void storeEepromInt(int address, int value);
int   readEepromInt(int address);

// Forward-Kalibrierung:
void calibrateForwardDirection();

// Gang-Position setzen:
void setActuatorToGear(int gear);


// ---------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(9600); 
  Serial.println("=== STARTING AUTOMATIC ELECTRICAL BICYCLE SHIFTING CODE (DEBUG) ===");

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

  // 1) Letzten Forward-Winkel aus EEPROM laden
  forwardCalibrationAngleDeg = readEepromFloat(EEPROM_ADDR_FORWARD_ANGLE);
  Serial.print("[setup] EEPROM: forwardCalibrationAngleDeg = ");
  Serial.println(forwardCalibrationAngleDeg, 2);

  // 2) Letzten Gang aus EEPROM laden
  int lastGear = readEepromInt(EEPROM_ADDR_LAST_GEAR);
  if (lastGear < 1 || lastGear > totalGears) {
    lastGear = totalGears; // Fallback: höchster Gang
  }
  currentGear = lastGear;
  
  // 3) Aktuator direkt auf lastGear fahren
  setActuatorToGear(currentGear);
  Serial.print("[setup] Letzter Gang aus EEPROM: ");
  Serial.println(currentGear);

  // 4) LED-Test
  Serial.println("[setup] LED rot einschalten zur Prüfung.");
  setLED(255, 0, 0); 
  delay(500);
  setLED(0, 0, 0);

  // 5) Gyro-Null kalibrieren (Stillstand)
  Serial.println("[setup] Starte Gyro-Kalibrierung...");
  calibrateZero();       

  // 6) Aktuator-Kalibrierung (mechanisch in Max-Position)
  Serial.println("[setup] Starte Aktuator-Kalibrierung...");
  calibrateActuator();

  // 7) Setup-Mode aktiv
  isSetupMode = true;
  setLED(255, 0, 0); // Rot = Setup

  // 8) Im Setup-Modus erzwingen wir Gang 11 (falls gewünscht)
  //    => Das überschreibt den zuvor gelesenen Gang aus EEPROM!
  currentGear = totalGears;
  setActuatorToGear(currentGear);
  Serial.print("[setup] SetupMode: Force highest gear => ");
  Serial.println(currentGear);
}

// ---------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------
void loop() {
  static unsigned long lastBlinkMillis = 0;

  batteryLevel = getBatteryPercentage();

  // 1) Tasten abfragen
  checkSetupButtonPress();

  // 2) Modus-Logik
  if (isSetupMode) {
    // Rote LED => "Setup Mode"
    // Hier passiert nichts Automatisches
  } 
  else {
    // Auto Shift Mode => LED aus (oder grün?)
    setLED(0, 0, 0);

    // Geschwindigkeit auf 0 setzen, wenn lange kein Puls
    unsigned long nowMs = millis();
    if ((nowMs - (lastPulseTime / 1000)) > 2000) {
      speed = 0;
    }

    // Akku-Warn-Blinken
    if (batteryLevel < criticalBattery) {
      if (nowMs - lastBlinkMillis >= 2000) {
        lastBlinkMillis = nowMs;
        blinkLED(255, 130, 0, 1, 500); // Orange-Blink
      }
    }

    float cadence   = calculateCadence();
    float tiltPerc  = getCurrentTiltPercentage();

    // Automatisches Schalten
    autoShiftGears();

    // Debug-Ausgaben
    Serial.print("[loop] Batterie: ");
    Serial.print(batteryLevel, 1);
    Serial.print("%, Speed: ");
    Serial.print(speed);
    Serial.print(" km/h, Cadence: ");
    Serial.print(cadence);
    Serial.print(" RPM, Tilt: ");
    Serial.print(tiltPerc, 2);
    Serial.print(" %, Gear: ");
    Serial.println(currentGear);

    delay(100);
  }
}

// ---------------------------------------------------------------------
// CHECK BUTTON PRESS (Kurz/Lang, Doppelklick)
// ---------------------------------------------------------------------
void checkSetupButtonPress() {
  int buttonState = digitalRead(calibrateButtonPin);

  // Doppelklick => Akkustatus
  if (buttonState == LOW) {
    unsigned long currentTime = millis();
    if (isWaitingForSecondClick && (currentTime - lastButtonPressTime <= doubleClickInterval)) {
      isWaitingForSecondClick = false;  
      Serial.println("[checkSetupButtonPress] Doppelklick => Akku-Check");
      checkBatteryStatus();             
    } else {
      isWaitingForSecondClick = true;
      lastButtonPressTime = currentTime;
    }
  } else if (isWaitingForSecondClick) {
    if (millis() - lastButtonPressTime > doubleClickInterval) {
      isWaitingForSecondClick = false;
    }
  }

  // Langer Klick => Moduswechsel Setup <-> AutoShift
  if (buttonState == LOW && buttonPressStartTime == 0) {
    buttonPressStartTime = millis();
  } 
  else if (buttonState == LOW && millis() - buttonPressStartTime > 3000) {
    // Modus umschalten
    isSetupMode = !isSetupMode;
    buttonPressStartTime = 0;

    if (isSetupMode) {
      setLED(255, 0, 0);
      Serial.println("[checkSetupButtonPress] Wechsel in Setup Mode");

      // Gang 11 erzwingen
      currentGear = totalGears;
      setActuatorToGear(currentGear);
      Serial.println("[checkSetupButtonPress] Forced highest gear (11)");

    } else {
      blinkLED(0, 255, 0, 3, 300);
      Serial.println("[checkSetupButtonPress] Wechsel in Auto Shift Mode");

      // Start-Gang erzwingen
      if (startGearAuto >= 1 && startGearAuto <= totalGears) {
        int diff = startGearAuto - currentGear;
        if (diff > 0) {
          for (int i = 0; i < diff; i++) shiftUp();
        } else if (diff < 0) {
          for (int i = 0; i < abs(diff); i++) shiftDown();
        }
      }
    }
  } 
  else if (buttonState == HIGH && buttonPressStartTime > 0) {
    unsigned long pressDuration = millis() - buttonPressStartTime;
    if (pressDuration < 3000) {
      // Kurzer Klick
      if (isSetupMode) {
        // => Forward-Kalibrierung starten (inkl. Gyro-Neukalibrierung)
        Serial.println("[checkSetupButtonPress] Kurzer Klick im Setup Mode => calibrateForwardDirection()");
        calibrateForwardDirection();
      } else {
        // Im Auto-Shift-Mode => LED Blink => Status
        blinkLED(0, 255, 0, 3, 300); 
        Serial.println("[checkSetupButtonPress] Status: Auto Shift Mode (kurzer Klick).");
      }
    }
    buttonPressStartTime = 0;
  }
}

// ---------------------------------------------------------------------
// FORWARD-KALIBRIERUNG: 5s Countdown + Gyro-Drift + 5s Schieben
// ---------------------------------------------------------------------
void calibrateForwardDirection() {
  // 1) 5× Blinken blau (1s pro Blink) => Countdown
  //    => Zeit, das Rad still hinzustellen
  Serial.println("[calibrateForwardDirection] Countdown 5s (Blink blau)...");
  for (int i = 0; i < 5; i++) {
    setLED(0, 0, 255);
    delay(500);
    setLED(0, 0, 0);
    delay(500);
  }

  // 2) Jetzt Gyro-Neukalibrierung (Sensor ruhig halten!)
  Serial.println("[calibrateForwardDirection] Jetzt Gyro-Neukalibrierung (Sensor still)...");
  setLED(0, 0, 255); // Dauern blau
  calibrateZero();

  // 3) 5s Vorwärts-Schieben => wir messen Mittelwert von (ax, ay)
  Serial.println("[calibrateForwardDirection] Bitte Fahrrad 5 Sekunden vorwärts schieben...");
  unsigned long startMs = millis();
  float sumAx = 0.0;
  float sumAy = 0.0;
  int samples = 0;

  while (millis() - startMs < 5000) {
    // Beschleunigung lesen
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

    int16_t axRaw = Wire.read() << 8 | Wire.read();
    int16_t ayRaw = Wire.read() << 8 | Wire.read();
    int16_t azRaw = Wire.read() << 8 | Wire.read();

    int16_t gxRaw = Wire.read() << 8 | Wire.read();
    int16_t gyRaw = Wire.read() << 8 | Wire.read();
    int16_t gzRaw = Wire.read() << 8 | Wire.read();

    float ax = axRaw * accelScale;
    float ay = ayRaw * accelScale;
    // float az = azRaw * accelScale; // nicht verwendet

    sumAx += ax;
    sumAy += ay;
    samples++;

    delay(50); // ~20Hz Abtastrate
  }

  // 4) Durchschnitt berechnen => Winkel in Grad
  if (samples > 0) {
    float avgAx = sumAx / samples;
    float avgAy = sumAy / samples;

    // atan2(ay, ax) => Winkel in XY-Ebene
    float angleRad = atan2(avgAy, avgAx);
    float angleDeg = angleRad * 180.0 / M_PI;

    forwardCalibrationAngleDeg = angleDeg;
    storeEepromFloat(EEPROM_ADDR_FORWARD_ANGLE, forwardCalibrationAngleDeg);

    Serial.print("[calibrateForwardDirection] Kalibrierung beendet. Vorwärtswinkel = ");
    Serial.print(angleDeg, 2);
    Serial.println("° (EEPROM gespeichert)");
  }

  // 5) LED aus, zurück in Setup
  setLED(255, 0, 0); 
  Serial.println("[calibrateForwardDirection] Zurück in Setup Mode.");
}

// ---------------------------------------------------------------------
// GYRO-NULL-KALIBRIERUNG
// ---------------------------------------------------------------------
void calibrateZero() {
  // Wir setzen angleX = 0, ermitteln den Gyro-Drift
  // => Dann liegt "Flach" bei angleX=0
  // (baseAngle entfällt, wir nutzen forwardCalibrationAngleDeg für Korrektur)
  Serial.println("[calibrateZero] Starte Null-Kalibrierung (Gyro-Drift). Bitte stillhalten...");

  float sumGyro = 0;
  const int sampleCount = 200;

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Gyro-Integration
  for (int i = 0; i < sampleCount; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

    int16_t axRaw = Wire.read() << 8 | Wire.read(); 
    int16_t ayRaw = Wire.read() << 8 | Wire.read(); 
    int16_t azRaw = Wire.read() << 8 | Wire.read();

    int16_t gxRaw = Wire.read() << 8 | Wire.read();
    int16_t gyRaw = Wire.read() << 8 | Wire.read();
    int16_t gzRaw = Wire.read() << 8 | Wire.read();

    // Summiere nur GyroX (Pitch)
    sumGyro += (gxRaw * gyroScale);

    delay(5);
  }

  gyroDrift = sumGyro / sampleCount;
  angleX    = 0;  // Reset angleX

  Serial.print("[calibrateZero] GyroDrift: ");
  Serial.print(gyroDrift, 3);
  Serial.println("°/s, angleX auf 0 gesetzt.");
  prevTime = millis();
}

// ---------------------------------------------------------------------
// NEIGUNG BERECHNEN (in %), inkl. Forward-Korrektur
// ---------------------------------------------------------------------
float getCurrentTiltPercentage() {
  // 1) Sensor lesen
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

  // 2) Delta-Zeit
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // 3) Accelerometer Pitch (in Grad)
  //    Hier: positiver Winkel => Vorderrad nach oben
  //    Falls umgekehrt gewünscht, kann man angle * -1 machen
  float accelAngleDeg = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;

  // 4) Gyro-Integration
  angleX += gx * deltaTime; // Add deg/s * s => deg

  // 5) Komplementärfilter
  angleX = alpha * angleX + (1.0 - alpha) * accelAngleDeg;

  // 6) Forward-Korrektur:
  //    => Wir ziehen den Vorwärts-Kalibrierwinkel ab.
  //    => Ist forwardCalibrationAngleDeg hoch => man war geneigt.
  float correctedAngle = angleX - forwardCalibrationAngleDeg;

  // 7) Werte begrenzen
  if (correctedAngle >  maxTiltDegrees)  correctedAngle =  maxTiltDegrees;
  if (correctedAngle < -maxTiltDegrees)  correctedAngle = -maxTiltDegrees;

  float tiltPercentage = (correctedAngle / maxTiltDegrees) * 100.0;

  // Debug
  Serial.print("[getCurrentTiltPercentage] angleX=");
  Serial.print(angleX, 2);
  Serial.print("°, forwardOffset=");
  Serial.print(forwardCalibrationAngleDeg, 2);
  Serial.print(" => correctedAngle=");
  Serial.print(correctedAngle, 2);
  Serial.print(" => tiltPerc=");
  Serial.print(tiltPercentage, 2);
  Serial.println("%");

  return tiltPercentage;
}

// ---------------------------------------------------------------------
// SPEED-BERECHNUNG (ISR)
// ---------------------------------------------------------------------
void calculateSpeed() {
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime > debounceTime) {
    pulseInterval = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
    if (pulseInterval > 0) {
      speed = 3.6 * (wheelCircumference / (pulseInterval / 1e6));
      Serial.print("[ISR] pulseInterval: ");
      Serial.print(pulseInterval);
      Serial.print(" us => Speed: ");
      Serial.print(speed);
      Serial.println(" km/h");
    }
  }
}

// ---------------------------------------------------------------------
// TRITTFREQUENZ
// ---------------------------------------------------------------------
float calculateCadence() {
  if (speed == 0) return 0.0;

  float speed_m_s = speed / 3.6; 
  float rev_s = speed_m_s / wheelCircumference; 
  wheelRPM = rev_s * 60.0;

  float gearRatio = (float)chainringTeeth / (float)getCassetteTeeth(currentGear);

  float cadence = wheelRPM * gearRatio;
  Serial.print("[calculateCadence] wheelRPM=");
  Serial.print(wheelRPM, 2);
  Serial.print(", gearRatio=");
  Serial.print(gearRatio, 2);
  Serial.print(", Cadence=");
  Serial.println(cadence, 2);

  return cadence;
}

// ---------------------------------------------------------------------
// OPTIMALER GANG
// ---------------------------------------------------------------------
int getOptimalGear(float wheelSpeed, float slopePercent, float desiredCadence) {
  if (slopePercent > 15.0)  slopePercent = 15.0;
  if (slopePercent < -15.0) slopePercent = -15.0;

  float adjustedCadence = desiredCadence + slopePercent;
  if (adjustedCadence < 1.0) adjustedCadence = 1.0;

  if (wheelSpeed <= 0 || adjustedCadence <= 0) {
    Serial.println("[getOptimalGear] Stillstand oder ungueltig => kein Gangwechsel.");
    return currentGear;
  }

  float optimalGearRatio = wheelSpeed / adjustedCadence; 
  float smallestDelta     = FLT_MAX;
  int   bestGear          = currentGear;

  Serial.print("[getOptimalGear] wheelSpeed=");
  Serial.print(wheelSpeed, 2);
  Serial.print(" km/h, slopePercent=");
  Serial.print(slopePercent, 1);
  Serial.print("% => adjustedCadence=");
  Serial.println(adjustedCadence, 2);

  for (int g = 1; g <= totalGears; g++) {
    float gRatio = (float)chainringTeeth / (float)getCassetteTeeth(g);
    float delta  = fabs(optimalGearRatio - gRatio);
    if (delta < smallestDelta) {
      smallestDelta = delta;
      bestGear = g;
    }
  }

  Serial.print("[getOptimalGear] => bestGear=");
  Serial.println(bestGear);
  return bestGear;
}

// ---------------------------------------------------------------------
// AUTO-SHIFT
// ---------------------------------------------------------------------
void autoShiftGears() {
  float slopePerc = getCurrentTiltPercentage();
  int newGear = getOptimalGear(wheelRPM, slopePerc, optimalCadence);
  if (newGear == currentGear) {
    return;
  }

  int diff = newGear - currentGear;
  Serial.print("[autoShiftGears] currentGear=");
  Serial.print(currentGear);
  Serial.print(", newGear=");
  Serial.print(newGear);
  Serial.print(", diff=");
  Serial.println(diff);

  if (abs(diff) > maxGearJump) {
    diff = (diff > 0) ? maxGearJump : -maxGearJump;
    Serial.print("[autoShiftGears] diff gekappt auf ");
    Serial.println(diff);
  }

  if (diff > 0) {
    for (int i = 0; i < diff; i++) shiftUp();
  } else {
    for (int i = 0; i < abs(diff); i++) shiftDown();
  }
}

// ---------------------------------------------------------------------
// SHIFT-FUNKTIONEN => speichern aktuellen Gang ins EEPROM
// ---------------------------------------------------------------------
void shiftUp() {
  if (currentGear < totalGears) {
    float pf = getCassettePullFactor(currentGear);
    currentGear++;
    actuatorPositionMicros += pf * microsPerMm;
    actuator.writeMicroseconds(actuatorPositionMicros);
    delay(300);

    // Gang speichern
    storeEepromInt(EEPROM_ADDR_LAST_GEAR, currentGear);

    Serial.print("[shiftUp] => newGear=");
    Serial.print(currentGear);
    Serial.print(", actuator=");
    Serial.println(actuatorPositionMicros);
  } else {
    Serial.println("[shiftUp] bereits highest gear");
  }
}

void shiftDown() {
  if (currentGear > 1) {
    int targetGear = currentGear - 1;
    float pf = getCassettePullFactor(targetGear);
    currentGear--;
    actuatorPositionMicros -= pf * microsPerMm;
    actuator.writeMicroseconds(actuatorPositionMicros);
    delay(300);

    // Gang speichern
    storeEepromInt(EEPROM_ADDR_LAST_GEAR, currentGear);

    Serial.print("[shiftDown] => newGear=");
    Serial.print(currentGear);
    Serial.print(", actuator=");
    Serial.println(actuatorPositionMicros);
  } else {
    Serial.println("[shiftDown] bereits lowest gear");
  }
}

// ---------------------------------------------------------------------
// AKTUATOR-KALIBRIERUNG (rein mechanisch)
// ---------------------------------------------------------------------
void calibrateActuator() {
  actuatorPositionMicros = actuatorMaxMicros;
  actuator.writeMicroseconds(actuatorPositionMicros);
  Serial.println("[calibrateActuator] auf max Position (mechanisch) gefahren.");
}

// ---------------------------------------------------------------------
// AKTUATOR AUF GEWÜNSCHTEN GANG FAHREN
// ---------------------------------------------------------------------
void setActuatorToGear(int gear) {
  if (gear < 1) gear = 1;
  if (gear > totalGears) gear = totalGears;

  float pfSum = 0.0;
  // Wir summieren die Pull-Faktoren von Gang=1 bis gear-1
  // oder verwenden Dein altes Schema:
  // => Du hattest: actuatorMinMicros + 150 + (pf * microsPerMm * (gear-1))
  //    Hier belassen wir es bei dem alten Rechenansatz:
  float pf = 0.0;
  for (int g = 1; g < gear; g++) {
    pf += getCassettePullFactor(g);
  }
  actuatorPositionMicros = actuatorMinMicros + (pf * microsPerMm);

  actuator.writeMicroseconds(actuatorPositionMicros);

  currentGear = gear;
  Serial.print("[setActuatorToGear] gear=");
  Serial.print(gear);
  Serial.print(" => actuatorMicros=");
  Serial.println(actuatorPositionMicros);

  // Letzten Gang ins EEPROM speichern
  storeEepromInt(EEPROM_ADDR_LAST_GEAR, currentGear);
}

// ---------------------------------------------------------------------
// EEPROM-Helfer
// ---------------------------------------------------------------------
void storeEepromFloat(int address, float value) {
  EEPROM.put(address, value);
}

float readEepromFloat(int address) {
  float val;
  EEPROM.get(address, val);
  if (isnan(val)) {
    val = 0.0;
  }
  return val;
}

// Wir nehmen an, dass int auf dem Arduino 2 Byte hat.
void storeEepromInt(int address, int value) {
  EEPROM.put(address, value);
}

int readEepromInt(int address) {
  int val;
  EEPROM.get(address, val);
  return val;
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

  Serial.print("[getBatteryPercentage] raw=");
  Serial.print(rawValue);
  Serial.print(", pinVolt=");
  Serial.print(voltageAtPin, 2);
  Serial.print(" V => battVolt=");
  Serial.print(batteryVoltage, 2);
  Serial.print(" V => ");
  Serial.print(percentage, 1);
  Serial.println("%");

  return percentage;
}

// ---------------------------------------------------------------------
// CHECK BATTERY STATUS
// ---------------------------------------------------------------------
void checkBatteryStatus() {
  float battery = getBatteryPercentage();

  Serial.print("[checkBatteryStatus] Akkustand: ");
  Serial.print(battery, 1);
  Serial.println("%");

  if (battery >= 20.0 && battery < 45.0) {
    blinkLED(255, 130, 0, 1, 300);
    Serial.println("[checkBatteryStatus] Level: 20%-45% (1x Orange Blink)");
  } 
  else if (battery >= 45.0 && battery < 75.0) {
    blinkLED(255, 130, 0, 2, 300);
    Serial.println("[checkBatteryStatus] Level: 45%-75% (2x Orange Blink)");
  } 
  else if (battery >= 75.0 && battery <= 100.0) {
    blinkLED(255, 130, 0, 3, 300);
    Serial.println("[checkBatteryStatus] Level: 75%-100% (3x Orange Blink)");
  } 
  else if (battery < 20.0) {
    Serial.println("[checkBatteryStatus] Kritischer Akkustand <20%. Keine Blinksignale.");
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
