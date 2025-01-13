

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

#include <Servo.h>   // Include the Servo library
#include <float.h>   // Für FLT_MAX
#include <Wire.h>    // Für Gyro-Auslesen
#include <math.h>    // Für fabs()

// ---------------------------------------------------------------------
// SERVO & HARDWARE
// ---------------------------------------------------------------------
Servo actuator;

// Pin Definitions
const int servoPin           = 9;    // Servo control pin
const int shiftUpButtonPin   = 11;   // Shift up button
const int shiftDownButtonPin = 12;   // Shift down button
const int calibrateButtonPin = 2;    // Calibration button
const int sensorPin          = 3;    // Hall sensor
const int analogPinBattery   = A0;   // Pin für Spannungsmessung

// RGB LED Pins
const int redPin   = 4;
const int greenPin = 5;
const int bluePin  = 6;

// ---------------------------------------------------------------------
// AKTUATOR & GANG-KONFIG
// ---------------------------------------------------------------------
const float actuatorStroke_mm = 46.0;   // Total stroke length in mm
const int   actuatorMaxMicros = 2000;   // Maximum servo pulse width in microseconds
const int   actuatorMinMicros = 1000;   // Minimum servo pulse width in microseconds
const float microsPerMm       = (actuatorMaxMicros - actuatorMinMicros) / actuatorStroke_mm; 

// ---------------------------------------------------------------------
// KASSETTE MIT INDIVIDUELLEN PULL-FAKTOREN
// ---------------------------------------------------------------------

const int chainringTeeth = 42; // Anzahl der Zähne des Kettenblatts (Beispielwert)

// Jeder Eintrag hat: { Ritzel-Zähne, Pull-Faktor in mm }
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
  {32, 3.07},
  {36, 3.75},
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
  int idx = cassetteIndexForGear(gear);
  // Debug-Ausgabe
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
  // Debug-Ausgabe
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
float maxSpeed           = 40.0;   // Nur Referenz, nicht zwingend genutzt
float wheelCircumference = 2.2;    // Radumfang in Metern (Beispielwert)
float optimalCadence     = 60;     // Trittfrequenz-Sollwert
float criticalBattery    = 20.0;   // Unter diesem Wert Warn-Blinken

const int maxGearJump   = 3;   // Max. Gänge in einem Schritt
const int startGearAuto = 2;   // Gang beim Start im Automodus

// ---------------------------------------------------------------------
// SPEED & HALL SENSOR
// ---------------------------------------------------------------------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
const unsigned long debounceTime     = 100000; // in Mikrosekunden
float speed       = 0;
float wheelRPM    = 0;

// ---------------------------------------------------------------------
// AKTUATOR-STATE
// ---------------------------------------------------------------------
int currentGear            = totalGears; // Start im höchsten Gang
int actuatorPositionMicros = actuatorMinMicros;
int startingPositionMicros = actuatorMaxMicros; // Nur für Kalibrierung

// ---------------------------------------------------------------------
// TIMING & MODES
// ---------------------------------------------------------------------
unsigned long gearChangeStartTime   = 0;
int previousGear                    = totalGears;
unsigned long buttonPressStartTime  = 0;
bool isSetupMode                    = true;

// Battery Button Check
unsigned long lastButtonPressTime   = 0;   // Zeit der letzten Tastenbetätigung
unsigned long doubleClickInterval   = 500; // Max. Zeit in ms zwischen zwei Klicks für Doppelklick
bool isWaitingForSecondClick        = false;    

// ---------------------------------------------------------------------
// GYRO
// ---------------------------------------------------------------------
float tiltPerc;
const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ;
int16_t gyroX,  gyroY,  gyroZ;
const float accelScale = 2.0 / 32768.0;  // ±2g Bereich
const float gyroScale  = 250.0 / 32768.0; 
float angleX    = 0;
float baseAngle = 0;
float gyroDrift = 0;
unsigned long prevTime;
const float maxTiltDegrees = 45.0; 
const float alpha          = 0.95; // Filterfaktor für Gyro/Accel-Fusion

// ---------------------------------------------------------------------
// BATTERY
// ---------------------------------------------------------------------
float batteryLevel;
const float R1        = 10000.0;
const float R2        = 15000.0;
const float minVoltage = 6.0;
const float maxVoltage = 8.4; // Li-Ion mit 2S => max 8.4 V

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

  // LED test
  Serial.println("[setup] LED rot einschalten zur Prüfung.");
  setLED(255, 0, 0); 
  delay(500);
  setLED(0, 0, 0);

  Serial.println("[setup] Starte Gyro-Kalibrierung...");
  calibrateZero();       

  Serial.println("[setup] Starte Aktuator-Kalibrierung...");
  calibrateActuator();   
  Serial.println("Starting in Setup Mode.");

  // Gang 11 (kleinstes Ritzel) erzwingen
  currentGear = totalGears;  
  float pf = getCassettePullFactor(currentGear);
  actuatorPositionMicros = actuatorMinMicros + 150 + (pf * microsPerMm * (currentGear - 1));
  actuator.writeMicroseconds(actuatorPositionMicros);

  Serial.print("[setup] Initialer Gang: ");
  Serial.println(currentGear);
  Serial.println("SetupMode: High gear forced (Gang 11, kleinstes Ritzel).");
}

// ---------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------
void loop() {
  unsigned long currentTime = millis();
  static unsigned long lastBlinkMillis = 0;

  batteryLevel = getBatteryPercentage();

  // Überprüfe Setup-Button (für Moduswechsel und Akkustand per Doppelklick)
  checkSetupButtonPress();

  if (isSetupMode) {
    // Setup Mode => Rote LED als Dauerlicht
    setLED(255, 0, 0);
  } 
  else {
    // Auto Shift Mode => LED aus (außer beim Blinken)
    setLED(0, 0, 0);

    // Falls lange kein Hall-Sensor-Impuls kam, Geschwindigkeit auf 0
    if ((currentTime - (lastPulseTime / 1000)) > 2000) {
      speed = 0;
    }

    // Bei niedrigem Akkustand Warn-Blinken
    if (batteryLevel < criticalBattery) {
      if (currentTime - lastBlinkMillis >= 2000) {
        lastBlinkMillis = currentTime;
        blinkLED(255, 130, 0, 1, 500); // Orange-Blink
      }
    }

    float cadence = calculateCadence();
    tiltPerc = getCurrentTiltPercentage();

    // Automatisches Schalten
    autoShiftGears();

    // Debug-Ausgaben
    Serial.print("[loop] Batterie-Ladestand: ");
    Serial.print(batteryLevel, 1);
    Serial.println("%");

    Serial.print("[loop] Speed: ");
    Serial.print(speed);
    Serial.print(" km/h, Cadence: ");
    Serial.print(cadence);
    Serial.print(" RPM, Neigung: ");
    Serial.print(tiltPerc, 2);
    Serial.print(" %, Gear: ");
    Serial.println(currentGear);

    delay(100);
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
      // Debug
      Serial.print("[ISR] pulseInterval: ");
      Serial.print(pulseInterval);
      Serial.print(" us => Speed: ");
      Serial.print(speed);
      Serial.println(" km/h");
    }
  }
}

// ---------------------------------------------------------------------
// Trittfrequenz
// ---------------------------------------------------------------------
float calculateCadence() {
  if (speed == 0) return 0.0;

  // Geschw. in m/s
  float speed_m_s = speed / 3.6; 
  // Radumdrehungen pro Sekunde
  float rev_s = speed_m_s / wheelCircumference; 
  // Radumdrehungen pro Minute
  wheelRPM = rev_s * 60.0;

  float gearRatio = (float)chainringTeeth / (float)getCassetteTeeth(currentGear);



  // Debug
  Serial.print("[calculateCadence] wheelRPM: ");
  Serial.print(wheelRPM, 2);
  Serial.print(" => gearRatio: ");
  Serial.print(gearRatio, 2);
  Serial.print(" => Cadence: ");
  Serial.println(wheelRPM * gearRatio, 2);

  return wheelRPM * gearRatio;
}

// ---------------------------------------------------------------------
// Optimaler Gang basierend auf Speed, Steigung & gewünschter Cadence
// ---------------------------------------------------------------------
int getOptimalGear(float wheelSpeed, float slopePercent, float desiredCadence) {
  // Begrenze slopePercent für Extremfälle
  if (slopePercent > 15.0)  slopePercent = 15.0;
  if (slopePercent < -15.0) slopePercent = -15.0;

  // Pseudo-Anpassung der Cadence an Steigung
  float adjustedCadence = desiredCadence + slopePercent;
  if (adjustedCadence < 1.0) adjustedCadence = 1.0;

  // Falls Stillstand oder absurde Eingaben => kein Gangwechsel
  if (wheelSpeed <= 0 || adjustedCadence <= 0) {
    Serial.println("[getOptimalGear] Ungültige Eingaben oder Stillstand => Kein Gangwechsel.");
    return currentGear;
  }

  float optimalGearRatio = wheelSpeed / adjustedCadence;
  float smallestDelta     = FLT_MAX;
  int   optimalGear       = currentGear;

  // Debug
  Serial.print("[getOptimalGear] wheelSpeed: ");
  Serial.print(wheelSpeed, 2);
  Serial.print(" km/h, slopePercent: ");
  Serial.print(slopePercent, 2);
  Serial.print("%, desiredCadence: ");
  Serial.print(desiredCadence, 2);
  Serial.print(" => adjustedCadence: ");
  Serial.println(adjustedCadence, 2);

  for (int g = 1; g <= totalGears; g++) {
    float gearRatio = (float)chainringTeeth / (float)getCassetteTeeth(g);
    float delta     = fabs(optimalGearRatio - gearRatio);
    if (delta < smallestDelta) {
      smallestDelta = delta;
      optimalGear   = g;
    }
  }

  Serial.print("[getOptimalGear] => Optimaler Gang: ");
  Serial.println(optimalGear);
  return optimalGear;
}

// ---------------------------------------------------------------------
// Mehrere Gänge in einem Schritt (AutoShift)
// ---------------------------------------------------------------------
void autoShiftGears() {
  int newGear = getOptimalGear(wheelRPM, tiltPerc, optimalCadence);
  if (newGear == currentGear) {
    // Debug
    //Serial.println("[autoShiftGears] Kein Gangwechsel nötig.");
    return;
  }

  int diff = newGear - currentGear;
  // Debug
  Serial.print("[autoShiftGears] CurrentGear: ");
  Serial.print(currentGear);
  Serial.print(", NewGear: ");
  Serial.print(newGear);
  Serial.print(", Diff: ");
  Serial.println(diff);

  if (abs(diff) > maxGearJump) {
    diff = (diff > 0) ? maxGearJump : -maxGearJump;
    Serial.print("[autoShiftGears] Diff größer als maxGearJump => gekürzt auf: ");
    Serial.println(diff);
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
  if (currentGear < totalGears) {
    // Pull-Faktor vom aktuellen Gang
    float pf = getCassettePullFactor(currentGear);
    currentGear++;
    actuatorPositionMicros += pf * microsPerMm;
    actuator.writeMicroseconds(actuatorPositionMicros);

    delay (300);

    Serial.print("[shiftUp] Shifted up to gear ");
    Serial.print(currentGear);
    Serial.print(" => Neuer actuatorPositionMicros: ");
    Serial.println(actuatorPositionMicros);
  } else {
    Serial.println("[shiftUp] Bereits im höchsten Gang. Keine Aktion.");
  }
}

void shiftDown() {
  if (currentGear > 1) {
    int targetGear = currentGear - 1;
    // Pull-Faktor für den darunterliegenden Gang
    float pf = getCassettePullFactor(targetGear);
    currentGear--;
    actuatorPositionMicros -= pf * microsPerMm;
    actuator.writeMicroseconds(actuatorPositionMicros);

    delay (300);

    Serial.print("[shiftDown] Shifted down to gear ");
    Serial.print(currentGear);
    Serial.print(" => Neuer actuatorPositionMicros: ");
    Serial.println(actuatorPositionMicros);
  } else {
    Serial.println("[shiftDown] Bereits im niedrigsten Gang. Keine Aktion.");
  }
}

// ---------------------------------------------------------------------
// SETUP-MODE: Kalibrierung des Aktuators
// ---------------------------------------------------------------------
void calibrateActuator() {
  actuatorPositionMicros = actuatorMaxMicros;
  actuator.writeMicroseconds(actuatorPositionMicros);
  Serial.println("[calibrateActuator] Aktuator in maximale Position gefahren.");
}

// ---------------------------------------------------------------------
// CHECK BUTTON PRESS (Moduswechsel, Doppelklick => Akku-Abfrage)
// ---------------------------------------------------------------------
void checkSetupButtonPress() {
  int buttonState = digitalRead(calibrateButtonPin);

  if (buttonState == LOW) {
    unsigned long currentTime = millis();

    // Prüfen, ob ein Doppelklick erkannt wurde
    if (isWaitingForSecondClick && (currentTime - lastButtonPressTime <= doubleClickInterval)) {
      // Zweiter Klick erkannt
      isWaitingForSecondClick = false;  
      Serial.println("[checkSetupButtonPress] Doppelklick erkannt => Akkustatus wird geprüft.");
      checkBatteryStatus();             
    } else {
      // Erstes Drücken, Doppelklick-Phase starten
      isWaitingForSecondClick = true;
      lastButtonPressTime = currentTime;
    }
  } else if (isWaitingForSecondClick) {
    // Prüfen, ob Zeit für den zweiten Klick abgelaufen ist
    if (millis() - lastButtonPressTime > doubleClickInterval) {
      // Zeit überschritten, kein Doppelklick
      isWaitingForSecondClick = false;
    }
  }

  // Langer Klick für Moduswechsel
  if (buttonState == LOW && buttonPressStartTime == 0) {
    // Starte Timer
    buttonPressStartTime = millis();
  } 
  else if (buttonState == LOW && millis() - buttonPressStartTime > 3000) {
    // Moduswechsel
    isSetupMode = !isSetupMode;
    buttonPressStartTime = 0;

    if (isSetupMode) {
      setLED(255, 0, 0);  // Setup-Mode LED
      calibrateActuator();
      Serial.println("[checkSetupButtonPress] Setup Mode aktiviert.");

      currentGear = totalGears; 
      float pf = getCassettePullFactor(currentGear);
      actuatorPositionMicros = actuatorMinMicros + (pf * microsPerMm * (currentGear - 1));
      actuator.writeMicroseconds(actuatorPositionMicros);
      Serial.println("[checkSetupButtonPress] SetupMode: Forced highest gear (11).");
    } else {
      blinkLED(0, 255, 0, 3, 300);  // Auto-Mode LED
      Serial.println("[checkSetupButtonPress] Auto Shift Mode aktiviert.");

      // Beim Wechsel in den Auto-Modus ggf. auf startGearAuto schalten
      if (startGearAuto < 1)  return; 
      if (startGearAuto > totalGears) return;

      int diff = startGearAuto - currentGear;
      if (diff > 0) {
        for (int i = 0; i < diff; i++) shiftUp();
      } else if (diff < 0) {
        for (int i = 0; i < abs(diff); i++) shiftDown();
      }
    }
  } 
  else if (buttonState == HIGH && buttonPressStartTime > 0) {
    // Kurzer Klick => Statusblinken
    if (millis() - buttonPressStartTime < 3000) {
      if (isSetupMode) {
        blinkLED(255, 0, 0, 3, 300);  // Setup Mode Status
        Serial.println("[checkSetupButtonPress] Status: Setup Mode");
      } else {
        blinkLED(0, 255, 0, 3, 300);  // Auto Shift Status
        Serial.println("[checkSetupButtonPress] Status: Auto Shift Mode");
      }
    }
    buttonPressStartTime = 0;
  }
}

// ---------------------------------------------------------------------
// LED-Steuerung
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
// GYRO-Kalibrierung (Null-Lage bestimmen & Gyro-Drift ermitteln)
// ---------------------------------------------------------------------
void calibrateZero() {
  Serial.println("[calibrateZero] Starte Kalibrierung. Bitte Sensor ruhig halten.");

  float sumAccelAngle = 0;
  float sumGyro       = 0;
  const int sampleCount = 200;

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // MPU aktivieren
  Wire.endTransmission(true);

  for (int i = 0; i < sampleCount; i++) {
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

    float accelAngle = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;

    sumAccelAngle += accelAngle;
    sumGyro       += (gyroX * gyroScale);

    delay(5);
  }

  baseAngle = sumAccelAngle / sampleCount;
  gyroDrift = sumGyro / sampleCount;
  angleX    = baseAngle;

  Serial.print("[calibrateZero] Calibration complete. Zero angle = ");
  Serial.print(baseAngle, 2);
  Serial.print("°, Gyro drift = ");
  Serial.print(gyroDrift, 2);
  Serial.println("°/s");

  prevTime = millis();
}

// ---------------------------------------------------------------------
// AKTUELLE NEIGUNG AUS GYRO+ACCEL
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

  // Gyro-Integration
  angleX += gx * deltaTime;
  // Komplementärfilter
  angleX = alpha * angleX + (1.0 - alpha) * accelAngle;

  float relativeAngle = angleX - baseAngle;
  if (relativeAngle >  maxTiltDegrees) relativeAngle =  maxTiltDegrees;
  if (relativeAngle < -maxTiltDegrees) relativeAngle = -maxTiltDegrees;

  float tiltPercentage = (relativeAngle / maxTiltDegrees) * 100.0;

  // Debug
  Serial.print("[getCurrentTiltPercentage] angleX: ");
  Serial.print(angleX, 2);
  Serial.print("°, baseAngle: ");
  Serial.print(baseAngle, 2);
  Serial.print("°, relativeAngle: ");
  Serial.print(relativeAngle, 2);
  Serial.print("° => tiltPercentage: ");
  Serial.print(tiltPercentage, 2);
  Serial.println("%");

  return tiltPercentage;
}

// ---------------------------------------------------------------------
// AKKUSTAND ERMITTELN
// ---------------------------------------------------------------------
float getBatteryPercentage() {
  int rawValue = analogRead(analogPinBattery);
  float voltageAtPin = (rawValue / 1023.0) * 5.0;

  float dividerFactor = R2 / (R1 + R2); 
  float batteryVoltage = voltageAtPin / dividerFactor;

  float percentage = (batteryVoltage - minVoltage)
                   / (maxVoltage - minVoltage) * 100.0;

  // Begrenzen
  if (percentage < 0.0)   percentage = 0.0;
  if (percentage > 100.0) percentage = 100.0;

  // Debug
  Serial.print("[getBatteryPercentage] rawValue: ");
  Serial.print(rawValue);
  Serial.print(", voltageAtPin: ");
  Serial.print(voltageAtPin, 2);
  Serial.print(" V, batteryVoltage: ");
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
    blinkLED(255, 130, 0, 1, 300); // 1x Orange blinken
    Serial.println("[checkBatteryStatus] Level: 20% - 45% (1x Orange Blink)");
  } 
  else if (battery >= 45.0 && battery < 75.0) {
    blinkLED(255, 130, 0, 2, 300); // 2x Orange blinken
    Serial.println("[checkBatteryStatus] Level: 45% - 75% (2x Orange Blink)");
  } 
  else if (battery >= 75.0 && battery <= 100.0) {
    blinkLED(255, 130, 0, 3, 300); // 3x Orange blinken
    Serial.println("[checkBatteryStatus] Level: 75% - 100% (3x Orange Blink)");
  } 
  else if (battery < 20.0) {
    Serial.println("[checkBatteryStatus] Kritischer Akkustand <20%! Keine Blinksignale.");
  }
}
