// Pins für die RGB-LED
const int redPin = 2;   // PWM Pin für Rot
const int greenPin = 3; // PWM Pin für Grün
const int bluePin = 4; // PWM Pin für Blau

void setup() {
  // Pins als Ausgang definieren
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  // Farben abwechselnd anzeigen
  setColor(255, 0, 0);  // Rot
  delay(1000);
  setColor(0, 255, 0);  // Grün
  delay(1000);
  setColor(0, 0, 255);  // Blau
  delay(1000);
  setColor(255, 255, 0);  // Gelb
  delay(1000);
  setColor(0, 255, 255);  // Cyan
  delay(1000);
  setColor(255, 0, 255);  // Magenta
  delay(1000);
  setColor(255, 255, 255);  // Weiß
  delay(1000);
  setColor(0, 0, 0);  // Aus
  delay(1000);
}

// Funktion zur Farbsteuerung
void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}
