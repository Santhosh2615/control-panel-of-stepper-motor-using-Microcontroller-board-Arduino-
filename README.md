# control-panel-of-stepper-motor-using-Microcontroller-board-Arduino-
Developed a control panel for stepper motor to control its speed , counts and direction of rotation with Arduino microcontroller board using C++ libraries

//code starts here
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define STEP_PIN 3       
#define DIR_PIN 4        
#define ON_OFF_BUTTON 7     
#define INC_SPEED_BUTTON 6  
#define DEC_SPEED_BUTTON 5  
#define CW_BUTTON 8         
#define CCW_BUTTON 9         

#define SPEED_STEP 200       

int stepDelay = 2000;         
unsigned long previousStepTime = 0;  
bool motorOn = false;  
int stepCount = 0;
bool directionCW = true;      

bool lastOnOffState = LOW;
bool lastIncState = LOW;
bool lastDecState = LOW;
bool lastCWState = LOW;
bool lastCCWState = LOW;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  pinMode(ON_OFF_BUTTON, INPUT);
  pinMode(INC_SPEED_BUTTON, INPUT);
  pinMode(DEC_SPEED_BUTTON, INPUT);
  pinMode(CW_BUTTON, INPUT);
  pinMode(CCW_BUTTON, INPUT);

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Motor: OFF");
  lcd.setCursor(0, 1);
  lcd.print("Speed: " + String(getRPM()) + " RPM");

  digitalWrite(DIR_PIN, directionCW ? HIGH : LOW);

  Serial.begin(9600);
}

void loop() {
  bool onOffState = digitalRead(ON_OFF_BUTTON);
  if (onOffState != lastOnOffState && onOffState == HIGH) {
    motorOn = !motorOn;

    lcd.setCursor(0, 0);
    lcd.print("Motor: ");
    lcd.print(motorOn ? "ON " : "OFF");

    Serial.println(motorOn ? "Motor ON" : "Motor OFF");
    delay(200);
  }
  lastOnOffState = onOffState;

  bool incState = digitalRead(INC_SPEED_BUTTON);
  bool decState = digitalRead(DEC_SPEED_BUTTON);

  if (incState != lastIncState && incState == HIGH) {
    stepDelay -= SPEED_STEP;
    if (stepDelay < 200) stepDelay = 200;

    updateSpeedDisplay();
    Serial.println("Speed increased: " + String(getRPM()) + " RPM");
    delay(200);
  }

  if (decState != lastDecState && decState == HIGH) {
    stepDelay += SPEED_STEP;
    if (stepDelay > 10000) stepDelay = 10000;

    updateSpeedDisplay();
    Serial.println("Speed decreased: " + String(getRPM()) + " RPM");
    delay(200);
  }

  lastIncState = incState;
  lastDecState = decState;

  bool cwState = digitalRead(CW_BUTTON);
  bool ccwState = digitalRead(CCW_BUTTON);

  if (cwState != lastCWState && cwState == HIGH) {
    directionCW = true;
    digitalWrite(DIR_PIN, HIGH);
    updateDirectionDisplay();
    Serial.println("Direction: CW");
    delay(200);
  }

  if (ccwState != lastCCWState && ccwState == HIGH) {
    directionCW = false;
    digitalWrite(DIR_PIN, LOW);
    updateDirectionDisplay();
    Serial.println("Direction: CCW");
    delay(200);
  }

  lastCWState = cwState;
  lastCCWState = ccwState;

  unsigned long currentTime = micros();
  if (motorOn && (currentTime - previousStepTime >= stepDelay)) {
    previousStepTime = currentTime;

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);

    stepCount++;
    Serial.println("Steps: " + String(stepCount));

    updateRotationDisplay();
  }
}

int getRPM() {
  float stepsPerRevolution = 200;
  float delayPerRevolution = stepsPerRevolution * stepDelay / 1000.0;
  float rpm = (60.0 * 1000.0) / delayPerRevolution;
  return int(rpm);
}

void updateSpeedDisplay() {
  lcd.setCursor(0, 1);
  lcd.print("Speed: " + String(getRPM()) + " RPM ");
}

void updateDirectionDisplay() {
  lcd.setCursor(10, 0);
  lcd.print(directionCW ? "CW " : "CCW");
}

void updateRotationDisplay() {
  float rotations = stepCount / 200.0;
  lcd.setCursor(0, 1);
  lcd.print("Rot: " + String(rotations, 2) + "     ");
}
