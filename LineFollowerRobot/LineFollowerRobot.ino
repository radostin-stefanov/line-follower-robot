#include <QTRSensors.h>
#include "U8glib.h"
#include <avr/eeprom.h>

// Comment the following line out when done debugging.
//#define DEBUG

// Use LOG("abc") to output "abc" on the Serial interface if DEBUG is enabled.
// If DEBUG is disabled then LOG("abc") has no effect.
#ifdef DEBUG
#define LOG(X) Serial.println(X)
#else
#define LOG(X)
#endif

// The OLED Display
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST); // Fast I2C / TWI

int maxSpeed = 180;
int baseSpeed = 130;

//Kp 1
//Kd 17
//250 max
//200 base
int Kp = 1;  // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
int Kd = 17;  // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
int parameters[] = { Kp, Kd, maxSpeed, baseSpeed };

#define NUM_SENSORS       6  // number of sensors used
#define TIMEOUT         2500 // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN       2  // emitter is controlled by digital pin 2

#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 5
#define leftMotor1 12
#define leftMotor2 11
#define leftMotorPWM 10
#define motorPower 8

#define KEY_NONE 0
#define KEY_PREV 1
#define KEY_NEXT 2
#define KEY_SELECT 3
#define KEY_BACK 4

uint8_t uiKeyPrev = 6;
uint8_t uiKeyNext = 13;
uint8_t uiKeySelect = 7;
uint8_t uiKeyBack = 9;

uint8_t uiKeyCodeFirst = KEY_NONE;
uint8_t uiKeyCodeSecond = KEY_NONE;
uint8_t uiKeyCode = KEY_NONE;

int lastError = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

bool doneCalibrating = false;

QTRSensorsRC qtrrc((unsigned char[]) {
  A1, A2, A3, A6, A7, A0
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

#define MENU_ITEMS 4
#define PAGES 5
const char *titles[PAGES] = {
  "F1Robot",
  "Start",
  "Calibration",
  "PID",
  "Speed"
};

#define CALIBRATING "Calibrating..."
#define KP "Kp = "
#define KD "Kd = "
#define MAX_SPEED "Max Speed = "
#define BASE_SPEED "Base Speed = "
#define STARTED "Started"

const char *menu_strings[PAGES][MENU_ITEMS] = {
  { "Start", "Calibration", "PID", "Speed" },
  { STARTED, "", "", "" },
  { CALIBRATING, "", "", "" },
  { KP, KD, "", "" },
  { MAX_SPEED, BASE_SPEED, "", "" }
};
uint8_t page_current = 0;
uint8_t menu_current = 0;
uint8_t menu_redraw_required = 0;
uint8_t last_key_code = KEY_NONE;


void uiSetup(void) {
  pinMode(uiKeyPrev, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyNext, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeySelect, INPUT_PULLUP);         // set pin to input with pullup
  pinMode(uiKeyBack, INPUT_PULLUP);           // set pin to input with pullup
}

void stopMotor() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(motorPower, LOW);
  analogWrite(leftMotorPWM, 0);
  analogWrite(rightMotorPWM, 0);
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
}

void turnRight() {
  digitalWrite(motorPower, HIGH);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 20);
}

void turnBackRight() {
  digitalWrite(motorPower, HIGH);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, 20);
}

void turnLeft() {
  digitalWrite(motorPower, HIGH);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 20);
}

void turnBackLeft() {
  digitalWrite(motorPower, HIGH);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 20);
}

void calibration() {
  for (int i = 0; i < 100; i++) { // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
    if (i < 50) {
      if (i < 25) {
        turnRight();
      } else {
        turnBackRight();
      }
      analogWrite(leftMotorPWM, 0);
    } else {
      if (i < 75) {
        turnLeft();
      } else {
        turnBackLeft();
      }
      analogWrite(rightMotorPWM, 0);
    }
    qtrrc.calibrate();
    delay(20);
  }
  stopMotor();
  doneCalibrating = true;
  menu_redraw_required = 1;
}

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);

  // Read Kd, Kp, LSpeed and RSpeed from EEPROM
  eeprom_read_block((void*)&parameters, (void*)0, sizeof(parameters));
  if (parameters[0] > 0)
    Kp = parameters[0];
  if (parameters[1] > 0)
    Kd = parameters[1];
  if (parameters[2] >= 0)
    maxSpeed = parameters[2];
  if (parameters[3] >= 0)
    baseSpeed = parameters[3];

  // rotate screen, if required
  //u8g.setRot180();
  uiSetup();                    // setup key detection and debounce algorithm
  menu_redraw_required = 1;     // force initial redraw
  u8g.setHiColorByRGB(255, 255, 255);
}

void uiStep(void) {
  uiKeyCodeSecond = uiKeyCodeFirst;
  if ( digitalRead(uiKeyPrev) == LOW )
    uiKeyCodeFirst = KEY_PREV;
  else if ( digitalRead(uiKeyNext) == LOW )
    uiKeyCodeFirst = KEY_NEXT;
  else if ( digitalRead(uiKeySelect) == LOW )
    uiKeyCodeFirst = KEY_SELECT;
  else if ( digitalRead(uiKeyBack) == LOW )
    uiKeyCodeFirst = KEY_BACK;
  else
    uiKeyCodeFirst = KEY_NONE;

  if ( uiKeyCodeSecond == uiKeyCodeFirst )
    uiKeyCode = uiKeyCodeFirst;
  else
    uiKeyCode = KEY_NONE;

  delay(10);
}

void drawHeader(void) {
  u8g.setFont(u8g_font_unifont);
  u8g.setDefaultForegroundColor();
  u8g.drawStr((u8g.getWidth() - u8g.getStrWidth(titles[page_current])) / 2, 10, titles[page_current]);
}

char drawMenuPlaceholder[32]; // 32 chars is more than enough.
char drawMenuNumberPlaceholder[16]; // Here numbers are prepared.
bool writeMode = false;
bool parameter = false;
bool calibrate = false;
bool started   = false;

void run() {
  // Here we go.
  LOG("Go");
  unsigned int sensors[6];
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.

  int error = 2500 - position; 

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  rightMotorSpeed = baseSpeed + motorSpeed;
  leftMotorSpeed = baseSpeed - motorSpeed;

  if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

  {
    digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, rightMotorSpeed);
    digitalWrite(motorPower, HIGH);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, leftMotorSpeed);
  }
}

void drawMenu(void) {
  uint8_t i, h;
  u8g_uint_t w, d;
  u8g.setFont(u8g_font_6x13);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();
  started = false;
  calibrate = false;

  for ( i = 0; i < MENU_ITEMS; i++ ) {
    const char* menu_item = menu_strings[page_current][i];
    bool param = false;
    if (strcmp(menu_item, CALIBRATING) == 0) {
      calibrate = true;
      if (doneCalibrating) {
        menu_item = "Done.";
      }
    }
    else if (strcmp(menu_item, KP) == 0) {
      sprintf(drawMenuNumberPlaceholder, "%d", Kp);
      strcpy(drawMenuPlaceholder, menu_item);
      menu_item = strcat(drawMenuPlaceholder, drawMenuNumberPlaceholder);
      param = true;
    }
    else if (strcmp(menu_item, KD) == 0) {
      sprintf(drawMenuNumberPlaceholder, "%d", Kd);
      strcpy(drawMenuPlaceholder, menu_item);
      menu_item = strcat(drawMenuPlaceholder, drawMenuNumberPlaceholder);
      param = true;
    }
    else if (strcmp(menu_item, MAX_SPEED) == 0) {
      sprintf(drawMenuNumberPlaceholder, "%d", maxSpeed);
      strcpy(drawMenuPlaceholder, menu_item);
      menu_item = strcat(drawMenuPlaceholder, drawMenuNumberPlaceholder);
      param = true;
    }
    else if (strcmp(menu_item, BASE_SPEED) == 0) {
      sprintf(drawMenuNumberPlaceholder, "%d", baseSpeed);
      strcpy(drawMenuPlaceholder, menu_item);
      menu_item = strcat(drawMenuPlaceholder, drawMenuNumberPlaceholder);
      param = true;
    }
    else if (strcmp(menu_item, STARTED) == 0) {
      started = true;
    }
    d = (w - u8g.getStrWidth(menu_item)) / 2;
    u8g.setDefaultForegroundColor();
    if ( i == menu_current ) {
      u8g.drawBox(0, 15 + (i * h + 1), w, h);
      u8g.setDefaultBackgroundColor();
      parameter = param;
    }
    u8g.drawStr(d, 15 + (i * h), menu_item);
    if (writeMode && i == menu_current) {
      u8g.drawStr(d + 1, 15 + (i * h), menu_item);
    }
  }

}

void updateMenu(void) {
  if ( uiKeyCode != KEY_NONE && last_key_code == uiKeyCode ) {
    return;
  }
  last_key_code = uiKeyCode;

  switch ( uiKeyCode ) {
    case KEY_NEXT:
      if ( writeMode ) {
        if ( page_current == 3 && menu_current == 0) { // Kp
          Kp--;
          if (Kp < 1)
            Kp = 1;
        }
        else if ( page_current == 3 && menu_current == 1) { // Kd
          Kd--;
          if (Kd < 1)
            Kd = 1;
        }
        else if ( page_current == 4 && menu_current == 0) { // maxSpeed
          maxSpeed--;
          if (maxSpeed < 0)
            maxSpeed = 0;
        }
        else if ( page_current == 4 && menu_current == 1) { // baseSpeed
          baseSpeed--;
          if (baseSpeed < 0)
            baseSpeed = 0;
        }
        menu_redraw_required = 1;
      }
      else {
        menu_current++;
        if ( menu_current >= MENU_ITEMS )
          menu_current = MENU_ITEMS - 1;
        if ( strlen(menu_strings[page_current][menu_current]) == 0 )
          menu_current--;
        menu_redraw_required = 1;
      }
      break;
    case KEY_PREV:
      if ( writeMode ) {
        if ( page_current == 3 && menu_current == 0) { // Kp
          Kp++;
        }
        else if ( page_current == 3 && menu_current == 1) { // Kd
          Kd++;
        }
        else if ( page_current == 4 && menu_current == 0) { // maxSpeed
          maxSpeed++;
        }
        else if ( page_current == 4 && menu_current == 1) { // baseSpeed
          baseSpeed++;
        }
        menu_redraw_required = 1;
      }
      else if ( menu_current > 0 ) {
        menu_current--;
        menu_redraw_required = 1;
      }
      break;
    case KEY_SELECT:
      if ( page_current == 0 ) {
        page_current = menu_current + 1;
        menu_current = 0;
        menu_redraw_required = 1;
        writeMode = false;
      } else {
        writeMode = parameter;
        menu_redraw_required = 1;
      }
      break;
    case KEY_BACK:
      if ( writeMode ) {
        // Store Kp, Kd, maxSpeed and baseSpeed in EEPROM
        parameters[0] = Kp;
        parameters[1] = Kd;
        parameters[2] = maxSpeed;
        parameters[3] = baseSpeed;
        eeprom_write_block((void*)&parameters, (void*)0, sizeof(parameters));

        writeMode = false;
        menu_redraw_required = 1;
      } else if ( page_current > 0 ) {
        menu_current = page_current - 1;
        page_current = 0;
        menu_redraw_required = 1;
        writeMode = false;
        doneCalibrating = false;
        started = false;
        stopMotor();
      }
      break;
  }
}

void loop() {
  LOG("Loop");

  uiStep();                     // check for key press
  if (  menu_redraw_required != 0 ) {
    u8g.firstPage();
    do  {
      drawHeader();
      drawMenu();
      drawHeader();
    } while ( u8g.nextPage() );
    menu_redraw_required = 0;
    if (calibrate && !doneCalibrating) {
      calibration();
    }
  }

  updateMenu();                 // update menu bar
  
  if (started) {
    run();
  }
}

