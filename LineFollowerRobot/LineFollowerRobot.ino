//#include <QTRSensors.h>
#include "U8glib.h"

// The OLED Display
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST); // Fast I2C / TWI

//Kp 1
//Kd 17
//250 max
//200 base
#define Kp                1  // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd                17  // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed   180  // max speed of the robot
#define leftMaxSpeed    180  // max speed of the robot
#define rightBaseSpeed  130  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed   130  // this is the speed at which the motors should spin when the robot is perfectly on the line

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

uint8_t uiKeyPrev = 3;
uint8_t uiKeyNext = 12;
uint8_t uiKeySelect = 4;
uint8_t uiKeyBack = 2;

uint8_t uiKeyCodeFirst = KEY_NONE;
uint8_t uiKeyCodeSecond = KEY_NONE;
uint8_t uiKeyCode = KEY_NONE;

//QTRSensorsRC qtrrc((unsigned char[]) {A1, A2, A3, A4, A5, 6} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);

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
const char *menu_strings[PAGES][MENU_ITEMS] = {
  { "Start", "Calibration", "PID", "Speed" },
  { "Started", "", "", "" },
  { "Calibrating...", "", "", "" },
  { "My PID is", "", "", "" },
  { "My speed is", "", "", "" } };
uint8_t page_current = 0;
uint8_t menu_current = 0;
uint8_t menu_redraw_required = 0;
uint8_t last_key_code = KEY_NONE;


void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);

  for (int i = 0; i < 100; i++){ // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
//  qtrrc.calibrate();
  delay(20);
  }
  wait();
  delay(2000); // wait for 2s to position the bot before entering the main loop

  //  // comment out for serial printing
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++){
//    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++){
//    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // rotate screen, if required
  //u8g.setRot180();
  uiSetup();                    // setup key detection and debounce algorithm
  menu_redraw_required = 1;     // force initial redraw
}

void uiSetup(void) {
  pinMode(uiKeyPrev, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyNext, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeySelect, INPUT_PULLUP);         // set pin to input with pullup
  pinMode(uiKeyBack, INPUT_PULLUP);           // set pin to input with pullup
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

void drawMenu(void) {
  uint8_t i, h;
  u8g_uint_t w, d;
  u8g.setFont(u8g_font_6x13);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

  h = u8g.getFontAscent() - u8g.getFontDescent();
  w = u8g.getWidth();
  for ( i = 0; i < MENU_ITEMS; i++ ) {
    d = (w - u8g.getStrWidth(menu_strings[page_current][i])) / 2;
    u8g.setDefaultForegroundColor();
    if ( i == menu_current ) {
      u8g.drawBox(0, 15 + (i * h + 1), w, h);
      u8g.setDefaultBackgroundColor();
    }
    u8g.drawStr(d, 15 + (i * h), menu_strings[page_current][i]);
  }
}

void updateMenu(void) {
  if ( uiKeyCode != KEY_NONE && last_key_code == uiKeyCode ) {
    return;
  }
  last_key_code = uiKeyCode;

  switch ( uiKeyCode ) {
    case KEY_NEXT:
      menu_current++;
      if ( menu_current >= MENU_ITEMS )
        menu_current = MENU_ITEMS-1;
      if ( strlen(menu_strings[page_current][menu_current]) == 0 )
        menu_current--;
      menu_redraw_required = 1;
      break;
    case KEY_PREV:
      if ( menu_current > 0 )
        menu_current--;
      menu_redraw_required = 1;
      break;
    case KEY_SELECT:
      if ( page_current == 0 )
        page_current = menu_current + 1;
      menu_current = 0;
      menu_redraw_required = 1;
      break;
    case KEY_BACK:
      menu_current = page_current - 1;
      page_current = 0;
      menu_redraw_required = 1;
      break;
  }
}

int lastError = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
void loop()
{
// !!!!!!!! QTRSensors.h is missing !!!!!!!!
//  unsigned int sensors[6];
//  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
//
//  int error = 2500 - position; 
//
//  int motorSpeed = Kp * error + Kd * (error - lastError);
//  lastError = error;
//
//  rightMotorSpeed = rightBaseSpeed + motorSpeed;
//  leftMotorSpeed = leftBaseSpeed - motorSpeed;
//
//  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
//  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
//  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
//  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
//
//  {
//    digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
//    digitalWrite(rightMotor1, HIGH);
//    digitalWrite(rightMotor2, LOW);
//    analogWrite(rightMotorPWM, rightMotorSpeed);
//    digitalWrite(motorPower, HIGH);
//    digitalWrite(leftMotor1, HIGH);
//    digitalWrite(leftMotor2, LOW);
//    analogWrite(leftMotorPWM, leftMotorSpeed);
//  }

  uiStep();                     // check for key press
  if (  menu_redraw_required != 0 ) {
    u8g.firstPage();
    do  {
      drawHeader();
      drawMenu();
      drawHeader();
    } while ( u8g.nextPage() );
    menu_redraw_required = 0;
  }
  updateMenu();                            // update menu bar
}

void wait() {
  digitalWrite(motorPower, LOW);
}
