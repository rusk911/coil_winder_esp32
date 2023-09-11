#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#include "TouchScreen_kbv.h"
#define TouchScreen TouchScreen_kbv
#define TSPoint     TSPoint_kbv
#define MINPRESSURE 200
#define MAXPRESSURE 1000

// load cell amplifier
#include <HX711.h>
HX711 scale;

#define LOADCELL_DOUT_PIN  34
#define LOADCELL_SCK_PIN  18

// streaming load cell readings to get smoother results
#include <movingAvg.h>
movingAvg mySensor(5);

#define TITLE "coil winder"

// ESP32 LCD pins
#define LCD_RD  2
#define LCD_WR  4
#define LCD_RS 15
#define LCD_CS 33
#define LCD_RST 32

#define LCD_D0 12
#define LCD_D1 13
#define LCD_D2 26
#define LCD_D3 25
#define LCD_D4 17
#define LCD_D5 16
#define LCD_D6 27
#define LCD_D7 14

const int XP=13,XM=33,YP=15,YM=12; //320x480 ID=0x9488
const int TS_LEFT=129,TS_RT=945,TS_TOP=45,TS_BOT=986;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

Adafruit_GFX_Button coilWidth_btn, coilOffset_btn, wireGauge_btn, revolutions_btn, speed_btn, spacing_btn, main_btn, setup_btn, start_btn, stop_btn;
bool is_main_screen = 0;
bool redraw = 0;

int pixel_x, pixel_y;     //Touch_getXY() updates global vars

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


// button pin
const uint8_t buttonPin = 19;
bool buttonState = 0;  // variable for reading the pushbutton status
bool isHomeCycle = 0; // is home cycle running

// number chars pixel coordinates
const uint8_t numberCharsCoords[80][3] = {
  {2, 0, 0}, {3, 0, 0}, {4, 0, 0}, {5, 0, 0}, {6, 0, 0}, {7, 0, 0}, {2, 1, 0}, {3, 1, 0}, {4, 1, 0}, {5, 1, 0}, 
  {6, 1, 0}, {7, 1, 0}, {0, 2, 1}, {1, 2, 1}, {8, 2, 2}, {9, 2, 2}, {0, 3, 1}, {1, 3, 1}, {8, 3, 2}, {9, 3, 2},
  {0, 4, 1}, {1, 4, 1}, {8, 4, 2}, {9, 4, 2}, {0, 5, 1}, {1, 5, 1}, {8, 5, 2}, {9, 5, 2}, {0, 6, 1}, {1, 6, 1},
  {8, 6, 2}, {9, 6, 2}, {2, 7, 3}, {3, 7, 3}, {4, 7, 3}, {5, 7, 3}, {6, 7, 3}, {7, 7, 3}, {2, 8, 3}, {3, 8, 3},
  {4, 8, 3}, {5, 8, 3}, {6, 8, 3}, {7, 8, 3}, {0, 9, 4}, {1, 9, 4}, {8, 9, 5}, {9, 9, 5}, {0,10, 4}, {1,10, 4},
  {8,10, 5}, {9,10, 5}, {0,11, 4}, {1,11, 4}, {8,11, 5}, {9,11, 5}, {0,12, 4}, {1,12, 4}, {8,12, 5}, {9,12, 5},
  {0,13, 4}, {1,13, 4}, {8,13, 5}, {9,13, 5}, {0,14, 4}, {1,14, 4}, {8,14, 5}, {9,14, 5}, {2,15, 6}, {3,15, 6}, 
  {4,15, 6}, {5,15, 6}, {6,15, 6}, {7,15, 6}, {2,16, 6}, {3,16, 6}, {4,16, 6}, {5,16, 6}, {6,16, 6}, {7,16, 6}
};


// number chars
const bool numberChars[10][7] = {// 10 numbers, 7 segments
  {1, 1, 1, 0, 1, 1, 1},//0
  {0, 0, 1, 0, 0, 1, 0},//1
  {1, 0, 1, 1, 1, 0, 1},//2
  {1, 0, 1, 1, 0, 1, 1},//3
  {0, 1, 1, 1, 0, 1, 0},//4
  {1, 1, 0, 1, 0, 1, 1},//5
  {1, 1, 0, 1, 1, 1, 1},//6
  {1, 0, 1, 0, 0, 1, 0},//7
  {1, 1, 1, 1, 1, 1, 1},//8
  {1, 1, 1, 1, 0, 1, 1}//9
};

// Motor Connections
const uint8_t stepPin1 = 21;
const uint8_t dirPin2 = 22;
const uint8_t stepPin2 = 23;
const float cycleDelay = 108;
const uint8_t pulseWidth = 10; // microseconds between digitalWrite() high and low
const unsigned int stepper1StepsPerRevolution = 400;
const unsigned int stepper2StepsPerRevolution = 20*16; // common DVD stepper with 1/16 microstep
const double stepper2StepsPerMillimeter = stepper2StepsPerRevolution / 3; // motor 2 steps per 1 mm. common DVD stepper with 1/16 microstep
const unsigned int maxSpeed = 850; // max possible speed
const unsigned int accelerationDuration = 2000; // acceleration duration in seconds
const unsigned int maxDelay = 5000; // max delay in microseconds between steps

int getDelay(int spd) {
  return round((1.0 / (double(spd) * 400.0 / 60.0 / 1000000.0)) - cycleDelay);
}

bool steppersMoving = 0; // don't read touchscreen then. only read button pin
unsigned long stepper1CurrentPosition; // main motor current position in steps
bool currentDir = 0; // 0 right, 1 left
unsigned long revolutions = 0; // motor 1 revolutions counter
unsigned int speed = 800; // target revolutions per minute
unsigned int accelerationStart = 0; // accelleration start in milliseconds
unsigned int currentDelay = maxDelay; // current delay in microseconds. Starting movement always from this speed
unsigned int minDelay = getDelay(speed); // min delay in microseconds = max speed
unsigned long prevRevMsec = 0; // previous revolution milliseconds
uint8_t stepper2StepsToGo = 0; // how many stepper2 steps to do on this iteration

// variables editable in GUI
unsigned long totalRevolutions = 5000; // total turns, default 5000 humbacker coil
float coilWidth = 7.2; // coil width in millimeters, default 7.2 humbacker coil
float coilOffset = 1.5; // coil offset from 0
float wireGauge = 0.056; // wire gauge in millimeters, default AWG43 = 0.056 mm
float spacing = wireGauge; // distance between wire centers in millimeters, default wireGauge
uint8_t strategy = 0; // TODO: different motor 2 move strategies, default 0 = plain wire to wire
unsigned long stepper2CurrentPosition = 0; // motor 2 current position in steps
unsigned long stepper2CurrentLayerPosition = 0; // motor 2 current position in steps within a layer, decrement when moving left.

// Recalculate this on wire gauge and coil width set!
double stepper1StepsPerMillimeter;
unsigned long stepper1StepsTotalWidth;
unsigned long stepper2StepsTotalWidth;
double stepper2StepsPerStepper1Step;
void recalculateIt() {
  stepper1StepsPerMillimeter = double(stepper1StepsPerRevolution) / double(spacing); // motor 1 steps per millimeter.
  stepper1StepsTotalWidth = long(stepper1StepsPerMillimeter * double(coilWidth - wireGauge)); // total steps per layer for stepper1
  stepper2StepsTotalWidth = long(stepper2StepsPerMillimeter * double(coilWidth - wireGauge)); // total steps per layer for stepper2
  stepper2StepsPerStepper1Step = double(stepper2StepsTotalWidth) / double(stepper1StepsTotalWidth); // stepper2 steps per stepper1 step
}

// Recalculate this on offset set
unsigned int offsetSteps = int(double(coilOffset) * stepper2StepsPerMillimeter); // steps to move after homing

// char array for enter values
// char keyarray[10];
String keyarray = "";
// touch
bool down = 0;
char buffer[10];

uint16_t readID(void) {
    uint16_t ID = tft.readID();
    if (ID == 0xD3D3) ID = 0x9486;
    return ID;
}
#define TFT_BEGIN()  tft.begin(ID)

void setup() {
  // serial. remove after complete debugging
  Serial.begin(115200);
  mySensor.begin();
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(7114.7368421);
  scale.tare(); //Reset the scale to 0

  recalculateIt();
  // pins setup
  pinMode(stepPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(buttonPin, INPUT);

  // start task on second core
  xTaskCreatePinnedToCore(
    taskForCore2,   /* Function to implement the task */
    "taskForCore2", /* Name of the task */
    10000,          /* Stack size in words */
    NULL,           /* Task input parameter */
    0,              /* Priority of the task */
    NULL,           /* Task handle. */
    0               /* Core where the task should run */
  );

  //start TFT
  uint16_t ID = readID();
  TFT_BEGIN();
  tft.setRotation(0);

  delay(2000);

  // draw splash screen for setup
  drawSplash();

  // draw main screen
  drawMainScreen();

  // home cycle for stepper2
  home();
  scale.tare(); //Reset the scale to 0
}

void loop() {
  if(is_main_screen) {

    if(steppersMoving) {
      drawCounter(revolutions);
      if(scale.is_ready()) {
        mySensor.reading(scale.get_units());
      }
      printWireTension();
    } else {
      // read touch screen
      if(Touch_getXY()) {
        if(pixel_x > 210 && pixel_x < 310) {
          if(pixel_y > 61 && pixel_y < 96) {
            coilWidth_btn.drawButton(true);
            enterNewValue(0);
          }
          if(pixel_y > 106 && pixel_y < 141) {
            coilOffset_btn.drawButton(true);
            enterNewValue(1);
          }
          if(pixel_y > 151 && pixel_y < 186) {
            wireGauge_btn.drawButton(true);
            enterNewValue(2);
          }
          if(pixel_y > 196 && pixel_y < 231) {
            revolutions_btn.drawButton(true);
            enterNewValue(3);
          }
          if(pixel_y > 241 && pixel_y < 276) {
            speed_btn.drawButton(true);
            enterNewValue(4);
          }
          if(pixel_y > 286 && pixel_y < 321) {
            spacing_btn.drawButton(true);
            enterNewValue(5);
          }
        }
      }           
    }

    if(redraw) {//just stopped, redraw counter last time
      drawCounter(totalRevolutions);
      redraw = 0;
    }
  }
}

void taskForCore2( void * pvParameters ) {
  while(true) {
    buttonState = digitalRead(buttonPin);
    if(buttonState == HIGH && !isHomeCycle && is_main_screen) {
      // wait for button release
      while(digitalRead(buttonPin) == HIGH) {
        delay(50);//do nothing
      }
      
      steppersMoving = !steppersMoving;

      if(steppersMoving) {
        // just started, update acceleration
        accelerationStart = millis();
      }
    }

    //reset speed to minimal
    if(!steppersMoving) {
      currentDelay = maxDelay;
    }

    if(steppersMoving) {
      step();
    } 
  }
}

void drawSplash() {
  is_main_screen = 0;
  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setCursor(94, 20);
  tft.setTextColor(WHITE, BLACK);
  tft.println("COIL WINDER");
  tft.drawFastHLine(10, 55, 300, WHITE);

  main_btn.initButton(&tft,  160, 170, 300, 40, WHITE, BLUE, BLACK, "WIND", 2);
  main_btn.drawButton();

  setup_btn.initButton(&tft,  160, 250, 300, 40, WHITE, BLUE, BLACK, "POSITION", 2);
  setup_btn.drawButton();
  while(true) {
    down = Touch_getXY();
    main_btn.press(down && main_btn.contains(pixel_x, pixel_y));
    if (main_btn.justPressed()) {
        main_btn.drawButton(true);
        break;
    }
    setup_btn.press(down && setup_btn.contains(pixel_x, pixel_y));
    if (setup_btn.justPressed()) {
        setup_btn.drawButton(true);
        setupHome();
        break;
    }
  }
}

void setupHome() {
  is_main_screen = 0;
  tft.fillScreen(BLACK);
  tft.setTextSize(5);
  tft.fillRoundRect(220, 410, 80, 60, 15, BLUE);
  tft.drawRoundRect(220, 410, 80, 60, 15, WHITE);
  tft.setTextColor(BLACK, BLUE);
  tft.setCursor(235, 422);
  tft.println(">");
  tft.fillRoundRect(120, 410, 80, 60, 15, GREEN);
  tft.drawRoundRect(120, 410, 80, 60, 15, WHITE);
  tft.setTextColor(BLACK, GREEN);
  tft.setCursor(132, 422);
  tft.println("OK");
  tft.fillRoundRect(20, 410, 80, 60, 15, BLUE);
  tft.drawRoundRect(20, 410, 80, 60, 15, WHITE);
  tft.setTextColor(BLACK, BLUE);
  tft.setCursor(47, 422);
  tft.println("<");
  while(1) {
    if(Touch_getXY()) {
      if(pixel_y > 410 && pixel_y < 470) { // bottom buttons line
        if(pixel_x > 20 && pixel_x < 100) { // <
          digitalWrite(dirPin2, HIGH);
          digitalWrite(stepPin2, HIGH);
          delayMicroseconds(pulseWidth);
          digitalWrite(stepPin2, LOW);
          delay(10);
        }
        if(pixel_x > 120 && pixel_x < 200) { // OK
          break;
        }
        if(pixel_x > 220 && pixel_x < 300) { // >
          digitalWrite(dirPin2, LOW);
          digitalWrite(stepPin2, HIGH);
          delayMicroseconds(pulseWidth);
          digitalWrite(stepPin2, LOW);
          delay(10);
        }
      }
    }
  }
}

void drawMainScreen() {
  is_main_screen = 1;
  tft.fillScreen(BLACK);
  tft.setTextSize(4);
  tft.setCursor(30, 11);
  tft.setTextColor(WHITE, BLACK);
  tft.println("TURNS: ");
  drawCounter(revolutions);
  tft.drawFastHLine(10, 55, 300, WHITE);

  tft.setTextSize(2);
  tft.setCursor(10, 72);
  tft.setTextColor(WHITE, BLACK);
  tft.println("COIL WIDTH mm:");
  coilWidth_btn.initButtonUL(&tft,  210, 61, 100, 35, CYAN, WHITE, BLACK, dtostrf(coilWidth, 3, 2, buffer), 2);
  coilWidth_btn.drawButton();

  tft.setCursor(10, 117);
  tft.setTextColor(WHITE, BLACK);
  tft.println("OFFSET mm:");
  coilOffset_btn.initButtonUL(&tft,  210, 106, 100, 35, CYAN, WHITE, BLACK, dtostrf(coilOffset, 2, 2, buffer), 2);
  coilOffset_btn.drawButton();

  tft.setCursor(10, 162);
  tft.setTextColor(WHITE, BLACK);
  tft.println("WIRE GAUGE mm:");
  wireGauge_btn.initButtonUL(&tft,  210, 151, 100, 35, CYAN, WHITE, BLACK, dtostrf(wireGauge, 0, 3, buffer), 2);
  wireGauge_btn.drawButton();

  tft.setCursor(10, 207);
  tft.setTextColor(WHITE, BLACK);
  tft.println("TOTAL TURNS:");
  revolutions_btn.initButtonUL(&tft,  210, 196, 100, 35, CYAN, WHITE, BLACK, dtostrf(totalRevolutions, 5, 0, buffer), 2);
  revolutions_btn.drawButton();

  tft.setCursor(10, 252);
  tft.setTextColor(WHITE, BLACK);
  tft.println("SPEED rev/min:");
  speed_btn.initButtonUL(&tft,  210, 241, 100, 35, CYAN, WHITE, BLACK, dtostrf(speed, 5, 0, buffer), 2);
  speed_btn.drawButton();

  tft.setCursor(10, 297);
  tft.setTextColor(WHITE, BLACK);
  tft.println("WIRE SPACING mm:");
  spacing_btn.initButtonUL(&tft,  210, 286, 100, 35, CYAN, WHITE, BLACK, dtostrf(spacing, 0, 3, buffer), 2);
  spacing_btn.drawButton();
  
  tft.drawFastHLine(10, 342, 300, WHITE);

  tft.setTextSize(2);
  tft.setCursor(10, 355);
  tft.setTextColor(WHITE, BLACK);
  tft.println("WIRE TENSION g:");
}

void printWireTension() {
  tft.setTextSize(2);
  tft.setCursor(200, 355);
  tft.setTextColor(WHITE, BLACK);
  tft.print(int(mySensor.getAvg()));
  tft.println("   ");
}

void step() {
  // move main motor
  digitalWrite(stepPin1, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(stepPin1, LOW);
  stepper1CurrentPosition++;

  // calculate motor2 steps
  stepper2StepsToGo = uint8_t(long(stepper1CurrentPosition * stepper2StepsPerStepper1Step) - stepper2CurrentPosition);

  
  // do actual stepper2 steps if any
  for(uint8_t i = stepper2StepsToGo; i > 0; i--) {
    delayMicroseconds(pulseWidth);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(stepPin2, LOW);
    stepper2CurrentPosition++;

    if(currentDir == 0) { // direction right
      stepper2CurrentLayerPosition++;
    } else { // direction left
      stepper2CurrentLayerPosition--;
    }

    // switch direction
    if(currentDir == 0 && stepper2CurrentLayerPosition >= stepper2StepsTotalWidth) {
      digitalWrite(dirPin2, HIGH);
      currentDir = 1;
    }
    if(currentDir == 1 && stepper2CurrentLayerPosition <= 0) {
      digitalWrite(dirPin2, LOW);
      currentDir = 0;
    }
  }

  // revolutions counter and speed
  if(stepper1CurrentPosition % stepper1StepsPerRevolution == 0) {
    revolutions++;
  }

  // delay
  if(currentDelay > 0) {
    delayMicroseconds(currentDelay);
  }

  //acceleration for next step
  if(currentDelay > minDelay) {
    currentDelay = accelerate();
  }

  // stop moving
  if(stepper1CurrentPosition >= (totalRevolutions * stepper1StepsPerRevolution)) {
    steppersMoving = 0;
    // redraw main screen
    redraw = 1;

    // then move to home position
    home();

    // and reset counter value
    revolutions = 0;
    stepper1CurrentPosition = 0;
  }
}

int accelerate() {
  // Sigmoid function for speed change depending on milliseconds from move start
  float multiplier = 1/(1+(pow(1000000, -1*((float(millis() - accelerationStart)/float(accelerationDuration))-0.5))));
  if(multiplier < 0.01) {
    return maxDelay;
  }
  double curSpeed = speed*multiplier;
  return getDelay(curSpeed);
}

void drawCounterPixel(unsigned int x, unsigned int y, unsigned int curRevSteps, unsigned int number) {
  int curNumber;
  uint8_t reg = uint8_t(curRevSteps / 80);
  // from left to right
  // from right to left
  switch(reg) {
    case 0:
      x += 96; // units
      curNumber = number%10;
      break;
    case 1: // tens
      x += 72;
      curRevSteps = curRevSteps - 80;
      curNumber = (number/10)%10;
      break;
    case 2: // hundreds
      x += 48;
      curRevSteps = curRevSteps - 160;
      curNumber = (number/100)%10;
      break;
    case 3: // thousands
      x += 24;
      curRevSteps = curRevSteps - 240;
      curNumber = (number/1000)%10;
      break;
    case 4: // tens of thousands
      curRevSteps = curRevSteps - 320;
      curNumber = (number/10000)%10;
      break;
  }
  tft.drawPixel(x + (numberCharsCoords[curRevSteps][0] * 2), y + (numberCharsCoords[curRevSteps][1] * 2), numberChars[curNumber][numberCharsCoords[curRevSteps][2]] ? WHITE : BLACK);
  tft.drawPixel(x + (numberCharsCoords[curRevSteps][0] * 2) + 1, y + (numberCharsCoords[curRevSteps][1] * 2), numberChars[curNumber][numberCharsCoords[curRevSteps][2]] ? WHITE : BLACK);
  tft.drawPixel(x + (numberCharsCoords[curRevSteps][0] * 2), y + (numberCharsCoords[curRevSteps][1] * 2) + 1, numberChars[curNumber][numberCharsCoords[curRevSteps][2]] ? WHITE : BLACK);
  tft.drawPixel(x + (numberCharsCoords[curRevSteps][0] * 2) + 1, y + (numberCharsCoords[curRevSteps][1] * 2) + 1, numberChars[curNumber][numberCharsCoords[curRevSteps][2]] ? WHITE : BLACK);
}

void drawCounter(unsigned long revs) {
  for(int i=0; i < 400; i++) {
    drawCounterPixel(172, 8, i, revs);
  }
}

void home() {
  isHomeCycle = 1;
  // move left until endstop is reached
  digitalWrite(dirPin2, HIGH);
  delayMicroseconds(pulseWidth);
  while(digitalRead(buttonPin) == LOW) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(pulseWidth*2);
    digitalWrite(stepPin2, LOW);
    delay(1);
  }

  // move right to offset position
  digitalWrite(dirPin2, LOW);
  delayMicroseconds(pulseWidth);
  for(int i = 0; i <= offsetSteps; i++) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(pulseWidth*2);
    digitalWrite(stepPin2, LOW);
    delay(1);
  }

  // reset initial values
  stepper2CurrentPosition = 0;
  stepper2CurrentLayerPosition = 0;
  currentDir = 0;

  // wait for button release
  while(digitalRead(buttonPin) == HIGH) {
    delay(50);//do nothing
  }
  isHomeCycle = 0;
}

// 0 = coilWidth
// 1 = coilOffset
// 2 = wireGauge
// 3 = totalRevolutions
// 4 = speed
// 5 = spacing

void enterNewValue(uint8_t varName) {
  is_main_screen = 0;
  switch(varName) {
    case 0:
      keyarray = String(coilWidth);
      break;
    case 1:
      keyarray = String(coilOffset);
      break;
    case 2:
      keyarray = String(wireGauge, 3);
      break;
    case 3:
      keyarray = String(totalRevolutions);
      break;
    case 4:
      keyarray = String(speed);
      break;
    case 5:
      keyarray = String(spacing, 3);
      break;
  }
  // clean diaplay
  tft.fillScreen(BLACK);
  //draw rectangle for typing
  tft.fillRoundRect(20, 10, 280, 80, 10, WHITE);

  // draw buttons
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(5);
  tft.fillRoundRect(20, 330, 180, 60, 15, BLUE);
  tft.drawRoundRect(20, 330, 180, 60, 15, WHITE);
  tft.setCursor(100, 342);
  tft.println("0");
  tft.fillRoundRect(20, 255, 80, 60, 15, BLUE);
  tft.drawRoundRect(20, 255, 80, 60, 15, WHITE);
  tft.setCursor(45, 267);
  tft.println("1");
  tft.fillRoundRect(120, 255, 80, 60, 15, BLUE);
  tft.drawRoundRect(120, 255, 80, 60, 15, WHITE);
  tft.setCursor(145, 267);
  tft.println("2");
  tft.fillRoundRect(220, 255, 80, 60, 15, BLUE);
  tft.drawRoundRect(220, 255, 80, 60, 15, WHITE);
  tft.setCursor(245, 267);
  tft.println("3");
  tft.fillRoundRect(20, 180, 80, 60, 15, BLUE);
  tft.drawRoundRect(20, 180, 80, 60, 15, WHITE);
  tft.setCursor(45, 192);
  tft.println("4");
  tft.fillRoundRect(120, 180, 80, 60, 15, BLUE);
  tft.drawRoundRect(120, 180, 80, 60, 15, WHITE);
  tft.setCursor(145, 192);
  tft.println("5");
  tft.fillRoundRect(220, 180, 80, 60, 15, BLUE);
  tft.drawRoundRect(220, 180, 80, 60, 15, WHITE);
  tft.setCursor(245, 192);
  tft.println("6");
  tft.fillRoundRect(20, 105, 80, 60, 15, BLUE);
  tft.drawRoundRect(20, 105, 80, 60, 15, WHITE);
  tft.setCursor(45, 117);
  tft.println("7");
  tft.fillRoundRect(120, 105, 80, 60, 15, BLUE);
  tft.drawRoundRect(120, 105, 80, 60, 15, WHITE);
  tft.setCursor(145, 117);
  tft.println("8");
  tft.fillRoundRect(220, 105, 80, 60, 15, BLUE);
  tft.drawRoundRect(220, 105, 80, 60, 15, WHITE);
  tft.setCursor(245, 117);
  tft.println("9");
  tft.fillRoundRect(220, 330, 80, 60, 15, BLUE);
  tft.drawRoundRect(220, 330, 80, 60, 15, WHITE);
  tft.setCursor(245, 332);
  tft.println(".");
  tft.fillRoundRect(220, 410, 80, 60, 15, GREEN);
  tft.drawRoundRect(220, 410, 80, 60, 15, WHITE);
  tft.setTextColor(BLACK, GREEN);
  tft.setCursor(235, 422);
  tft.println("OK");
  tft.fillRoundRect(120, 410, 80, 60, 15, YELLOW);
  tft.drawRoundRect(120, 410, 80, 60, 15, WHITE);
  tft.setTextColor(BLACK, YELLOW);
  tft.setCursor(140, 422);
  tft.println("<");
  tft.fillRoundRect(20, 410, 80, 60, 15, RED);
  tft.drawRoundRect(20, 410, 80, 60, 15, WHITE);
  tft.setTextColor(BLACK, RED);
  tft.setCursor(47, 422);
  tft.println("X");
  
  while(1) {
    if(Touch_getXY()) {
      if(pixel_y > 410 && pixel_y < 470) { // bottom buttons line
        if(pixel_x > 20 && pixel_x < 100) { // cancel button. do nothing and return to main screen
          drawMainScreen();
          keyarray = "";
          break;
        }
        if(pixel_x > 120 && pixel_x < 200) { // backspace button. clear last symbol
          keyarray.remove(keyarray.length() - 1, 1);
          tft.fillRect(32 + (keyarray.length() * 48), 25, 48, 60, WHITE);
        }
        if(pixel_x > 220 && pixel_x < 300) { // OK button. store value if any and return to main screen
          if(keyarray.length() == 0) continue;
          // store variable
          switch(varName) {
            case 0:
              coilWidth = keyarray.toFloat();
              recalculateIt();
              break;
            case 1:
              coilOffset = keyarray.toFloat();
              offsetSteps = int(double(coilOffset) * stepper2StepsPerMillimeter);
              // move to new home position
              home();
              break;
            case 2:
              wireGauge = keyarray.toFloat();
              recalculateIt();
              break;
            case 3:
              totalRevolutions = keyarray.toInt();
              break;
            case 4:
              speed = keyarray.toInt();
              if(speed > maxSpeed) speed = maxSpeed; // this is the maximum possible speed. 
              minDelay = getDelay(speed);
              break;
            case 5:
              spacing = keyarray.toFloat();
              recalculateIt();
              break;
          }
          drawMainScreen();
          break;
        }
      }
      if(pixel_y > 105 && pixel_y < 165) { // first line
        if(pixel_x > 20 && pixel_x < 100) {
          addchar('7');
        }
        if(pixel_x > 120 && pixel_x < 200) {
          addchar('8');
        }
        if(pixel_x > 220 && pixel_x < 300) {
          addchar('9');
        }
      }
      if(pixel_y > 180 && pixel_y < 240) { // second line
        if(pixel_x > 20 && pixel_x < 100) {
          addchar('4');
        }
        if(pixel_x > 120 && pixel_x < 200) {
          addchar('5');
        }
        if(pixel_x > 220 && pixel_x < 300) {
          addchar('6');
        }
      }
      if(pixel_y > 255 && pixel_y < 315) { // third line
        if(pixel_x > 20 && pixel_x < 100) {
          addchar('1');
        }
        if(pixel_x > 120 && pixel_x < 200) {
          addchar('2');
        }
        if(pixel_x > 220 && pixel_x < 300) {
          addchar('3');
        }
      }
      if(pixel_y > 330 && pixel_y < 390) { // fourh line
        if(pixel_x > 20 && pixel_x < 200) {
          addchar('0');
        }
        if(pixel_x > 220 && pixel_x < 300) {
          addchar('.');
        }
      }
      delay(200);
    }

    tft.setTextColor(BLACK, WHITE);
    tft.setCursor(40, 25);
    tft.setTextSize(8);
    tft.println(keyarray);
  }
  
  // reset touch
  pixel_x = 0;
  pixel_y = 0;
  down = 0;
  // reset temporary value string
  keyarray = "";
}

void addchar(char inchar)
{
  if(keyarray.length() > 4) {
    return;
  }
  keyarray += inchar;
}

bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
    }
    return pressed;
}