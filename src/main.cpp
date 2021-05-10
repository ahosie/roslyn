#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include "rotary_encoder.h"
#include "main.h"

#define ROTATE_CW_PULSE 2
#define ROTATE_CCW_PULSE 3
#define BUTTONPAD_PIN A0

#define DIGIPOT_CS 4
//#define DIGIPOT_CLOCK 5
//#define DIGIPOT_DI 6

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI:
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

static Configuration config;
static ApplicationState state;

inline void initialise()
{
  state.mode = ApplicationMode::initialising;
  state.speedPercent = 0.0;

  config.software.serialBaudRate = 115200;
  config.software.encoderDivisor = 4;

  config.hardware.digitPotMaxValue = 255;
  config.hardware.digitPotMinValue = 0;
}


//static ApplicationMode _APP_CURRENT_MODE = ApplicationMode::initialising;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

/* Comment out above, uncomment this block to use hardware SPI
 */
/*
#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);
*/
static uint16_t uintClockwiseTriggers = 0;
static uint16_t uintCClockwiseTriggers = 0;

/** Output string buffers used to display values on the OLED
 */

// -32768[NULL]
// 1234567
static char strCWTriggerCount[7];
static char strCCWTriggerCount[7];
static char xPosStr[7];
static char yPosStr[7];
static char digipotValueStr[7];
static char tempUintConversionBuffer[7];

static char screenLowerHalfSize1Buffer[84];

static bool DoExecute = false;
static bool keyPressed = false;
static int jitterCount = 0;

long int BUTTON_TRIGGER_POINTS[][2] = {
  {-1, 20},
  {120, 130},
  {280, 300},
  {440, 450},
  {650, 660},
  {900, 920},
  {65534, 65535}
};

static uint32_t plunge_offset_micrometers = 0;
static uint32_t surface_meters_per_minute = 0;

static int buttonLevel;
static char buttonLevelStr[10];

static Button buttonState = Button::unknown;

#define KEYSTATE_BUFFER_LENGTH 8
static Button keyStateBuffer[KEYSTATE_BUFFER_LENGTH] = {};

static int8_t keyStateBufferIndex = 0;
static uint8_t xPos = 0, yPos = 0;
static Button lastDebouncedKey = Button::unknown;

static Direction spinDirection = Direction::clockwise;

static uint8_t triggerToggle = 0;


static uint16_t encoderPulsesPerRevolution = 400;
//static uint16_t encoderDivisor = 4; // Scale down the rate of pulses received from the rotary encoder by this figure

static uint16_t pendingDigipotNextValue = 0;
static uint16_t currentDigipotValue = 0;

#define KEYSTROKE_BUFFER_SIZE 8
static Button keystrokeBuffer[KEYSTROKE_BUFFER_SIZE] = {
  Button::none,
  Button::none,
  Button::none,
  Button::none,
  Button::none,
  Button::none,
  Button::none,
  Button::none
};
static int keyLogIndex = -1;

void digitalPotWrite(int value) {
  // take the SS pin low to select the chip:
  digitalWrite(DIGIPOT_CS, LOW);

  //  send in the address and value via SPI:
  SPI.transfer(0b00010001);
  SPI.transfer(value);
  
  // take the SS pin high to de-select the chip:
  digitalWrite(DIGIPOT_CS, HIGH);
}

inline char buttonCharacterRepresentation(Button buttonState)
{
  switch(buttonState)
  {
    case Button::left:
      return 17; // ◄
    case Button::up:
      return 30; // ▲
    case Button::down:
      return 31; // ▼
    case Button::right:
      return 16; // ►
    case Button::exec:
      return 19; // ‼
    case Button::unknown:
      return 63; // ?
    case Button::none:
      return ' ';
    default:
      return 168; // ¿
  }
}

void IVR_KNOB_CCW()
{
  uintCClockwiseTriggers += 1;
  Direction newDirection = (Direction) (digitalRead(ROTATE_CW_PULSE) == HIGH);
  if(newDirection == spinDirection)
  {
    // Only sample one in {encoderDivisor} pulses (increment by 100 per full rotation)
    if(uintCClockwiseTriggers % config.software.encoderDivisor == 0)
    {
      if(spinDirection == Direction::clockwise)
      {
        if(pendingDigipotNextValue < config.hardware.digitPotMaxValue)
        {
          pendingDigipotNextValue++;
        }
      }
      else
      {
        if(pendingDigipotNextValue > config.hardware.digitPotMinValue)
        {
          pendingDigipotNextValue--;
        }
      }

      state.speedPercent = 100.0 * ((float)pendingDigipotNextValue) / ((float) config.hardware.digitPotMaxValue);
    }
  }
  spinDirection = newDirection;
}

void IVR_KNOB_CW(){
  uintClockwiseTriggers += 1;
}

void setup() {
  initialise();
  Serial.begin(config.software.serialBaudRate);
  
  pinMode(ROTATE_CCW_PULSE, INPUT_PULLUP);
  pinMode(ROTATE_CW_PULSE, INPUT_PULLUP);
  pinMode(BUTTONPAD_PIN, INPUT);  
  pinMode(DIGIPOT_CS, OUTPUT);

  // set the slaveSelectPin as an output:
  pinMode(DIGIPOT_CS, OUTPUT);
  digitalWrite(DIGIPOT_CS, HIGH);

  // initialize SPI:
  SPI.begin();
  
  attachInterrupt(digitalPinToInterrupt(ROTATE_CCW_PULSE), IVR_KNOB_CCW, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTATE_CW_PULSE), IVR_KNOB_CW, RISING);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println("Init complete");

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setTextSize(2);
  display.setCursor(32, 32);
  display.write("READY");
  display.display();

  state.mode = ApplicationMode::speed_controller;
  //state.mode = 153;
}

inline void updateButtonState()
{
  buttonLevel = analogRead(BUTTONPAD_PIN);
  for(int buttonEnumIdx = 0; buttonEnumIdx < (int)Button::unknown; buttonEnumIdx++)
  {
    if(buttonLevel >= BUTTON_TRIGGER_POINTS[buttonEnumIdx][0] && buttonLevel <= BUTTON_TRIGGER_POINTS[buttonEnumIdx][1])
    {
      buttonState = (Button)buttonEnumIdx;
    }
  }

  keyStateBufferIndex++;
  if(keyStateBufferIndex >= KEYSTATE_BUFFER_LENGTH)
  {
    keyStateBufferIndex = 0;
    //digitalWrite(DIGIPOT_CS, (triggerToggle++ % 2 == 1) ? HIGH : LOW);
  }
  keyStateBuffer[keyStateBufferIndex] = buttonState;

  // TODO figure out why this goddamn debounce isn't working
  jitterCount = 0;
  for(int i = 0; i < KEYSTATE_BUFFER_LENGTH; i++)
  {
    if(keyStateBuffer[keyStateBufferIndex] != buttonState)
    {
      jitterCount++;
    }
  }
  keyPressed = (jitterCount == 0);
  if(keyPressed && buttonState != lastDebouncedKey)
  {
    state.executePressedThisLoop = buttonState == Button::exec;

    switch(buttonState)
    {
      case Button::left:
        xPos--;
        break;
      case Button::up:
        yPos++;
        break;
      case Button::down:
        yPos--;
        break;
      case Button::right:
        xPos++;
        break;
      default:
        break;
    }

    if(buttonState != Button::none)
    {
      if(++keyLogIndex >= KEYSTROKE_BUFFER_SIZE)
      {
        keyLogIndex = 0;
      }
      keystrokeBuffer[keyLogIndex] = buttonState;
    }

    lastDebouncedKey = buttonState;
  }
}

// 100.00 %[NULL]
// 123456789
static char strSpeed[9];
//digitPotMaxValue 

inline void APP_SPEED_CONTROLLER()
{
  if(state.executePressedThisLoop)
  {
    currentDigipotValue = pendingDigipotNextValue;

    // TODO perform a smooth step?
    digitalPotWrite(currentDigipotValue);
  }

  utoa(uintCClockwiseTriggers, strCWTriggerCount, 10);
  utoa(uintClockwiseTriggers, strCCWTriggerCount, 10);
  utoa(buttonLevel, buttonLevelStr, 10);
  utoa(pendingDigipotNextValue, digipotValueStr, 10);
  utoa(xPos, xPosStr, 10);
  utoa(yPos, yPosStr, 10);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);                 // Use full 256 char 'Code Page 437' font
  display.setTextSize(2);
  display.setCursor(0, 0);

  /*
  switch(spinDirection)
  {
    case Direction::clockwise:
      display.write("CW  ");
      break;
    case Direction::counterclockwise:
      display.write("CCW ");
      break;
  }

  display.write(0xED); // 237 0xED φ
  
  if(!keyPressed)
  {
    display.write(' ');
  }
  else if(jitterCount > 0)
  {
    display.write('b');
  }
  else
  {
    display.write(buttonCharacterRepresentation(buttonState));
  }

  display.setCursor(0, 0);
  display.write(digipotValueStr);
  */
  //sprintf(strSpeed, "%X", (unsigned int)state.speedPercent);
  //sprintf(strSpeed, "%f", state.speedPercent);
  sprintf(strSpeed, "%f", 100.0 * float(pendingDigipotNextValue) / float(config.hardware.digitPotMaxValue));
  //sprintf(strSpeed, "%i", (int)(state.speedPercent * 100.0));


  display.write(strSpeed);

  display.write("    ");
  display.write(digipotValueStr);
  display.write("  ");
  display.write(buttonCharacterRepresentation(buttonState));
  
  display.setTextSize(1);
  display.setCursor(64, 16);
  display.write(strCWTriggerCount);
  display.setCursor(64, 24);
  display.write(strCCWTriggerCount);

  //display.drawPixel(64, 32, WHITE);
  display.setCursor(64, 32);
  display.setTextSize(1);
  display.write(buttonLevelStr);

  display.setCursor(0, 48);

  if(keyLogIndex >= 0)
  {
    for(int relativeIndex = 0; relativeIndex < KEYSTROKE_BUFFER_SIZE; relativeIndex++)
    {
      int absoluteIndex = keyLogIndex + relativeIndex + 1;
      display.write(buttonCharacterRepresentation(keystrokeBuffer[absoluteIndex % KEYSTROKE_BUFFER_SIZE]));
    }

    display.setCursor(64, 48);
    display.write(xPosStr);
    display.write(", ");
    display.write(yPosStr);
  }
  else
  {
    display.write("[awaiting input]");
  }

  display.display();
}

inline void HCF(char* title, char* message)
{
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);

      // Not implemented in the SSD1306 library
      //display.invertDisplay(true);
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.write(title);

      display.invertDisplay(false);
      display.setTextSize(1);
      display.setCursor(0, 16);
      display.write(message);

      display.display();
      for(;;); // Don't proceed, loop forever
}

void loop()
{
  updateButtonState();

  switch(state.mode)
  {
    case ApplicationMode::initialising:
      HCF("ERROR", "App mode is initialising within loop");
      break;

    case ApplicationMode::speed_controller:
      APP_SPEED_CONTROLLER();
      break;

    default:
      sprintf(screenLowerHalfSize1Buffer, "illegal state: %u", state.mode);
      HCF("ERROR", screenLowerHalfSize1Buffer);
      break;
  }

  delay(16);
}
