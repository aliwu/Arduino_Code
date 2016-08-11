#include <Adafruit_NeoPixel.h>


#define ledPin   6
#define ledCount 24
#define buttonA  0
#define buttonB  1
#define buttonC  2
#define buttonD  3
#define maxSpeed 1        //Lower is faster
#define minSpeed 1000     //Higher is slower
#define maxBrightness 100
#define minBrightness 10
#define cycleNum      7

Adafruit_NeoPixel strip = Adafruit_NeoPixel(ledCount, ledPin, NEO_GRB + NEO_KHZ800);

int rxPin[]         = {5, 4, 3, 2};  //Refers to D0,D1,D2,D3 from the receiver
int rxNum           = 4;             //Number of signal pins in the receiver

unsigned long debounceTimerA = 0;    //Timer to debounce keys
unsigned long debounceTimerB = 0;    //Timer to debounce keys
unsigned long debounceTimerC = 0;    //Timer to debounce keys
unsigned long debounceTimerD = 0;    //Timer to debounce keys

unsigned long debounceTime  = 250;  //How long to wait before clearing a flag
int           limitFlagC    = 0;    //When min max is reached
int           limitFlagD    = 0;
int           pressedFlagC  = 0;    //Flags when a button WAS pressed
int           pressedFlagD  = 0;

int speedIncrement = (minSpeed - maxSpeed) / 7;
int cycleSpeed      = 1000;          //Lower speed faster rpm
int cycleIntensity  = 10;
int cyclePattern    = 0;
signed int graduation = 3;            //How many LED's on at the same time

unsigned long colorTimer = 0;
uint16_t loopVar         = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);                 //Open serial comm for debugging purposes
  for (int i = 0; i < rxNum; i++) {   //Declare all pins to be input
    pinMode(rxPin[i], INPUT);
  }
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(cycleIntensity);
}

uint32_t red        = strip.Color(255, 0, 0);
uint32_t green      = strip.Color(0, 255, 0);
uint32_t blue       = strip.Color(0, 0, 255);
uint32_t hwhite     = strip.Color(255, 255, 255);
uint32_t wwhite     = strip.Color(130, 130, 130);
uint32_t pink       = strip.Color(255, 0, 127);
uint32_t orange     = strip.Color(255, 128, 0);
uint32_t lblue      = strip.Color(51, 255, 255);
uint32_t dgreen     = strip.Color(0, 102, 0);
uint32_t purple     = strip.Color(160, 32, 240);
uint32_t x          = strip.Color(0, 255, 128);
uint32_t yellow     = strip.Color(255, 255, 0);
uint32_t violet     = strip.Color(238, 130, 238);

uint32_t colors[]   = {red, green, blue, hwhite, purple, pink, orange, lblue, dgreen, wwhite, x, yellow, violet};

int colorIndex      = 0;                                      //Counter for the color
int colorCount      = sizeof(colors) / sizeof(uint32_t);      //number of colors


void loop() {

  // put your main code here, to run repeatedly:
  if (digitalRead(rxPin[buttonA]) == 1) {                     //CASE BUTTON A
    if (timer(debounceTimerA, debounceTime)) {
      debounceTimerA = millis();
      colorIndex++;
    }
  }

  if (digitalRead(rxPin[buttonB]) == 1) {                     //CASE BUTTON A
    if (timer(debounceTimerB, debounceTime)) {
      debounceTimerB = millis();
      cyclePattern ++;
      if (cyclePattern > cycleNum) {
        cyclePattern = 0;
      }
      if (cyclePattern >= 0 && cyclePattern < 3) {
        graduation += 2;
        if (graduation > 7) {
          graduation = 3;
        }
      }
    }
  }


  if (digitalRead(rxPin[buttonC]) == 1) {                     //CASE BUTTON C
    if (timer(debounceTimerC, debounceTime)) {
      debounceTimerC = millis();
      cycleSpeed -= speedIncrement;
      if (cycleSpeed < maxSpeed) {
        cycleSpeed = maxSpeed;
        limitFlagC = 1;
      }
      if (pressedFlagC) {
        cycleSpeed     = minSpeed;
        pressedFlagC   = 0;
        limitFlagC     = 0;
      }
    }
  } else if (limitFlagC == 1) {
    pressedFlagC = 1;
  }


  if (digitalRead(rxPin[buttonD]) == 1) {                     //CASE BUTTON D
    if (timer(debounceTimerD, debounceTime)) {
      debounceTimerD = millis();
      cycleIntensity += 10;
      if (cycleIntensity > maxBrightness) {
        cycleIntensity = maxBrightness;
        limitFlagD     = 1;
      }
      if (pressedFlagD) {
        cycleIntensity = minBrightness;
        pressedFlagD   = 0;
        limitFlagD     = 0;
      }
      //if flag set to min brightness
      strip.setBrightness(cycleIntensity);
    }
  } else if (limitFlagD == 1) {
    pressedFlagD = 1;
  }


  if (colorIndex > colorCount - 1) {
    colorIndex = 0;
  }
  Serial.print("color index is ");
  Serial.print(colorIndex);
  Serial.print(" cyclePattern  ");
  Serial.print(cyclePattern);
  Serial.print(" graduation ");
  Serial.println(graduation);

  if (timer(colorTimer, cycleSpeed)) {
    colorTimer = millis();
    colorWipe(colors[colorIndex]);
    if (cyclePattern == 3) {
      colorIndex++;
    }
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t color) {
  loopVar++;
  if (loopVar > ledCount - 1) {
    loopVar = 0;
  }
  int prev = loopVar - graduation;
  if (prev < 0) {
    prev += ledCount;
  }
  strip.setPixelColor(prev, 0);
  strip.setPixelColor(loopVar, color);
  strip.show();
}

//Reads the voltage in pinNum
int rxRead(int pinNum) {
  return (digitalRead(pinNum));
}



//timer variable
bool timer(unsigned long timerVal, unsigned long timerDuration) {
  if (millis() - timerVal >= timerDuration)
    return true;
  else
    return false;
}
