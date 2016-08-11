#include <LiquidCrystal.h>
#include <math.h>
#include<Wire.h>
#include<Adafruit_PWMServoDriver.h>

#define brightnessTimer 10
#define brightnessMax 4095
#define pwmRate 150
#define gPin 7          //PWM board pin that controls green
#define rPin 11         //PWM board pin that controls red
#define yPin 15         //PWM board pin that controls yellow
#define warningTemp 45  //Temperature where red turns on
#define HEATER_ON_TIME 30  //Heater turn on time
#define HEATER_OFF_TIME 30 //Heater on time + off time actually, not just off
#define HEATER_MAX_TEMP 10 //Max temperature difference between air and heater (heater - air only, not the other way around)
#define fanMax 4095        //Max pwm output of shield aka 100% on
#define fanMin 0
#define btnRIGHT  0   //btnCase - makes it easier to read, all btn definitions are for the LCD keypad siheld
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5
#define ON        1
#define OFF       0
#define UVrelayPin 2    //Relay control pin
#define keyinPin  A0    //Analog display switch input pin
#define doorPin   11    //Pin to check door open or closed
#define heaterRelayPin 3    //Pin for heater
#define fanPin    3    //pwm fan output pin 
#define airTempPin   A3   //temperature feedback pin
#define heaterTempPin A1 // Heater temperature sensor
#define minTemp 25      //minimum temperature
#define maxTemp 70      //maximum temperature
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))  //finds the size of any array
#define tempThreshold 1 //Temperature +/- variation
#define heaterRelayTimer 5000

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  //Set up all the lcd control pins


int counter     = 0;  //Var to increase increment speed when needed
float temperature = 0;//Stores temperature
int rpm = 0;          //rpm value

int brightnessVar = 0;  //Brightness for blinking light stick
unsigned long lightTimer = 0; //Light stick timer
int orientation  = 1;         //light stick glow or dim

//light on flags indicates when a light is on or off, on means pulsing
int green = 0;
int red = 0;
int yellow = 0;
int finishCure = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();    //Adafruit shield


char *resinArr[] = {"GPR", "FR "};
char *GPR[]    = {"Clear", "Grey ", "White", "Black", "Cast ", "Flex ", "Tough"};
int GPR_temp[] = {60, 60, 60, 60, 45, 45, 45}; //Degrees C
int GPR_time[] = {60, 60, 60, 60, 120, 60, 30}; //In minutes
char *FR[]     = {"Cast ", "Flex ", "Tough"};
int FR_temp[]  = {45, 45, 45};      //Degrees C
int FR_time[]  = {120, 60, 30};     //In minutes

unsigned long millis_in_seconds = 1000;
unsigned long millis_counter = 0;
unsigned long heaterOnTimeTimer = 0;
unsigned long redLightTimer = 0;
int hoursbu;
int minutesbu;
unsigned long heaterRelayTime = 0;
//------------------------------------------
int enterFlag = 0;
int heaterOnFlag = 0;

//Manual input flags
int timeFlag = 1;
int tempFlag = 0;
int curingTemp = 25;
int hours = 0;
int minutes = 0;
int seconds = 0;

//Page number flag
int mainMenuFlag = 1;
int presetFlag = 0;
int manualFlag = 1;

//Preset page flags   was 1 0 0 0
int resinTypeFlag = 0;
int resinType = 0;
int resSelect = 0;
int resSelectFlag = 1;


//Fully turn on the fan for half a second
void fankickStart( void ) {
  setFanSpeed(fanMax);
  delay(2000);
  setFanSpeed(fanMax / 5);
}


//is the door open?
int getDoorState(int doorsPin) { //not doorPin otherwise it gets replaced by 3 in the global defs
  return digitalRead(doorsPin);
}

void turnHeater(int state) {
  if (state) {
    Serial.println("HEATER TURNED ON=================================================================");
    digitalWrite(heaterRelayPin, HIGH);
    red = 1;
  } else {
    Serial.println("HEATER TURNED OFF****************************************************************");
    digitalWrite(heaterRelayPin, LOW);
  }
}


void setFanSpeed(int Speed) {
  if (Speed < fanMin) {
    Speed = fanMin;
  }
  if (Speed > fanMax) {
    Speed = fanMax;
  }
  pwm.setPWM(fanPin, 0, Speed);
}


void setup()
{

  lcd.begin(16, 2);              // start the library
  lcd.setCursor(0, 0);           //Set the cursor
  pinMode(UVrelayPin, OUTPUT);   //set the relay control Pin as output
  pinMode(airTempPin, INPUT);       //Temperature pin as input
  pinMode(doorPin, INPUT);       //pin to detect if door is closed
  pinMode(heaterRelayPin, OUTPUT);    //control heating unit
  turnUVlight(OFF);              //set the relay control pin high to keep the circuit open
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1600);
  fankickStart();         //Start fan at max speed to overcome static
}

void loop()
{
  glowLED();
  printCurrentTab();
  int lcd_key = read_LCD_buttons();
  //When a key is pressed, a different resistance is created so you have to read the analogPin in which the keys are connected to (A1)
  switch (lcd_key) {
    case btnNONE:
      {
        counter = 0;
        break;
      }
    case btnRIGHT:
      {
        rightRoutine();
        break;
      }
    case btnLEFT:
      {
        leftRoutine();
        break;
      }
    case btnUP:
      {
        upRoutine();
        break;
      }
    case btnDOWN:
      {
        downRoutine();
        break;
      }
    case btnSELECT:
      {
        startCuring();
        break;
      }
  }
  //if Select is taken, start the timer
  if (enterFlag) {
    while (hours > 0 || minutes > 0 || seconds > 0) {
      if (getDoorState(doorPin) || ((getTemp(heaterTempPin) - getTemp(airTempPin)) > HEATER_MAX_TEMP)) {
        errorRoutine();
      }
      turnUVlight(ON);  //Turn uv light relay ON
      monitorTemp();
      if (timer(millis_counter, millis_in_seconds)) {
        millis_counter = millis();
        seconds -= 1;
        if (seconds < 0) {
          minutes--;
          seconds = 59;
        }
        if (minutes < 0) {
          hours --;
          minutes = 59;
        }
      }
      glowLED();
      printCuring();
    }

    turnUVlight(OFF); //Turn uv light relay OFF
    printFinished();
    enterFlag = 0;
    turnHeater(OFF);
    setFanSpeed(fanMax);
    while (1) {
      if (getTemp(airTempPin) > warningTemp) {
        red = 1;
        setFanSpeed(fanMax);
      } else {
        red = 0;
        setFanSpeed(fanMax * 0.2);
      }
      glowLED();
    }
  }

}

////Read temperature on the thermistor control pin

//Function taken from manufacturer
float getTemp(int temperaturePin) {
  double RawADC = analogRead(temperaturePin);
  temperature = log(((10240000 / RawADC) - 10000));
  temperature = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temperature * temperature )) * temperature );
  temperature = temperature - 273.15;// Convert Kelvin to Celcius
  return temperature;
}

//timer function
bool timer(unsigned long timerVal, unsigned long timerDuration) {
  if (millis() - timerVal >= timerDuration)
    return true;
  else
    return false;
}

// return which button was pressed, we have two different lcd keypad shields, and each button returns different readings
int read_LCD_buttons()                   //Different buttons give different voltage level, and analogReading it will tell you which one it is
{
  int adc_key_in = analogRead(keyinPin);      // read the value from the sensor
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  // if (adc_key_in > 1000) return btnNONE;  // We make this the 1st option for speed reasons since it will be the most likely result
  // if (adc_key_in < 50)   return btnRIGHT;
  // if (adc_key_in < 250)  return btnUP;
  // if (adc_key_in < 450)  return btnDOWN;
  // if (adc_key_in < 650)  return btnLEFT;
  // if (adc_key_in < 850)  return btnSELECT;

  // For V1.0 comment the other threshold and use the one below:
  if (adc_key_in < 50)   return btnRIGHT;
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;

  return btnNONE;  // when no button is pressed return none
}

//Print the main menu on lcd
void printMainMenu() {
  lcd.setCursor(0, 0);
  //lcd.print("1234567890123456");
  lcd.print("  Select mode   ");
  lcd.setCursor(0, 1);
  if (manualFlag) {
    lcd.print(">");
    lcd.print("Manual");
    lcd.setCursor(9, 1);
    lcd.print(" Preset");
  } else {
    lcd.print(" Manual");
    lcd.setCursor(9, 1);
    lcd.print(">");
    lcd.print("Preset");
  }
}

//Print the preset mode tab
void printPreset() {
  lcd.setCursor(0, 0);
  //lcd.print("1234567890123456");
  lcd.print(" Type Color Time");
  lcd.setCursor(1, 1);
  lcd.print(resinArr[resinType]);
  if (resinType == 0) {
    if (resSelect >= ARRAY_SIZE(GPR)) {
      resSelect = 0;
    }
    lcd.setCursor(6, 1);
    lcd.print(GPR[resSelect]);
    lcd.setCursor(12, 1);
    lcd.print(GPR_time[resSelect]);
    lcd.print("M ");
  } else if (resinType == 1) {
    if (resSelect >= ARRAY_SIZE(FR)) {
      resSelect = 0;
    }
    lcd.setCursor(6, 1);
    lcd.print(FR[resSelect]);
    lcd.setCursor(12, 1);
    Serial.println(resSelect);

    lcd.print(FR_time[resSelect]);
    lcd.print("M ");
  }
  lcd.setCursor(0, 1);
  if (resinTypeFlag) {
    lcd.print(">");
  } else {
    lcd.print(" ");
  }
  lcd.setCursor(5, 1);
  if (resSelectFlag) {
    lcd.print(">");
  } else {
    lcd.print(" ");
  }
}

//print the manual mode tab
void printManual() {
  lcd.setCursor(0, 0);
  //lcd.print("1234567890123456");
  lcd.print("  Time    Temp");
  lcd.setCursor(1, 1);
  //print the time here
  lcd.print(hours);
  lcd.print("H");
  lcd.print(":");
  if (minutes < 10) {
    lcd.print("0");
  }
  lcd.print(minutes);
  lcd.print("M  ");
  lcd.setCursor(10, 1);
  lcd.print(curingTemp);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 1);
  if (timeFlag) {
    lcd.print(">");
    lcd.setCursor(9, 1);
    lcd.print(" ");
  } else {
    lcd.print(" ");
    lcd.setCursor(9, 1);
    lcd.print(">");
  }
}

//print the curing time and temperature
void printCuring() {
  lcd.setCursor(0, 0);
  lcd.print("Curing at ");
  lcd.print(curingTemp);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(hours);
  lcd.print(":");
  lcd.print(minutes);
  lcd.print(":");
  lcd.print(seconds);
  lcd.print("  ");
  lcd.setCursor(10, 1);
  temperature = getTemp(airTempPin);
  lcd.print(temperature);
  lcd.print((char)223);
  lcd.print("C");
}

//Updates hour and minutes and temperature field when preset mode is used
void updateCuringTime() {
  if (presetFlag) {
    if (resinType == 0) {
      hours = GPR_time[resSelect] / 60;
      minutes = GPR_time[resSelect] % 60;
      curingTemp = GPR_temp[resSelect];
    }
    else {
      hours = FR_time[resSelect] / 60;
      minutes = FR_time[resSelect] % 60;
      curingTemp = FR_temp[resSelect];
    }
  }
  hoursbu = hours;
  minutesbu = minutes;
}


//Print the current tab the user is in
void printCurrentTab() {
  if (presetFlag && !mainMenuFlag) {
    printPreset();
  } else if (manualFlag && !mainMenuFlag) {
    printManual();
  } else if (mainMenuFlag) {
    printMainMenu();
  }
}


//The following routine sets flags and indicators when user modifies the parameters through the keypad shield

void rightRoutine() {
  if (mainMenuFlag) {
    presetFlag = 1;
    manualFlag = 0;
  }
  if (presetFlag && !mainMenuFlag) {
    resSelectFlag = 1;
    resinTypeFlag = 0;
  }
  if (manualFlag && !mainMenuFlag) {
    tempFlag = 1;
    timeFlag = 0;
  }
}

void leftRoutine() {
  if (mainMenuFlag) {
    manualFlag = 1;
    presetFlag = 0;
  }
  if (presetFlag && !mainMenuFlag) {
    resinTypeFlag = 1;
    resSelectFlag = 0;
  }
  if (manualFlag && !mainMenuFlag) {
    timeFlag = 1;
    tempFlag = 0;
  }
}

void upRoutine() {
  if (presetFlag && !mainMenuFlag) {
    if (resinTypeFlag) {
      resinType = !resinType;
      resSelect = 0;
    }
    if (resSelectFlag) {
      resSelect += 1;
    }
    delay(200);
  }
  if (manualFlag && !mainMenuFlag) {
    if (timeFlag) {
      minutes += 5;
      if (counter > 3) {
        minutes += 5;
      }
      if (minutes >= 60) {
        minutes = 0;
        hours ++;
      }
      counter++;
    }
    if (tempFlag) {
      curingTemp += 5;
      if (curingTemp > maxTemp) {
        curingTemp = maxTemp;
      }
    }
    delay(200);
  }
}

void downRoutine() {
  if (presetFlag && !mainMenuFlag) {
    if (resinTypeFlag) {
      resinType = !resinType;
      resSelect = 0;
    }
    if (resSelectFlag) {
      resSelect -= 1;
      if (resSelect < 0) {
        if (resinType == 0) {
          resSelect = ARRAY_SIZE(GPR) - 1;
        } else {
          resSelect = ARRAY_SIZE(FR) - 1;
        }
      }
    }
    delay(200);
  }
  if (manualFlag && !mainMenuFlag) {
    if (timeFlag) {
      minutes -= 5;
      if (counter > 3) {
        minutes -= 5;
      }
      if (minutes <= 0) {
        if (hours != 0) {
          minutes = 55;
        } else {
          minutes = 0;
        }
        hours--;
        if (hours < 0) {
          hours = 0;
        }
      }
      counter++;
    }
    if (tempFlag) {
      curingTemp -= 5;
      if (curingTemp <= minTemp) {
        curingTemp = minTemp;
      }
    }
    delay(200);
  }
}

void printFinished() {
  finishCure = 1;
  lcd.setCursor(0, 0);
  lcd.print("FINISHED   hh:mm");
  lcd.setCursor(0, 1);
  lcd.print("Time Cured ");
  lcd.print(hoursbu);
  lcd.print(":");
  lcd.print(minutesbu);
  lcd.print("   ");
}

void startCuring() {
  lcd.clear();
  if (mainMenuFlag == 0) {
    enterFlag = 1;
    if (manualFlag) {
      if (!(hours > 0 || minutes > 0)) {
        enterFlag = 0;
      }
    }
  }
  if (manualFlag != 0 || presetFlag != 0) {
    mainMenuFlag = 0;
  }

  updateCuringTime();
  delay(200);
}

//Heater stays on for a max of 15 seconds

void monitorTemp() {
  temperature = getTemp(airTempPin);


  if (temperature > (curingTemp - 1)) {
    if (timer(heaterRelayTime, heaterRelayTimer)) {
      heaterRelayTime = millis();
      turnHeater(OFF);
      heaterOnFlag = 0;
      setFanSpeed(fanMax / 3);
      if (temperature > (curingTemp + 5)) {
        setFanSpeed(fanMax);
      }
      //Serial.println("Heater achieved target temp---------------------------------------------------------------------------------------");
    }
  } else if (temperature < (curingTemp - tempThreshold - 1)) {
    if (temperature > 30) {
      setFanSpeed(fanMax * 0.7);
    }
    if (temperature > 45) {
      setFanSpeed(fanMax / 2);
    }
    if (timer(heaterRelayTime, heaterRelayTimer) && heaterOnFlag == 0) {
      heaterRelayTime = millis();
      heaterOnTimeTimer = millis();
      turnHeater(ON);
      heaterOnFlag = 1;
    }
  }
  //Following if statement turns heater off after set time
  if (timer(heaterOnTimeTimer, millis_in_seconds * HEATER_ON_TIME) && heaterOnFlag) {
    //turnHeater(OFF);
  }
  if (timer(heaterOnTimeTimer, millis_in_seconds * HEATER_OFF_TIME) && heaterOnFlag) {
    heaterOnFlag = 0;
    //Serial.println("Heater timed out-----------------------------------------------------------------------------------------------------");
  }
}

void errorRoutine() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (getDoorState(doorPin)) {
    lcd.print("ERROR DOOR OPEN!");
  }
  if ((getTemp(heaterTempPin) - getTemp(airTempPin)) > HEATER_MAX_TEMP) {
    lcd.print("HEATER OVER TEMP");
  }
  lcd.setCursor(0, 1);
  lcd.print("Rem: ");
  lcd.print(hours);
  lcd.print(":");
  lcd.print(minutes);
  lcd.print(":");
  lcd.print(seconds);
  turnUVlight(OFF);
  turnHeater(OFF);
  brightnessVar = brightnessMax;
  glowLED();
  while (getDoorState(doorPin) || ((getTemp(heaterTempPin) - getTemp(airTempPin)) > HEATER_MAX_TEMP)) {
    delay(800);
  }
  delay(2000);
  turnUVlight(ON);
  lcd.clear();
  printCuring();
}

void turnUVlight(int state) {
  if (state == OFF) {
    Serial.println("UV LIGHT TURNED OFF>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    digitalWrite(UVrelayPin, HIGH);
  }
  if (state == ON) {
    Serial.println("UV LIGHT TURNED ON <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    digitalWrite(UVrelayPin, LOW);
  }
}



void glowLED(void) {
  Serial.print("Air temp reading:");
  Serial.println(getTemp(airTempPin));

  Serial.print("Heater temp reading: ");
  Serial.println(getTemp(heaterTempPin));

  // put your main code here, to run repeatedly:
  if (timer(lightTimer, brightnessTimer)) {
    lightTimer = millis();
    if (orientation) {
      brightnessVar += pwmRate;
    } else {
      brightnessVar -= pwmRate;
    }
    //4095 max
    if (brightnessVar > brightnessMax) {
      orientation = 0;
      brightnessVar = brightnessMax;
    } else if (brightnessVar < 0) {
      orientation = 1;
      brightnessVar = 0;
    }
    if (timer(redLightTimer, 2000)) {
      redLightTimer = millis();
      if (getTemp(airTempPin) > warningTemp) {
        red = 1;
      } else if ((getTemp(airTempPin <= warningTemp)) && (heaterOnFlag == 0)) {
        red = 0;
      }
    }
    if (enterFlag) {
      yellow = 1;
    } else {
      yellow = 0;
    }
    green = !yellow;
    if (red) {
      pwm.setPWM(rPin, 0, brightnessVar);
    } else {
      pwm.setPWM(rPin, 0, 0);
    }
    if (green) {
      pwm.setPWM(gPin, 0, brightnessVar);
    } else {
      pwm.setPWM(gPin, 0, 0);
    }
    if (yellow) {
      pwm.setPWM(yPin, 0, brightnessVar);
    } else {
      pwm.setPWM(yPin, 0, 0);
    }
  }
}


