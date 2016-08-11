/*
   Systems Clock V1
   Author: Brad Kubiak
   Revision Date: 9/16/17
   Modified: Richard Li
*/

#include <genieArduino.h>
#include <SoftwareSerial.h>
#include <Stepper2.h>
#include <Servo.h>
#include <Wire.h>
#include "RTClib.h"

#define FEEDBACK_NULL 99

//Pins are to be tied to logic high to allow current to flow
//  to stepper motor

//Servo constants
#define SERVO_PIN 22            //Control Signal for servo motor
#define SERVO_TIME_DELAY 2000   //delay for changing servo direction
#define SERVO_MIN_ANGLE 100     //Minimum angle for servo
#define SERVO_MAX_ANGLE 140     //Maximum angle for servo
#define SERVO_DEFAULT_ANGLE 107 //Normal position 107 becuase angle jitters at 110
#define STATIONARY 0            //The values define the state
#define FORWARD 1               //  of the servo motor and the
#define BACKWARD 2              //  direction of motor rotation
#define DEBOUNCE_DELAY 250      //Time to debounce display touch screen


// Stepper Constants - Motor Speed Ranges
#define MOTOR_SPEED 180         //180 RPM Motor Speed normal speed
#define UPPER 200 //Servo speed max
#define LOWER 120 //Servo speed min

//Feedback Constants
#define FEEDBACK_TIME_DELAY 1000
#define HOURS 0
#define MINUTES 1

//Diplay Constants
#define RESETLINE 4
#define SCREEN_TIME_DELAY 1000     //500 was too short of a time

//Feedback variables
int minutePins[] = {30, 32, 34, 36};
int hourPins[] = {38, 40, 42, 44};
unsigned long feedbackTimer = 0;  //Timer for whole loop
int minuteFeedback = 0;           //Minutes feedBack from mech
int hourFeedback = 0;             //Hours   feedback  from mech
int minutePrevious = 0;           //Previous non null minute
int hourState;                    //Run servo flag


//Motor objects
Servo hourServo;
//Stepper clockStepper(STEP_RESOLUTION, DIR_A, DIR_B);

//Servo Values
unsigned long servoTimer = 0;             //The timer value for the servo timer
int servoState = 0;                       //The state of the servo: STATIONARY, FORWARD, BACKWARD
int rpm = MOTOR_SPEED;                    //Stepper motor base speed 180
int servo_running_flag = 0;               //Is servo running? 1 yes 0 no

//Display variables
Genie genie;                              //Display
unsigned long screenTimer = 0;            //Screen timer for display

//Time Variables
int current_hour = 12;                    //Hour now
int current_minute = 0;                   //Minute now
int current_second = 0;                   //Seconds now
int minute_last_valid = 0;                //Last valid minute reading
int hour_last_valid = 0;                  //Last valid hour reading

//PI Variables
float sum_of_errors =  0;           //Integral term (proportional declared above)
float curr_error    =  0;           //proportional term

#define prop_K 0.4               //PI constants
#define int_K  0.2

//RTC variables
RTC_DS1307 rtc;
DateTime now;

int brightness_flag = 0;          //If slider changes, this will be asserted
int brightness = 10;              //Brightness level of the clock
//Time Comparison Variables
int timeDiffInMins = 0;   //Diff in Mins
int prevTemp;             //prev valid reading
int setFlag = 0;          //Flag that triggers when SET is pressed, rushes hour hand to correct hour 
                          //This is used when the hour has been changed (ie daylight savinds)

//Debouncing timers
unsigned long hourDebounce = 0;     //Debounce timer for hour button on display
unsigned long minuteDebounce = 0;   //Debounce timer for minute button ' '

void setup() {
  // put your setup code here, to run once:
  hourServo.attach(SERVO_PIN);    //attach servo to pin
  initializeMHPins();             //Initialize the minute and hour pins
  Serial.begin(115200);           //VisiGenie Dispplay
  Serial1.begin(115200);          //Arduino Comm
  if (! rtc.begin()) {            //Until RTC begins don't run, this will not run without a working RTC chip
    while (1);
  }
  if (! rtc.isrunning()) {
    // following line sets the RTC to the date & time this sketch was compiled, make sure to remove the battery from the RTC to run this line
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  genie.Begin(Serial);                    //Initialize VisiGenie
  genie.AttachEventHandler(eventHandler); //Display eventhandler(what does the display do)
  resetDisplay();                                  //Resets the display
  genie.WriteObject(GENIE_OBJ_SLIDER, 0x00,75 );   //Write the current brightness level on the slider, inputs from 0- 100
  genie.WriteContrast(brightness);                 //Set screen contrast (5 fights with servo)
}

void loop() {
  // put your main code here, to run repeatedly:
  now = rtc.now();                  //Now is just any variable a DateTime variable
  genie.DoEvents();                 //Do stuff when display gets data
  getCurrentTime();                 //Updates current_time hour minute & seconds variables
  
  if (timer(feedbackTimer, FEEDBACK_TIME_DELAY)) {  //Ensure enough time has passed since last run
    feedbackTimer = millis();                       //Update timer
    //lol
    int temp_m = checkFeedback(MINUTES);            //Check the feedback from the 4 pins
    int temp_h = checkFeedback(HOURS);
    if (prevTemp == temp_m){
      temp_m == FEEDBACK_NULL;
    }
    if (temp_m != FEEDBACK_NULL){
        minutePrevious = minuteFeedback;
        minuteFeedback = temp_m * 5;      
        minute_last_valid = minuteFeedback;
    }
    if (temp_h != FEEDBACK_NULL){
        hourFeedback = temp_h;              //Set hour (from duino)
        if (hourFeedback == 0) {
          hourFeedback = 12;                 //Set to 12 if it's 0
          hour_last_valid = hourFeedback;
      }
    }
    /*
    int temp_m = checkFeedback(MINUTES);
    int temp_h = checkFeedback(HOURS);
    int temp_m2 = FEEDBACK_NULL;
    int temp_h2 = FEEDBACK_NULL;
    if (timer(dcheckTimer, FEEDBACK_TIME_DELAY)) {  //Routine to check the pins once again to avoid
      dcheckTimer = millis();                     //incorrect reading of pins
      temp_m2 = checkFeedback(MINUTES);           //Second reading of pins
      temp_h2 = checkFeedback(HOURS);             //Check feedback returns 0-12
      if (temp_h != temp_h2) {                    //Check if the reading before and now matches to reduce noise
        temp_h = FEEDBACK_NULL;
      }
      if (temp_m != FEEDBACK_NULL) {  //If minute is valid, update previous
        minutePrevious = minuteFeedback;
        minuteFeedback = temp_m * 5;      
        minute_last_valid = minuteFeedback;
      }
      if (temp_h != FEEDBACK_NULL) {
        hourFeedback = temp_h;              //Set hour (from duino)
      }
      if (hourFeedback == 0) {
        hourFeedback = 12;                 //Set to 12 if it's 0
        hour_last_valid = hourFeedback;
      }
    }*/
  }   //Double check routine ends here
  
  if (minuteFeedback != FEEDBACK_NULL && prevTemp != minuteFeedback) { //If readings were valid run this code once
    curr_error = compareTime();                //Function that returns difference in seconds
    if (abs(curr_error) > 30){                          //If the difference is more than 30 seconds, we will linearly catch up to the real time by slowing or speeding up
      rpm = (curr_error) / 60 * 180 / 5 + 180;          //Proportional method      
    }else{                                              //If the difference is less than 30, we will do finer adjustments with PID control
      sum_of_errors += curr_error;
      rpm = int_K * sum_of_errors + prop_K * curr_error + MOTOR_SPEED;  //PID
    } 
    //Ensure rpm is within safe lower and upper bounds
    rpm = rpm_limit(LOWER, UPPER);
    Serial1.write(rpm);   //Sent speed to arduino UNO
  }
  
  if ((hourFeedback != current_hour) && (servo_running_flag == 0) && (setFlag == 1))  { //If servo is not running and hours are
    hourState = 1;                                                                      //not matching, and the user pressed set
    servo_running_flag = 1;                                                             //run the servo to the right time
  } else if (hourFeedback == current_hour && setFlag == 1) {
    hourState = 0;
    servo_running_flag = 0;
    setFlag = 0;
  }
  
  if (minuteFeedback == 0 && minutePrevious == 55 && setFlag == 0)   //Setting the hourState to 1 will run the servo, and if we went from 55 mins to 0,increase hour
    hourState = 1;
    
  if (hourState == 1 || hourState == 3) {
    servoState = BACKWARD;
  }
  if (timer(screenTimer, SCREEN_TIME_DELAY)) {      //Write to display if screenTimer reaches screen_time_delay
    screenTimer = millis();
    writeDisplay();
  }
  if (minuteFeedback != FEEDBACK_NULL){
    prevTemp = minuteFeedback;                      //Update the temporary holder variable for minutes, (we used temporary to know if we read the next reading minute or not)
  }
  runServo();                                      
}


void initializeMHPins(void) {
  for (int i = 0; i < 4; i++) {       //initialize INPUT pins to read hour and minutes from gear
    pinMode(minutePins[i], INPUT);
    pinMode(hourPins[i], INPUT);
  }
  return;
}

void resetDisplay(void) {
  pinMode(RESETLINE, OUTPUT);
  digitalWrite(RESETLINE, 1);
  delay(100);
  digitalWrite(RESETLINE, 0);
  delay(5000);
}


int rpm_limit(int lower, int upper) {
  if  (rpm < lower) {
    rpm = lower;
  }
  if (rpm > upper) {
    rpm = upper;
  }
  return rpm;
}
/*
   Function: Timer
   Purpose: To set timer flags
   Parameters:
   Returns: True if timer has expired
*/
bool timer(unsigned long timerVal, unsigned long timerDuration) {
  if (millis() - timerVal >= timerDuration)
    return true;
  else
    return false;
}
/*
   Function: runServo
   Purpose: Advances the clock by one hour
   Parameters: None
   Returns: None
*/
void runServo() {
  if (servoState == FORWARD) {
    hourServo.write(SERVO_MIN_ANGLE);
    if (timer(servoTimer, SERVO_TIME_DELAY)) {
      servoTimer = millis();
      servoState = STATIONARY;
      if (hourState == 2)
        hourState++;
      else if (hourState == 4)
        hourState = 0;
        servo_running_flag = 0;
    }
  }
  else if (servoState == BACKWARD) {
    if (hourState == 1 || hourState == 3)
      hourState++;
    hourServo.write(SERVO_MAX_ANGLE);
    if (timer(servoTimer, SERVO_TIME_DELAY)) {
      servoTimer = millis();
      servoState = FORWARD;
    }
  }
  else if (servoState == STATIONARY) {
    hourServo.write(SERVO_DEFAULT_ANGLE);
  }
}
/*
   Function: checkFeedback
   Purpose: Checks the feedback pins for time
   Parameters: the pins to be checked
   Returns: None
*/
int checkFeedback (int hand) {
  int *pins;

  if (hand == HOURS) {
    pins = hourPins;
  }
  else {
    pins = minutePins;
  }
  char pinValue = 0;  //char has 1 byte (8 bits)
  //int minutePins[] = {30, 32, 34, 36};
  //int hourPins[] = {38, 40, 42, 44};
  if ((digitalRead(pins[0]) && hand == HOURS) || (digitalRead(pins[0]) == 0 && hand == MINUTES))  //Read mechanical clock feedback
    pinValue |= B00001000;
  if (digitalRead(pins[1]))
    pinValue |= B00000100;
  if (digitalRead(pins[2]))
    pinValue |= B00000010;
  if (digitalRead(pins[3]))
    pinValue |= B00000001;
  //See "binary" hour values
  if (pinValue == B00001101)  //This is actually 12
    return 0;
  if (pinValue == B00001110)
    return 1;
  if (pinValue == B00001111)
    return 2;
  if (pinValue == B00000010)
    return 3;
  if (pinValue == B00000011)
    return 4;
  if (pinValue == B00000100)
    return 5;
  if (pinValue == B00000101)
    return 6;
  if (pinValue == B00000110)
    return 7;
  if (pinValue == B00000111)
    return 8;
  if (pinValue == B00001010)
    return 9;
  if (pinValue == B00001011)
    return 10;
  if (pinValue == B00001100)
    return 11;

  return FEEDBACK_NULL;
}

/*
   Function: eventHandler
   Purpose: Deals with events from the digital display (i.e. button presses)
   Parameters: None
   Returns: None
*/
void eventHandler(void) {
  genieFrame Event;
  genie.DequeueEvent(&Event);
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT) {
    // If the Reported Message was from a Win Button
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON) {
      if (Event.reportObject.index == 0) {           // If WinButton0 - hour button
        if (timer(hourDebounce, DEBOUNCE_DELAY)) {   //Increase hours
          hourDebounce = millis();
          increaseHour();
        }
      }
      if (Event.reportObject.index == 1) {           // If WinButton1 - minute button
        if (timer(minuteDebounce, DEBOUNCE_DELAY)) { //Increase minutes
          minuteDebounce = millis();
          increaseMinute();
        }
      }
      if (Event.reportObject.index == 2) {           //If set was pressed----------------------Nothing here yet
        setFlag = 1;
      }  
    }
    if (Event.reportObject.object == GENIE_OBJ_SLIDER)                // If the Reported Message was from a Slider
    {
      if (Event.reportObject.index == 0)                              // If Slider0
      {
        brightness = genie.GetEventData(&Event);                      // Receive the event data from the Slider00
        brightness = brightness * 15 / 100;
        brightness_flag = 1;
      }
    }
  }
}

/*
   Function: increaseMinute
   Purpose: Used to increase the minute on the digital clock when the minute button is pressed on the clock
   Parameters: None
   Returns: None
*/
void increaseMinute() {
  current_minute = now.minute();
  current_minute += 1;
  if (current_minute >= 60) {
    current_minute -= 60;
  }
  rtc.adjust(DateTime(1924,11,14,now.hour(),current_minute,now.second()));
}

/*
   Function: increaseHour
   Purpose: Used to increase the minute on the digital clock when the minute button is pressed on the clock
   Parameters: None
   Returns: None
*/
void increaseHour() {
  current_hour = now.hour();
  current_hour += 1;
  if (current_hour > 12) {
    current_hour -= 12;
  }
  rtc.adjust(DateTime(1924,11,14,current_hour,now.minute(),now.second()));
}

/*
   Function: getCurrentTime()
   Purpose: used to update the digital time parameters with the actual time
   Parameters: None
   Returns: None
*/
void getCurrentTime() {

  // Getting the current time
  current_hour = now.hour();
  if (current_hour > 12) {
    current_hour -= 12;
  }
  current_minute = now.minute();
  current_second = now.second();
}

/*
   Function: writeDisplay
   Purpose: Used to output the current time to the digital display
   Parameters: None
   Returns: None
*/
void writeDisplay() {
  getCurrentTime();
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x00, current_hour);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x03, current_minute);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x04, current_second);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x05, rpm);
  genie.WriteObject(GENIE_OBJ_ANGULAR_METER, 0x00, rpm-120); //genie takes input from 0 - 80, and RPM ranges from 120 - 200

  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x01, hour_last_valid);
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0x02, minute_last_valid);

  if (brightness_flag){
    if (brightness == 0){
      brightness = 1;
    }
    genie.WriteContrast(brightness);
    brightness_flag = 0;
  }
  
}


//Time Diff is more like in seconds now
int compareTime() {
  int timeDiffInMins = current_minute * 60 + current_second - minuteFeedback * 60;  //Find the time difference in seconds, real time - feedback time
  //Find whether it's easier to get back or speed up
  if (timeDiffInMins > 1800){
    timeDiffInMins -= 3600;
  }else if (timeDiffInMins <= -1800){
    timeDiffInMins += 3600;
  }
  return timeDiffInMins;
}
