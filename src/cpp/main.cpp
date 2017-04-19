draft_main.cpp
DETAILS
ACTIVITY
Long ago

You uploaded an item
Jun 22, 2015
C++
draft_main.cpp
No recorded activity before June 22, 2015
All selections cleared All selections cleared All selections cleared All selections cleared All selections cleared All selections cleared

/*

latest update:
6/22/15 00:41
by abmo


 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Time.h>
#include <TimeAlarms.h>
#include <StopWatch.h>
#include <LiquidCrystal.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select Port M1 for motor
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12,11,5,4,3,2);

// I/O PIN VARIABLES
#define inTempSensorPin A1
#define outTempSensorPin A2
#define lightPin A0

// OTHER VARIABLES
//StopWatch sw_millis;    // MILLIS (default)
//StopWatch sw_micros(StopWatch::MICROS);
StopWatch sw_secs(StopWatch::SECONDS);

int posCurtain;      
int nextPosCurtain;  
int commandCurtain;
int mode;
char inKey;
float inTemp, outTemp;


void setup()
{
  // Start serial connection
  Serial.begin(9600);
  
  lcd.clear();
  lcd.print("WELCOME\nECO-CURTAIN BY TEAM CHOPSTICKS");
  
  // Set program into its first setup
  // Boot motor, sensors and LCD into their respective initial conditions
  bootEverything();

}

void  loop() {  
  lcd.print("SETTINGS\nPress A to change time and date\nPress B to choose curtain mode");
  // Start SW
  sw_secs.start();
  while (sw_secs.elapsed<15) {
    // inKey
    if (inKey=='A' || inKey=='B') {
      sw_secs.stop();
      lcd.clear();
      if (1) inputTime(); // if inKey=='A', input time
      else if (0) ModeSelection(); // if inKey=='B', input mode
      break;
    }
  }

  delay(1000);
  // Reset SW
  sw_secs.reset();
  inKey=-99;
  while (1) {
    Idle();
    // Wait for inKey
    if (inKey!=-99) break;
  }  
  
}


void Idle() {
    // Display temperature (outside and inside)
    float outsideT = getTempReadingF(outTempSensorPin);
    float insideT = getTempReadingF(inTempSensorPin);
    lcd.print("Temperature: Out: ");
    lcd.print(outsideT);
    lcd.print(" In: ");
    lcd.print(insideT);

    // Display date and time
    digitalClockDisplay();
    Alarm.delay(1000); // wait one second between clock display
}


// ================================================
//          LCD WELCOME AND SETTINGS
// ================================================ 

void bootEverything() {
  delay(1000);
  lcd.clear();
  do {
    lcd.print("Please open your curtain fully and place the string to the machine\n");
    lcd.print("Press '#' when you're ready to set up your ECO Curtain");
    //inKey
  } while(inKey!='#')
  lcd.print("Thank you! Kindly wait while we set up your curtain");
  mode=-1; posCurtain=0; nextPosCurtain=-1; repeatornah=-1
  bootMotor();
  bootTmpSensors();
  delay(1000);
  lcd.clear();
  inputTime();
  ModeSelection();
}


// DATE AND TIME SETTINGS
void inputTime() {
  int d,m,y,h,min;
  bool m_fl,d_fl,y_fl,h_fl,min_fl;
  m_fl=d_fl=y_fl=h_fl=min_fl=0;
  
  lcd.print("DATE AND TIME SETTINGS");
  lcd.print("Please enter today's date (MM DD YYYY)");
  lcd.print("M:1-12, D:1-31, Y:1977-2015");
  // inKey m, d, y
  if (m>=1 && m<=12) m_fl=1;
  if (d>=1 && d<=31) d_fl=1;
  if (y>=1977 && y<=2015) y_fl=1;
  while ( !(m_fl*d_fl*y_fl) ) {
    lcd.println("Error! Please enter a valid date (M:1-12, D:1-31, Y:1977-2015)");
    // inKey m, d, y
  }
  
  lcd.clear();
  lcd.print("Thank you");
  delay(1000);
  
  lcd.clear();
    lcd.print("Please enter current military time (hh mm)");
  // inKey h, m
  if (h>=0 && h<=12) h_fl=1;
  if (min>=0 && min<=59) min_fl=1;
  while( !(h_fl*min_fl) ) {
    lcd.println("Error! Please enter a valid time (h:0-23, m:0-59)");
    // inKey h, min
  }
  // setTime(hour,min,secs=0,month,day,year)
  setTime(h,min,0,m,d,y);
  lcd.clear();
  lcd.print("Thank you! The date and time set is ");
  digitalClockDisplay();
  delay(2000);
}

// MODE SELECTION
void ModeSelection() {
  lcd.print("MODE SELECTION");
  lcd.print("Select A for Auto Mode, B for Alarm Mode, C for Manual Override");
  // mode = inKey
  while(mode!=1 || mode!=2 || mode!=3) {
    lcd.clear();
    lcd.print("Error! Please enter a valid input (A, B, C)");
    // mode = inKey
  }
  if (mode==1) { AutoMode(); Alarm.timerRepeat(30, Repeats); }          // Auto Mode selected
  else if (mode==2) AlarmMode();                                    // Alarm Mode selected
  else if (mode==3) ManualMode();                                       // Manual Mode selected
  lcd.clear();
}


// ================================================
//             MODE 1 - AUTO MODE
// ================================================

void AutoMode() {
  lcd.println("Auto Mode selected!");
  lcd.println("This mode will automatically adjust your curtain based on outside and inside temperature");
  
  inTemp = getTempReading(inTempSensorPin);
  outTemp = getTempReading(outTempSensorPin);
  int light = getLightReading();
  float tmpDiff = outTemp - inTemp;
  
  // CASE 1
  if (abs(tmpDiff)<5) {
    if (inTemp<10) {
      // Curtain will be fully opened
      adjustCurtain(posCurtain,0x00);
    } else if (inTemp>10 && inTemp<27) {
        if (light<500) {
          // Curtain will be fully opened
          adjustCurtain(posCurtain,0x00);
        } else
          // Curtain will be half opened
          adjustCurtain(posCurtain,0x01);
    } else if (inTemp>27) {
        if (light<500) {
          // Curtain will be half opened
          adjustCurtain(posCurtain,0x01);
        } else
            // Curtain will be fully closed
            adjustCurtain(posCurtain,0x02);
    }
  }

  // CASE 2
  else if (tmpDiff<-5) {
    if (inTemp<10) {
      // Curtain will be fully closed
      adjustCurtain(posCurtain,0x02);
    } else if (inTemp>10 && inTemp<27) {
        // Curtain will be half opened
        adjustCurtain(posCurtain,0x01);
    } else if (inTemp>27) {
        // Curtain will be fully opened
        adjustCurtain(posCurtain,0x00);
    }
  }

  // CASE 3
  else if (tmpDiff>5) {
    if (inTemp<10) {
      // Curtain will fully opened
      adjustCurtain(posCurtain,0x00);
    } else if (inTemp>10 && inTemp<27) {
        // Curtain will be half openedopen half
        adjustCurtain(posCurtain,0x01);
    } else if (inTemp>27) {
        // Curtain will be fully closed
        adjustCurtain(posCurtain,0x02);
    }
  }
  delay(5000);
  lcd.clear();
}


void Repeats() {
  if (mode==1) AutoMode();
}






// ================================================
//             MODE 2 - ALARM MODE
// ================================================ 
// *set fix alarm to close the window in the night?

void AlarmMode() {
  lcd.println("Alarm Mode selected!");
  lcd.println("This mode will automatically open your curtain at a time you selected");
  delay(2000);
  lcd.clear();

  lcd.print("Please set time (military format) for the curtain to open");
  int hours,mins,repeatornah;
  // inKey hours, mins
  lcd.clear();
  lcd.print("Do you want the alarm to repeat everyday or only once? (1-repeat, 0-only once)");
  // inKey repeatornah
  while (repeatornah!=1 || repeatornah!=0) {
    lcd.clear();
    lcd.print("Error! Please enter a valid input (1-repeat, 0-only once)");
    // inKey repeatornah
  }

  lcd.clear();
  setAlarm(hours,mins,repeatornah);
  lcd.print("Thank you! The alarm is set at ");
  lcd.print(hours);
  lcd.print(": ");
  lcd.print(mins);
  if (repeatornah) lcd.print(" everyday");
  delay(2000);
}

void setAlarm(int h,int m,int repeat) {
  if (repeat) {
    Alarm.alarmRepeat(h,m,0,UserAlarm);
  }
  else 
    Alarm.alarmOnce(h,m,0,UserAlarm);

}

void UserAlarm() {
  if (mode==2) adjustCurtain(posCurtain,0x00); 
}




// ================================================
//            MODE 3 - MANUAL OVERRIDE
// ================================================ 

void ManualMode() {
  lcd.println("Manual override");
  lcd.println("Press A to open the curtain completely, B to close the curtain completely or C to open the curtain halfway");
  // inKey
  while (inKey!=1 || inKey!=2 || inKey!=3) {
    lcd.print("Error! Please enter a valid input (A, B, C)");
    // inKey
  }
  lcd.clear();
  lcd.print("Curtain position confirmed. Automatic settings and existing alarm will be disabled");
  if (inKey==1) adjustCurtain(posCurtain,0x00);
  else if (inKey==2) adjustCurtain(posCurtain,0x02);
  else if (inKey==3) adjustCurtain(posCurtain,0x01);
  delay(4000);
}





// ================================================
//                MOTOR FUNCTIONS
// ================================================

void bootMotor() {
  AFMS.begin();             // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
}






// ================================================
//             TMP SENSOR FUNCTIONS
// ================================================

void bootTmpSensors() {
  return;
}

int getTempReadingF(int pin) {
  return getTempReading(pin)*1.8 + 32;
}
       
float getTempReading(int pin) {
  float temperature = getVoltage(pin);       // getting the voltage reading from the temperature sensor
  return (temperature - .5) * 100;           // converting from 10 mv per degree wit 500 mV offset
                                             // to degrees ((voltage - 500mV) times 100)
}


// ================================================
//            LIGHT SENSOR FUNCTIONS
// ================================================

int getLightReading() {
  int lightLevel = analogRead(lightPin); //Read the
                                        // lightlevel
  return 1023-lightLevel;
  
  //lightLevel = map(lightLevel, 0, 900, 0, 255); 
          //adjust the value 0 to 900 to
          //span 0 to 255
}




// ================================================
//             HELPER FUNCTIONS
// ================================================

void adjustCurtain(int pos_now,int pos_next) {
  int z = (pos_now<<2) + pos_next;
  int x2,x1,x0;
  getCommandCurtain(z,x2,x1,x0);

  if (x2) {
    // Motor does nothing
    return;
  } else if (!x1) {
      if (!x0) {
        // Motor opens curtain full
        myMotor->run(FORWARD);
        delay(4000);
        myMotor->run(RELEASE);
        return;
      }
      else {
        // Motor opens curtain half
        myMotor->run(FORWARD);
        delay(2000);
        myMotor->run(RELEASE);
      }
  } else if (x1) {
      if (!x0) {
        // Motor closes curtain full
        myMotor->run(BACKWARD);
        delay(4000);
        myMotor->run(RELEASE);
      }
      else {
        // Motor closes curtain half
        myMotor->run(BACKWARD);
        delay(2000);
        myMotor->run(RELEASE);
      }
  }
  posCurtain = pos_next;
  nextPosCurtain = -1;
}

void getCommandCurtain(int &z,int &x2,int &x1,int &x0) {
  int b3,b2,b1,b0;
  bitValue(z,b3,b2,b1,b0);
  x2 = !b3*!b2*!b1*!b0 + b3*!b2*b1*!b0 
          + b2*!b0;
  x1 = !b3*b1*!b0 + b2*b1*!b0 + !b3*!b2*b0;
  x0 = b2*!b0 + !b2*b0;
}


void digitalClockDisplay() {
  // digital clock display of the time
  lcd.print(hour());
  printDigits(minute());
  printDigits(second());
  lcd.print(" ");
  lcd.print(dayStr(weekday()));
  lcd.print(" ");
  lcd.print(day());
  lcd.print(" ");
  lcd.print(monthStr(month()));
  lcd.print(monthShortStr(month()));
  lcd.print(" ");
  lcd.print(year()); 
  lcd.println(); 
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  lcd.print(":");
  if(digits < 10)
    lcd.print('0');
  lcd.print(digits);
}

/*
 * getVoltage() - returns the voltage on the analog input defined by
 * pin
 */
float getVoltage(int pin){
 return (analogRead(pin) * .004882814); //converting from a 0 to 1023 digital range
                                        // to 0 to 5 volts (each 1 reading equals ~ 5 millivolts
}

// b3 is MSB, b0 is LSB
void bitValue(int &z,int &b3,int &b2,int &b1,int &b0) {
  b3 = (z>>3) & 1;
  b2 = (z>>2) & 1;
  b1 = (z>>1) & 1;
  b0 = z & 1;
}