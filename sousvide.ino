//========================================================================
//PID Libs
//========================================================================
#include <PID_v1.h> //PID controller to control water temperature
#include <PID_AutoTune_v0.h>
/*
 * PID autotuner library. Handy for getting a good ballpark of Kp Ki and Kd constants
 * The autotuner works when the temperature is at he setpoint and at steady state already,
 * so it is easily included in the working program, and not left for its own seperate program.
 * another benefit of this is that the same controller program can be used in multiple
 * sous vide machines, or the same controller can use different equiptment/enclosures.
 * on the downside, it is a hog of dynamic memory, using almost 30% of all available SRAM.
 * Remeber to be mindful of the SRAM usage.
 */


//========================================================================
//LCD libs
//========================================================================
#include <Wire.h> //I2C -  for LCD Display
#include <Adafruit_RGBLCDShield.h> //For specific adafruit display with buttons
#include <utility/Adafruit_MCP23017.h> //I2C chip for adafruit display

//========================================================================
//Dallas Temp Libs
//=========================================================================
#include <DallasTemperature.h> //0.5c accuracy or about a degree
#include <OneWire.h>

//========================================================================
//Misc Libs
//========================================================================
#include <EEPROM.h> //storing PID constants between 
#include <avr/pgmspace.h> //F macro - save on RAM by storing strings in flash mem
#include "pitches.h" //for later timer done use - probably cant do this becuase 2 interrupts. find another way?
//suggestions - backlight on off every second until ackgnowledge?
//text with esp? 


//=========================================================================
//#define section
//=========================================================================
#define steakMedRare 131.5 //degrees F. defualt start temp
#define OFF 0x0 //backlight on 
#define ON 0x1 //backlight off
#define ONE_WIRE_BUS 2
#define RelayPin 3
#define MotorPin 4
#define SpeakerPin 5


//==
//NOTE: single operator is bitwise, double is logical. ex: & vs &&. vitl in the case of the LCD buttons.
//==


//============================================================================
//Function prototypes (Danielson Tribute)
//===========================================================================

void startSousvide(void);
void stopSousvide(void);
//double readTmp36(void); DEPRECATED - USING DALLAS
void setParamScreen(void);

void idleScreen(void);
void EEPROM_writeDouble(int adress, double val); //done
double EEPROM_readDouble(int adress); //done

/*
 * TODO
 * 1. comment all code
 * 2. clean up all code - delete unused, restructure, add prototypes, move functions around
 * and ut them in a better order (same as prot0types) - maybe use visual studio for this.
 */


//===========================================================
// Defining five bytes to be used by the createChar functions
// in the setup function. Each is a symbol that has some value,
// either to save characters and look good or to define an
// otherwise unusable symbol. The B is for Binary values.
//=========================================================== 

byte degree[8] = 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
};

byte arrowleft[8] = 
{
  0b00000,
  0b00000,
  0b00100,
  0b01000,
  0b10111,
  0b01000,
  0b00100,
  0b00000
};

byte arrowright[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b00010,
  0b11101,
  0b00010,
  0b00100,
  0b00000
};

byte arrowup[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b01010,
  0b10001,
  0b00100,
  0b00100,
  0b00100
};

byte arrowdown[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b00100,
  0b00100,
  0b10101,
  0b01010,
  0b00100
};
//==================================================

int timerSong[] = { 
  NOTE_F3, NOTE_A3, NOTE_C4, NOTE_E4, NOTE_F4
};
int durations[] = {
  2, 2, 2, 2, 0.85
};
unsigned long windowStart;
int windowsize = 10000; //10000ms = 10sec
int count = 0; //counter used in ISR to determine when a second passed
bool tempUnset = true;
bool timerSet = false;
bool timerDone = false;
bool hourssingle = true;
bool minutesssingle = true;
bool secondssingle = true;
bool safemode = false;
bool songPlayed = false;
int hours = 0;
int minutes = 0; 
int seconds = 0;
int SafteySeconds = 0;
double mostRecentTemp = 0.0;
double setpoint;
double input;
double output;
double Kp = 100; //good starting values b4 autotune
double Ki = 3.0;
double Kd = 0.2;

//autotune shit
byte autoTunemoderemember = 2;
double autoTunestep = 500;
double autoTunenoise = 1;
unsigned int autoTunelookback = 20;
bool tuning = false;
PID_ATune autoTuner(&input, &output);

//EEPROM vars. adress incremented by 8 becuase vaules stored are
//doubles - aka 8 bytes
const int setpointAdr = 0; //first EEPROM adress
const int KpAdr = 8;
const int KdAdr = 16;
const int KiAdr = 24;

//=================================================================
// Defining enums for heirarchical ordering of menus to be used
// as "state machines" via sitch statements. left and right chooses
// from main menus from "Menus" varibale, and up and down chooses
// from available submenus if not in idle menu.
//================================================================

enum Menus {Idle, Run, SetParams};
Menus menu = Idle;

enum RunSubMenus {SetTemp, SetTimer, RunScreen};
RunSubMenus runmenu = SetTemp;

enum SetParamSubMenus {SetKp, SetKi, SetKd};
SetParamSubMenus parammenu = SetKp;

//==================================================================
//Constructors
//==================================================================

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
PID tempController(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT, P_ON_M);
//P_ON_M - better for no overshoot;
//DIRECT - father away from setpoint means on more.
//(REVERSE would have it be on less)
//defaults to off when constructor is run
OneWire oneWire(ONE_WIRE_BUS); //setup one wire communication
DallasTemperature sensors(&oneWire); //passing onewire to dallas
DeviceAddress tempSensor; //used to store the address once weve got it


void setup() 
{
  //Serial.begin(115200); //for debug only, uncomment if not in use
  
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW); //heater off initially.

  //pinMode(SpeakerPin, OUTPUT);
  //little on tune here!

  pinMode(MotorPin, OUTPUT);
  digitalWrite(MotorPin, LOW); //motor off until heater on
  
  lcd.begin(16, 2); //set lcd columns and rows.
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.setBacklight(ON);
  lcd.createChar(1, degree); //creating degree symbol for lcd
  lcd.createChar(2, arrowleft);
  lcd.createChar(3, arrowright);

  /*
  unused
  lcd.createChar(4, arrowup);
  lcd.createChar(5, arrowdown);
  */

  //dallas sensor setup
  sensors.begin();
  if(!sensors.getAddress(tempSensor, 0))
  {
    lcd.setCursor(0,0);
    lcd.print("Sensor is");
    lcd.setCursor(0,1);
    lcd.print("fucked up.");
    delay(10000);
    lcd.clear();
  }
  sensors.setResolution(tempSensor, 12); //12 bit, max res
  sensors.setWaitForConversion(false); //async readings

  //load params from eeprom
  tempController.SetTunings(Kp, Ki, Kd);
  tempController.SetSampleTime(1000); //compute evals every 1s (1000ms)
  //not done using interupts besides millis, so good for use
  tempController.SetOutputLimits(0, windowsize);

  if (sensors.isConversionAvailable(0))
  {
      double tempC = sensors.getTempC(tempSensor);
      double tempF = DallasTemperature::toFahrenheit(tempC);
      sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }

  LoadParameters();

  //=======================================
  //TIMER interrupt register setup
  //=======================================
  
  TCCR1A = 0;
  TCCR1B = 0; //setting timer registers to zero
  TCNT1 = 64286;
  TCCR1B |= (1<<CS12); //prescaler 256
  TIMSK1 |= (1<<TOIE1); // enable the timer interrupts
  
  /*
   * formula -> TCNT1 = 65536 - (Fclock/(prescaler*Ftarget))
   * 20ms (50hz) -> TCNT1 = 64286
   */
 
  //splash screen before menu system begins.
  lcd.print(F("    The WATT    "));
  lcd.setCursor(0,1);
  lcd.print(F("   Sous Vide!   "));
  delay(2000); 
  
}

//============================================
//ISR
//===========================================

ISR(TIMER1_OVF_vect) 
{
  /*
   * frequency: every 20ms
   * purpose: every time ISR called, drive the output for the heater element
   * additionally, increment a seconds counter every second.
   */

   if(menu == Run && tempUnset == false)
   {
    //driving the output
    long now = millis();

    if(now - windowStart > windowsize)
      windowStart+=windowsize;
    if( (output>100) && (output>(now-windowStart))) //in the example ontime=output
      digitalWrite(RelayPin, HIGH);
    else
      digitalWrite(RelayPin, LOW);
   }
   if(menu != Run || tempUnset == true)
    digitalWrite(RelayPin, LOW); //make sure its off.
   
   
   if(count != 49)
    count++;
   else
   {
    count = 0;
    SafteySeconds++;
    if(timerSet == true)
    {
      if(hours == 0 && minutes == 0 &&seconds == 0)
      {
        timerSet = false;
        timerDone = true;
      }

      else
      {
        if(seconds != 0)
          seconds--;
        else
        {
          seconds = 59;
          if(minutes != 0)
            minutes--;
          else
          {
            minutes = 59;
            if(hours != 0)
            {
              hours--;
            }
          }
        }
      }
        
    }
   }
   if(SafteySeconds == 20)
   {
    SafteySeconds = 0;
    if(mostRecentTemp >= 200)
    {
      digitalWrite(RelayPin, LOW);
      menu = Idle; //will need to reenter run mode before the element can be turned on again, likely requiring a reset.
    }
   }
   
   TCNT1 = 64286; ////+- 1ms accuracy for second counter.
}

void loop() 
{

  switch(menu)
  {
    case Idle:
      idleScreen();
      break;
    case Run:
      startSousvide();
      break;
    case SetParams:
      setParamScreen();
      break;            
  }
  
}

void idleScreen(void)
{
  tempController.SetMode(MANUAL);
  lcd.noBlink();
  lcd.clear();
  delay(500);
  uint8_t buttons = 0;
  buttons = lcd.readButtons();
  digitalWrite(RelayPin, LOW); //safety
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.write(2);
  lcd.print(F("    Idling    "));
  lcd.write(3);
  while(true)
  {
    buttons = lcd.readButtons();
    lcd.setCursor(0,1);
    //read the temp sensor when its configured
    lcd.print("H2O Temp: ");
    if (sensors.isConversionAvailable(0))
    { 
      double tempC = sensors.getTempC(tempSensor);
      double tempF = DallasTemperature::toFahrenheit(tempC);
      mostRecentTemp = tempF;
      sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
    }
    lcd.print(mostRecentTemp);
    lcd.setCursor(15,1);
    lcd.write(1);
 
    if(buttons & (BUTTON_LEFT | BUTTON_RIGHT))
    {
      menu = Run;
      tempController.SetMode(AUTOMATIC);
      windowStart = millis();
      break; //start
    }
    if((buttons & BUTTON_SELECT) && (buttons & BUTTON_UP))
    {
      menu = SetParams;
      break; //setparams
    }
    
  }
}

void setParamScreen(void)
{
  bool notdone = true;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Set PID params. "));
  lcd.setCursor(0,1);
  lcd.print(F("DWN+SEL exits."));
  delay(1500);

  while(notdone)
  {
    switch(parammenu)
    {
      case SetKp:
        notdone = kp();
        break;
      case SetKi:
        notdone = ki();
        break;
      case SetKd:
        notdone = kd();
        break;
    }
  }

  menu = Idle;    
}

bool kp(void)
{
  uint8_t buttons = 0;
  bool select = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Tune Kp:"));
  lcd.setCursor(0,1);
  lcd.print(Kp);
  lcd.blink();
  delay(100);
  
  while(true)
  {
    buttons = lcd.readButtons();
 
    if((buttons & BUTTON_SELECT) && (buttons & BUTTON_DOWN))
    {
      SaveParameters();
      tempController.SetTunings(Kp,Ki,Kd);
      return(false);
    }
    if(buttons & BUTTON_DOWN)
    {
      if(select == false)
      {
        Kp -= 0.1;
        lcd.setCursor(0,1);
        lcd.print(Kp);
        delay(100);
      }
      if(select == true)
      {
        parammenu = SetKi;
        break;
      }
    }
    if(buttons & BUTTON_UP)
    {
      if(select == false)
      {
        Kp += 0.1;
        lcd.setCursor(0,1);
        lcd.print(Kp);
        delay(100);
      }
      if(select == true)
      {
        parammenu = SetKd;
        break;
      }
    }
    if(buttons & BUTTON_SELECT)
    {
      select = true;
      lcd.noBlink();
    }
  }
  return true;
}

bool ki(void)
{
  uint8_t buttons = 0;
  bool select = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Tune Ki:"));
  lcd.setCursor(0,1);
  lcd.print(Ki);
  lcd.blink();
  delay(100);
  
  while(true)
  {
    buttons = lcd.readButtons();
 
    if((buttons & BUTTON_SELECT) && (buttons & BUTTON_DOWN))
    {
      SaveParameters();
      tempController.SetTunings(Kp,Ki,Kd);
      return(false);
    }
    if(buttons & BUTTON_DOWN)
    {
      if(select == false)
      {
        Ki -= 0.1;
        lcd.setCursor(0,1);
        lcd.print(Ki);
        delay(100);
      }
      if(select == true)
      {
        parammenu = SetKd;
        break;
      }
    }
    if(buttons & BUTTON_UP)
    {
      if(select == false)
      {
        Ki += 0.1;
        lcd.setCursor(0,1);
        lcd.print(Ki);
        delay(100);
      }
      if(select == true)
      {
        parammenu = SetKp;
        break;
      }
    }
    if(buttons & BUTTON_SELECT)
    {
      select = true;
      lcd.noBlink();
    }
  }
  return true;
}

bool kd(void)
{
  uint8_t buttons = 0;
  bool select = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Tune Kd:"));
  lcd.setCursor(0,1);
  lcd.print(Kd);
  lcd.blink();
  delay(100);
  
  while(true)
  {
    buttons = lcd.readButtons();
 
    if((buttons & BUTTON_SELECT) && (buttons & BUTTON_DOWN))
    {
      SaveParameters();
      tempController.SetTunings(Kp,Ki,Kd);
      return(false);
    }
    if(buttons & BUTTON_DOWN)
    {
      if(select == false)
      {
        Kd -= 0.1;
        lcd.setCursor(0,1);
        lcd.print(Kd);
        delay(100);
      }
      if(select == true)
      {
        parammenu = SetKp;
        break;
      }
    }
    if(buttons & BUTTON_UP)
    {
      if(select == false)
      {
        Kd += 0.1;
        lcd.setCursor(0,1);
        lcd.print(Kd);
        delay(100);
      }
      if(select == true)
      {
        parammenu = SetKi;
        break;
      }
    }
    if(buttons & BUTTON_SELECT)
    {
      select = true;
      lcd.noBlink();
    }
  } 
  return true;
}

void startSousvide(void)
{
 bool notdone = true;
 while(notdone)
 {
   switch(runmenu)
   {
    case SetTemp:
      notdone = settemp();
      break;
    //case SetTimer:
    //  notdone = settimer();
    //  break;
    case RunScreen:
      notdone = runscreen();
      break;
   }
 }
}

bool settemp()
{
  lcd.noBlink();
  lcd.clear();
  delay(500);
  double temp;
  temp = steakMedRare;
  lcd.setCursor(0,0);
  lcd.write(2);
  lcd.print(F("   Set Temp   "));
  lcd.write(3);
  lcd.setCursor(0,1);
  lcd.print(F("    "));
  lcd.print(temp);
  lcd.write(1);
  //lcd.write(4);
  //lcd.write(5);
  lcd.blink();
  uint8_t buttons =0;
  while(true)
  {
    buttons = lcd.readButtons();
    if(buttons)
    {
      if(buttons &  (BUTTON_LEFT | BUTTON_RIGHT))
      {
        menu = Idle;
        return false;
      }
      if(buttons & BUTTON_UP)
      {
        temp+=0.5;
        lcd.setCursor(4,1);
        lcd.print(temp);
        lcd.write(1);
        //lcd.write(4);
        //lcd.write(5);
        lcd.blink();
        delay(200);
      }
      if(buttons & BUTTON_DOWN)
      {
        temp-=0.5;
        lcd.setCursor(4,1);
        lcd.print(temp);
        lcd.write(1);
        //lcd.write(4);
        //lcd.write(5);
        lcd.blink();
        delay(200);
      }
      if(buttons & BUTTON_SELECT)
      {
        lcd.noBlink();
        delay(500);
        setpoint = temp;
        break;
      }
    }
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("STARTING HEATING"));
  lcd.setCursor(0,1);
  lcd.print(F("    SEQUENCE    "));
  delay(2000);
  lcd.clear();
  tempUnset = false;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("RIGHT+SEL to"));
  lcd.setCursor(0,1);
  lcd.print(F("stop heat + exit"));
  runmenu = RunScreen;
  delay(2000);
  return true;
  

}


void settimer(void)
{
  //.setCursor(0,1);
  //lcd.print("not functional");
  //return;

  lcd.clear();
  timerSet = false;
  timerDone = false;
  hours = 0;
  minutes = 0;
  seconds = 0;
  enum HMS {H, M};
  HMS Time = M;
  //it auto moves to next char space! dank
  lcd.setCursor(0,0);
  lcd.print(F("Set: "));
  printTime(false);
  uint8_t buttons = 0;
  while(true)
  {
    //lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Set Timer "));
    printTime(false);
    tempController.Compute();
 
    while(true)
    {
      buttons = lcd.readButtons();

      if (sensors.isConversionAvailable(0))
      {
        double tempC = sensors.getTempC(tempSensor);
        double tempF = DallasTemperature::toFahrenheit(tempC);
        input = tempF;
        mostRecentTemp = tempF;
        sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
      }
      lcd.setCursor(0,1);
      lcd.print("H2O Temp: ");
      lcd.print(mostRecentTemp);
      lcd.write(1);
      
      if(buttons & BUTTON_UP)
      {
        if(Time == M)
        {
          if(minutes != 59)
            minutes++;
          else
            minutes = 0;
        }
        else //hours
        {
          if(hours != 23)
            hours++;
          else
            hours = 0;
        }
        break;
      }

      if(buttons & BUTTON_DOWN)
      {
        if(Time == M)
        {
          if(minutes != 0)
            minutes--;
          else
            minutes = 59;
        }
        else //hours
        {
          if(hours != 0)
            hours--;
          else
            hours = 23;
        }
        break; 
      }

      if(buttons & BUTTON_SELECT)
      {
        timerSet = true;
        songPlayed = false;
        return;
        //why did i write all this code that literally was so stupid
        /*
        if(hours > 0 && minutes > 0)
        {
         hours--;
         minutes = 59;
         seconds = 59;
         timerSet = true;
         return;
        }
        else if(minutes > 0)
        {
          minutes--;
          seconds = 59;
          timerSet = true;
          return;
        }
        else
        {
          timerSet = true;
          return;
        }
        */
      }

      if(buttons & BUTTON_LEFT)
        Time = H;
      if(buttons & BUTTON_RIGHT)
        Time = M;
    }
    
  }
}
void printTime(bool printSeconds)
{
  //prints time in a way that looks good
  //wont set cursor for you, must be preceded by a call to setcursor

  if(printSeconds)
  {
    if(seconds > 9)
      secondssingle = false;
    else
      secondssingle = true;
  }
  
  if(minutes > 9)
    minutesssingle = false;
  else
    minutesssingle = true;
  if(hours > 9)
    hourssingle = false;
  else
    hourssingle = true;
  if(hourssingle)
    lcd.print(F("0"));
  lcd.print(hours);
  lcd.print(F(":"));
  if(minutesssingle)
    lcd.print(F("0"));
  lcd.print(minutes);

  if(printSeconds)
  {
    lcd.print(F(":"));
    if(secondssingle)
      lcd.print(F("0"));
    lcd.print(seconds);
  }
  
  //lcd.blink();
}  

bool runscreen()
{
  lcd.clear();
  uint8_t buttons = 0;
  bool alreadycleared = false;
  while(true)
  {
    buttons = lcd.readButtons();
    // Read the input:
    if (sensors.isConversionAvailable(0))
    {
      double tempC = sensors.getTempC(tempSensor);
      double tempF = DallasTemperature::toFahrenheit(tempC);
      input = tempF;
      mostRecentTemp = tempF;
      sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
    }
    tempController.Compute(); //!!*****!!**%%&&%#@#$!@#$@#$* put in timer loop?

    if(tuning)
    {
      if(autoTuner.Runtime())
      {
        finishTuning();
        menu = Idle;
        return false;
      }
    }
    
    lcd.setCursor(0,0);
    lcd.print(F("Timer:  "));
    
    if(timerDone)
    {
      if(!alreadycleared)
      {
        lcd.clear();
        alreadycleared = true;
        
      }
      lcd.setCursor(0,0);
      lcd.print(F("Timer: DONE!"));
      if(!songPlayed)
      {
        for(int i=0; i < 5; i++)
        {
          tone(SpeakerPin, timerSong[i], 1000/durations[i]);
          delay(400); 
        }
        songPlayed = true;
        noTone(SpeakerPin);
        //playItAgain = seconds;
      }
      
    }
      
    else if(timerSet)
      printTime(true);
      
    else
    {
      lcd.print(F("Unset   "));
      lcd.write(3);
    }
      
    lcd.setCursor(0,1);
    lcd.print("H2O Temp: ");
    lcd.print(mostRecentTemp);
    lcd.write(1);

    if((buttons & BUTTON_RIGHT) && (buttons & BUTTON_SELECT))
    {
      menu = Idle;
      return false;
    }
    if(buttons & BUTTON_RIGHT)
    {
      settimer();
      return false;
    }
    if((buttons & BUTTON_LEFT) && (buttons & BUTTON_SELECT) && (abs(input - setpoint) < 0.5))
    {
      beginTuning();
    }
    
  }
}

void beginTuning(void)
{
  autoTunemoderemember = tempController.GetMode();
  autoTuner.SetNoiseBand(autoTunenoise);
  autoTuner.SetOutputStep(autoTunestep);
  autoTuner.SetLookbackSec((int)autoTunelookback);
  tuning = true;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("  AUTOTUNE IN  "));
  lcd.setCursor(0,1);
  lcd.print(F("    PROGRESS    "));
}

void finishTuning(void)
{
  tuning = false;
  Kp = autoTuner.GetKp();
  Kd = autoTuner.GetKd();
  Ki = autoTuner.GetKi();
  tempController.SetTunings(Kp, Ki, Kd);
  tempController.SetMode(autoTunemoderemember);
  SaveParameters();
  lcd.clear() ;
  lcd.setCursor(0,0);
  lcd.print(F("TUNING PROCESS"));
  lcd.setCursor(0,1);
  lcd.print(F("    FINISHED    "));
  delay(10000);
}

void EEPROM_writeDouble(int adress, double val)
{

  byte* bp = (byte*)(void*)&val; 
  //take the adress of the double and convert to byte pointer
  
  for(int i=0; i<sizeof(val); i++)
  {
    //loop runs for as many times as val has bytes stored
    EEPROM.write(adress, *bp); //write takes a value, deref byte pointer
    adress++;
    bp++;
  }
  
}

double EEPROM_readDouble(int adress)
{
  double val = 0.0;

  byte* bp = (byte*)(void*)&val;

  for(int i=0; i<sizeof(double); i++) //change duble to val if issues..
  {
    *bp = EEPROM.read(adress);
    bp++;
    adress++;
  }

  return val;
}

void SaveParameters(void)
{
  if(setpoint != EEPROM_readDouble(setpointAdr))
   {
      EEPROM_writeDouble(setpointAdr, setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAdr))
   {
      EEPROM_writeDouble(KpAdr, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAdr))
   {
      EEPROM_writeDouble(KiAdr, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAdr))
   {
      EEPROM_writeDouble(KdAdr, Kd);
   }
}

void LoadParameters()
{
  // Load from EEPROM
   setpoint = EEPROM_readDouble(setpointAdr);
   Kp = EEPROM_readDouble(KpAdr);
   Ki = EEPROM_readDouble(KiAdr);
   Kd = EEPROM_readDouble(KdAdr);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(setpoint))
   {
     setpoint = steakMedRare;
   }
   if (isnan(Kp))
   {
     Kp = 100;
   }
   if (isnan(Ki))
   {
     Ki = 3.0;
   }
   if (isnan(Kd))
   {
     Kd = 0.2;
   }  
}


