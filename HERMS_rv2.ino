/*
Copyright (c) 2012, Ben Rouse
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <EEPROM.h>
#include "double_to_eeprom.h"
#include <PID_v1_ITerm_Clamp.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>

#define BADTEMP -1000
#define MAXSTEPS 10
#define RELAYPIN 4
#define TEMPCOLUMN 0
#define SETPOINTCOLUMN 10
#define DSPIN 6
//#define RECV_PIN 3
#define LEFTBUTTON 14
#define RIGHTBUTTON 15
#define SELECTBUTTON 16

// preset mash regimes: { number of steps, step temp, step time (minutes), etc... }
//const byte zwickel[9]={4, 52, 20, 63, 30, 72, 30, 76, 0};
//const byte ale[9]={4, 38, 20, 68, 30, 72, 30, 76, 0};

//IRrecv irrecv(RECV_PIN);               // define IR Receiver
//decode_results results;                // IR receiver decoded results

//define unique sensor serial code
//call search_devices() to determine codes
//byte HERMS[8]  = {0x10, 0x04, 0x91, 0x60, 0x01, 0x08, 0x00, 0x10};    // DS18B20 64bit address
int HighByte, LowByte, SignBit, Whole, Fract, TReading, Tc_100, FWhole;
int sampleTime=1000;
byte Conversion_started=0;
byte Busy=0;
byte HERMS[8];
byte time=0;
byte steps=1;
byte currstep=0;
byte steptime=0; 
byte steptimes[10];
byte inputval=0;
byte start_heater=0;
double Kp, Ki, Kd, KiClampLow, KiClampHigh;
double Temperature_C;
double Output;
double steptemp;
double steptemps[MAXSTEPS];
OneWire  ds(6);                        // define a Dallas OneWire bus on pin 6
unsigned int WindowSize;
unsigned int last_result;
unsigned long windowStartTime;
unsigned long start_timer=0;             // holds value of millis
unsigned long curr_timer;
unsigned long stepstart=0;
LiquidCrystal lcd(12, 2, 7, 8, 9, 10); // LiquidCrystal(rs, enable, d4, d5, d6, d7) 
volatile int state = LOW;
volatile int modeSet = 0;

//
// degrees rise per second = number of kilowatts / (volume * 4.184)
//
// 0.1, 2, 1        1-2deg overshoot, little settling
// 1, 0, 100        MASSIVE overshoot (20deg)
// 0.5, 0, 5        Won't get to set point
// 0.5, 0, 7.5      Closer to set point, but not quite there. slow to get there.
// 0.8, 0, 10       
// 5, 5, 10         (Iterm clamped between 0 and 20) MASSIVE OVERSHOOT (on jug)
// 5, 0.1, 100
// 6, 0.1, 50 (jug)
// 5, .05, 98 (jug) 0.3degC overshoot
PID myPID(&Temperature_C, &Output, &steptemp,5,0.05,98, DIRECT); //Specify the links and initial tuning parameters

byte RightArrow[8] =
{
  B10000,
  B11000,
  B11100,
  B11110,
  B11110,
  B11100,
  B11000,
  B10000
};
byte LeftArrow[8] =
{
  B00001,
  B00011,
  B00111,
  B01111,
  B01111,
  B00111,
  B00011,
  B00001
};

byte ArrowShaft[8] =
{
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000
};
byte DegreeSymbol[8] =
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

// function prototypes
// functions are declared after the main loop, but prototyped here, so the loop doesn't get confused
//
void decimate(char[],int);          // place a decimal in the appropriate place for ASCII output
void get_temp(byte *);              // get temperature reading from the OneWire bus.
void search_devices();              // search the OneWire bus for device addresses
int validate_temp(char[], double);  // validate that the temperature is within range
void lcdPrintLabels();              // print all the labels to the LCD
void clearlcd();                    // clear LCD
unsigned long steptimer();          // The step timing code lives here
void printTemp(int, int, char[]);   // print temperature to LCD
void PID_HEAT ();                   // The brains - the PID algorithm lives here
//void IR_RECV();                     // Read IR receiver module
void printStepConfigLabels(byte);   // print step configuration mode labels to LCD
void printConfigLabels(byte);       // print configuration mode labels to LCD
void configSteps();                 // Configure the step temperatures and times
void configNumSteps();              // Configure the number of mash steps
void configMode();                  // Configuration mode
byte getInput();                    // get input (either from buttons or IR receiver)
void loadPIDsettings();             // load PID tuning settings from eeprom
void savePIDsettings();             // save PID tuning settings to eeprom
void customConfigMode(byte);        // config menu for custom programs
void printCustomConfigLabels(byte, byte);  // print menu labels for custom program menus
void loadCustomProgram(byte);       // load custom program steps from eeprom
void saveCustomProgram(byte);       // save custom program steps to eeprom
void PIDconfigLabels(byte mode);    // print LCD labels for PID config mode
void PIDconfigMode();               // PID config mode
void loadPIDparameters();           // load PID parameters from EEPROM
void savePIDparameters();           // save PID parameters to EEPROM

void setup(void) {
  char buf[12];
  char incomingByte;

  steptemps[0]=0;
  // Setup LCD output
  pinMode(7, OUTPUT);         // LCD
  pinMode(9, OUTPUT);         // LCD
  pinMode(14, INPUT);        // LEFT BUTTON
  pinMode(15, INPUT);        // RIGHT BUTTON
  pinMode(16, INPUT);        // SELECT BUTTON
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Initialising....");
  lcd.createChar(1,RightArrow);
  lcd.createChar(0,ArrowShaft);
  lcd.createChar(2,DegreeSymbol);
  lcd.createChar(3,LeftArrow);
  //Serial.begin(9600); // start serial port
  //irrecv.enableIRIn(); // Start the IR receiver  
  pinMode(RELAYPIN, OUTPUT); // set RELAYPIN to output. RELAYPIN switches the HERMS relay on or off
  digitalWrite(RELAYPIN, LOW); // HERMS heater initial setting is OFF
  search_devices(HERMS); //enable this line to get sensor device id. comment out when finished
  //WindowSize=1000; //5000 for the HERMS?? // now loaded from EEPROM
  loadPIDparameters();
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(0,WindowSize);
  windowStartTime=millis();
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleTime); //HERMS: 5000
  myPID.SetITermClamp(KiClampLow,KiClampHigh);  //HERMS: 0,20
  //delay(1000);
  /*
  Serial.flush();
  Serial.println();
  delay(1000);
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
  */
  
  // Launch in config mode
  //configMode();
  // Start temperature conversion on DS18B20 before entering main loop
  get_temp(HERMS);
  clearlcd();
  lcdPrintLabels();
  inputval=0;
  steptemp=steptemps[currstep];
  delay(1000);
  get_temp(HERMS);
  //start_heater=1;
}



// ########### L O O P ###########
void loop(void) {
  char buf[12];
  
   
    //TCCR2B=0x00; // disable interrupt 2
    //delay(800);
    get_temp(HERMS);
    //TCCR2B=0x02; // re-enable interrupt 2
    if(validate_temp(buf,Temperature_C)) {    // if the temp is valid, convert it to ASCII and put it in buf
      decimate(buf,1);              // add the decimal place to the ASCII string
      printTemp(TEMPCOLUMN,0,buf);
    }
     
    if(validate_temp(buf,steptemp)) {
      buf[2]=NULL;
      if(steptemp<10){
        lcd.setCursor(SETPOINTCOLUMN,0);
        lcd.print(" ");
        lcd.print((int)steptemp);
        //printTemp(SETPOINTCOLUMN+1,0,buf);
      } else {
        printTemp(SETPOINTCOLUMN,0,buf);
        //decimate(buf,1);
      }
    }
    lcd.setCursor(0,1);
    if(start_heater==1){
      lcd.print("S:");
      if(currstep+1<10){
        lcd.print("0");
      } 
      lcd.print(currstep+1);
      lcd.print(" ");
      lcd.print((int)trunc(steptimer()/60));  //print time left in current step
      lcd.print(":");
      if((steptimer()%60)<10){
        lcd.print("0");
      }
      lcd.print(steptimer()%60);
      lcd.print(" ");
    } else {
      lcd.print("  END PROGRAM   ");
    }
    getInput();
    if(inputval==1){
      steptemp++;
      inputval=0;
    }
    
    if(inputval==2){
      steptemp--;
      inputval=0;
    }
    
    
    if(inputval==3){
      inputval=0;
      configMode();
      clearlcd();
      lcdPrintLabels();
    }
    
    //IR_RECV();  
     
    // DEBUG OUTPUT  
    /*
    Serial.print("start_timer: ");
    Serial.print(start_timer);
    Serial.print("\n");
    Serial.print("time remaining: ");
    Serial.print(steptimer());
    Serial.print("seconds \n");
    Serial.print("set point: ");
    Serial.print(steptemp);
    Serial.print("\n");
    Serial.print("current temp: ");
    Serial.print(Temperature_C);
    Serial.print("\n");
    */
    if(start_heater){
      PID_HEAT();
    }
    
}

/*
void IR_RECV()
{
  delay(800);
  if (irrecv.decode(&results)) {
    if(results.value==0xFFFFFFFF){   // special code for repeat (button held down)
      results.value=last_result;
    }

    irrecv.resume();
  }
}
*/

void clearlcd()
{
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
}

unsigned long steptimer()
{
  unsigned long seconds_left;
  
  curr_timer=millis();
  if(start_timer>0){
    seconds_left=((steptimes[currstep]*60)-((curr_timer-start_timer)/1000)); //seconds for debug
    if((steptimes[currstep]*60000)-(curr_timer-start_timer)>4294967000){ // 
      currstep++;                    //increment the step counter
      /*
      // debug output
      Serial.print("current step: ");
      Serial.print(currstep);
      Serial.print("\n");
      Serial.print("steps: ");
      Serial.print(steps);
      Serial.print("\n");
      */
      if(currstep>(steps-1)){            //just finished the last step
        start_heater=0;
        steptemp=0;
        currstep=steps-1;
      }
      start_timer=0;                 //zero the start timer
      steptemp=steptemps[currstep];  //set  the set point to the new step temp
    }
    return seconds_left;
  } else {
    return steptimes[currstep]*60; //seconds for debug
  }
}

void lcdPrintLabels()
{

  lcd.setCursor(0,0); // cursor to line 0, char 0
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,0); // cursor to line 0, char 0
  lcd.print("----");
  lcd.setCursor(TEMPCOLUMN+4,0);
  lcd.write((uint8_t)2);
  lcd.print('C');
  lcd.setCursor(TEMPCOLUMN+6,0);
  lcd.print(" ");
  lcd.write((uint8_t)0);
  lcd.write((uint8_t)1);
  lcd.print(" --");
  lcd.write((uint8_t)2);
  lcd.print('C');
  lcd.setCursor(11,1);
  lcd.print("  ");
  lcd.setCursor(13,1);
  lcd.print("   ");
}

void printTemp(int col, int row, char buf[])
{
  lcd.setCursor(col,row); // cursor to line 1, char 11
  lcd.print(buf);
}

void decimate(char test[],int dec) {
  int i=0;  
  int length=strlen(test);
  char msg[10]="";

  strcpy(msg,test);  // copy string to temporary working variable

  if (length <= dec) {  // if the string length is equal or less than the number of digits after the decimal place, then
    for(i=dec;i>(dec-length);i--){  
      msg[i] = msg[i-(dec-length+1)];  // move every digit to make place for the decimal
    }
    for(i=0;i<(dec+1-length);i++){
      msg[i]='0';    // pad with leading zeros.
    }
    length = strlen(msg);
  }
  for (i=length;i>(length-dec);i--){
    msg[i]=msg[i-1]; // move digits to make place for the decimal
  }
  msg[length-dec]='.'; // place the decimal

  strcpy(test,msg); // copy new string back to original variable
} 

int validate_temp(char buf[], double temp)
{
  unsigned char valid=0;
  if(temp*10>1250 || temp*10 < -550){    // DS18S20 measures between -55 and 125degC. Values outside this range are invalid
    strcpy(buf,"----");                 // write dashes to show invalid data 
    valid=0;                            // set return value
  } 
  else {
    valid=1;                            // set return value
    itoa(temp*10,buf,10);               // convert integer to ASCII string
  }
  return valid;                          // return true if valid, false if invalid
}

void get_temp(byte* addr)
{
  byte present = 0;
  byte i;
  byte data[12];
  int foo, bar;

  ds.reset();
  ds.select(addr);
  if(!(Conversion_started)){
    ds.write(0x44,0);         // start conversion, with parasite power off at the end
    Conversion_started=1;
    return;
  } else {
    Busy=ds.read_bit();
    if(Busy==0){
      return;
    }
    
  ds.reset();
  ds.select(addr);   
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

 
  LowByte = data[0];      // load all 8 bits of the LSB
  HighByte = data[1];     // load all 8 bits of the MSB

  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  
  if (SignBit) {
    TReading = -TReading;
  }
  Tc_100 = ((6.25 * TReading) + (TReading / 4));    // multiply by (100 * 0.0625) or 6.25
  //Serial.print(Tc_100);
  if (SignBit) {
    bar = -1;
  } else {
    bar = 1;
  }
  Temperature_C=(double)Tc_100/100;
  Conversion_started=0;
  return;
  }
}

void search_devices(byte addr[8])
{
  //byte addr[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  //byte addr[8];
  int i=0;

  if ( !ds.search(addr)) {
    ds.reset_search();
    return;
  }
  
  if ( OneWire::crc8( addr, 7) != addr[7]) {
    //Serial.print("CRC is not valid!\n");
    return;
  }

  if ( addr[0] != 0x28) {
    //Serial.print("Device is not a DS18B20 family device.\n");
    return;
  }
  return;
}

void PID_HEAT (void){
  /*
  if((steptemp - Temperature_C)>5){
    digitalWrite(RELAYPIN,HIGH);
    if((steptemp - Temperature_C)<6){
      myPID.Compute();
    }
  }
  else
  */
  {
    myPID.Compute();
    unsigned long now = millis();
    
    while(now - windowStartTime>WindowSize)
    {                                     //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if((Output*(WindowSize/100)) > now - windowStartTime) {
      digitalWrite(RELAYPIN,HIGH);
      lcd.setCursor(12,1);
      lcd.print("HEAT");
    } else {
      digitalWrite(RELAYPIN,LOW);
      lcd.setCursor(12,1);
      lcd.print("    ");
    }
     
    if(Temperature_C>=steptemp && start_timer==0){
      //Serial.print("\n START TIMER \n");
      start_timer=millis();
    }
  }
}



void printStepConfigLabels(byte steptemptime)
{
  int steploop;
  lcd.setCursor(0,0); // cursor to line 0, char 0
  switch (steptemptime){
    case (1): //temp
      lcd.print(" Step    Temp   ");
      lcd.setCursor(0,1);
      lcd.print("       ");
      lcd.write((uint8_t)3);
      lcd.print("       ");
      lcd.write((uint8_t)1);
      lcd.print(" ");
      break;
    case (2): //time
      lcd.print(" Step    Time   ");
      lcd.setCursor(0,1);
      lcd.print("       ");
      lcd.write((uint8_t)3);
      lcd.print("       ");
      lcd.write((uint8_t)1);
      lcd.print(" ");
      break;
    default:
      break;
  }
  //lcd.write((uint8_t)0); //-
  //lcd.write((uint8_t)1);  //>
  //lcd.write((uint8_t)2);  //degree symbol
}

void configNumSteps()
{
  byte stepexit=0;

  lcd.setCursor(0,0); // cursor to line 0, char 0
  lcd.print("Number of Steps ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  while(!stepexit){
    lcd.setCursor(0,1);
    lcd.print(steps);
    lcd.print("     ");
    getInput();
    switch (inputval){
      case (1):  //right
        steps++;
        if (steps==11){
          steps=10;
        }
        inputval=0;
        break;
      case (2):  //left
        steps--;
        if (steps==0){
          steps=1;
        }
        inputval=0;
        break;
      case (3):  //enter
        stepexit=1;
        inputval=0;
        //lcd.setCursor(0,10);
        //lcd.write("SEL");
        //Serial.write("  SEL  ");
        break;
      default:
        break;  
    }
    //delay(300);
  }
  stepexit=0;
}

void configSteps()
{
  byte steploop, stepdataloop, stepexit;
  for (steploop=0;steploop<steps;steploop++){      //loop through steps
    for (stepdataloop=1;stepdataloop<=2;stepdataloop++){    //set temp and time for each step
      printStepConfigLabels(stepdataloop);          // print time or temp appropriate labels
      lcd.setCursor(2,1);
      lcd.print(steploop+1);
      while(!stepexit){                                  // do the actual config editing
        getInput();
        lcd.setCursor(9,1);
        if(stepdataloop==1){
          lcd.print(steptemps[steploop]);
          lcd.print(" ");
        } else {
          lcd.print(steptimes[steploop]);
          lcd.print("   ");
        }
        switch (inputval){
        case (1):  //up/right
          if(stepdataloop==1){
            steptemps[steploop]++;
          } else {
            steptimes[steploop]++;
          }
          inputval=0;
          break;
        case (2):  //down/left
          if(stepdataloop==1){
            steptemps[steploop]--;
          } else {
            steptimes[steploop]--;
          }
          inputval=0;
          break;
        case (3):  //enter/select
          stepexit=1;
          inputval=0;
          //Serial.write("  SEL  ");
          break;
        default:
          break;  
        }
      }
      stepexit=0;
    }
  }
  steptemp=steptemps[currstep];
}

byte getInput()
{
  
  //IR_RECV();
  
  if (digitalRead(LEFTBUTTON)==LOW){
    delay (120);    //debounce
    if (digitalRead(LEFTBUTTON)==LOW){
      //Serial.print("LEFT\n");
      inputval=2;
      //return((byte)2);
    }
  }
  if (digitalRead(RIGHTBUTTON)==LOW){
    delay (120);    //debounce
    if (digitalRead(RIGHTBUTTON)==LOW){
      //Serial.print("RIGHT\n");
      inputval=1;
      //return((byte)1);
    }
  }
  
  if (digitalRead(SELECTBUTTON)==LOW){
    delay (120);    //debounce
    if (digitalRead(SELECTBUTTON)==LOW){
      //Serial.print("SELECT\n");
      inputval=3;
      //return((byte)3);
    }
  }
}

void configMode(){
  byte maxModes=7;
  byte configexit=0;
  byte mode=1;
  byte enter=0;
  byte presetloop=0;
  
  digitalWrite(RELAYPIN, LOW);  // turn off heater while in config mode
  while(!configexit){
    while(!enter){
      printConfigLabels(mode);
      //mode=selectMode();
      getInput();
      //delay(500);
      switch (inputval){
        case (1):  //up/right
          mode++;
          inputval=0;
          break;
        case (2):  //down/left
          if(mode==0){
            mode=maxModes;
          } else {
            mode--;
          }
          inputval=0;
          break;
        case (3):  //enter/select
          enter=1;
          inputval=0;
          break;
        default:
          break;  
      }
      if(mode>maxModes){
        mode=0;
      }   
    }
    enter=0;   
    //getInput();
    //delay(500);
    switch (mode){
      case(0):  //exit menu
        configexit=1;
        if(start_heater==0){              // if program has ended
          start_heater=1;                 // start the program
          currstep=0;                     // reset to the first step
        }
        steptemp=steptemps[currstep];   // set the set point to the current step temp in the program
        break;
      case(1):  //number of steps
        configNumSteps();
        break;
      case(2):  //set steps
        configSteps();
        break;
      case(3):  //preset 1 (ale)
        //load preset steps
        steps=5;
        steptemps[0]=42;
        steptimes[0]=40;
        steptemps[1]=52;
        steptimes[1]=10;
        steptemps[2]=66;
        steptimes[2]=30;
        steptemps[3]=72;
        steptimes[3]=30;
        steptemps[4]=78;
        steptimes[4]=0;
        for (presetloop=5;presetloop<MAXSTEPS;presetloop++){
          steptemps[presetloop]=0;
          steptimes[presetloop]=0;
        }
        mode=0;
        steptemp=steptemps[currstep];
        break;
      case(4):  //preset 2 (zwickel lager)
        //load preset steps
        steps=4;
        steptemps[0]=52;
        steptimes[0]=20;
        steptemps[1]=63;
        steptimes[1]=45;
        steptemps[2]=72;
        steptimes[2]=30;
        steptemps[3]=78;
        steptimes[3]=5;
        for (presetloop=4;presetloop<MAXSTEPS;presetloop++){
          steptemps[presetloop]=0;
          steptimes[presetloop]=0;
        }
        mode=0;
        steptemp=steptemps[currstep];
        break;
      case(5):
        customConfigMode(1);
        // load custom 1 from eeprom
        // save custom 2 to eeprom
        break;
      case(6):  
        customConfigMode(2);
        // load custom 2 from eeprom
        // save custom 2 to eeprom
        //configexit=1;  //exit config mode
        break;
      case(7):
        // configure PID tunings
        // save/load
        PIDconfigMode();
        break;
      default:
        break;
    }
  }
}

void printConfigLabels(byte mode)
{
  int steploop;
  lcd.setCursor(0,0); // cursor to line 0, char 0
  lcd.print("   Configure    ");
  // option to start / stop ???
  switch (mode){
    case (0):   
      lcd.setCursor(0,1);
      //lcd.write(0x7F);        //built in left arrow
      lcd.write((uint8_t)3);    //custom left arrow
      lcd.print(" EXIT CONFIG  ");
      lcd.write((uint8_t)1);    //custom right arrow
      //lcd.write(0x7E);        //built in right arrow
      //lcd.setCursor(15,0);
      //lcd.print(mode);
      break;
    case (1):
      lcd.setCursor(0,1);
      //lcd.write(0x7F);
      lcd.write((uint8_t)3);
      lcd.print(" NUM OF STEPS ");
      lcd.write((uint8_t)1);
      //lcd.write(0x7E);
      break;
    case (2):
      lcd.setCursor(0,1);
      //lcd.write(0x7F);
      lcd.write((uint8_t)3);
      lcd.print(" CONFIG STEPS ");
      lcd.write((uint8_t)1);
      //lcd.write(0x7E);
      break;
    case (3):
      lcd.setCursor(0,1);
      //lcd.write(0x7F);
      lcd.write((uint8_t)3);
      lcd.print("  PRESET ALE  ");
      lcd.write((uint8_t)1);
      //lcd.write(0x7E);
      break;
    case (4):
      lcd.setCursor(0,1);
      //lcd.write(0x7F);
      lcd.write((uint8_t)3);
      lcd.print(" PRESET LAGER ");
      lcd.write((uint8_t)1);
      //lcd.write(0x7E);
      break;
    case (5):
      lcd.setCursor(0,1);
      //lcd.write(0x7F);
      lcd.write((uint8_t)3);
      lcd.print("   CUSTOM 1   ");
      lcd.write((uint8_t)1);
      //lcd.write(0x7E);
      break;
    case (6):
      lcd.setCursor(0,1);
      //lcd.write(0x7F);
      lcd.write((uint8_t)3);
      lcd.print("   CUSTOM 2   ");
      lcd.write((uint8_t)1);
      //lcd.write(0x7E);
      break;
    case (7):
      lcd.setCursor(0,1);
      lcd.write((uint8_t)3);
      lcd.print("  PID TUNING  ");
      lcd.write((uint8_t)1);
      break;
    default:
      break;
  }
      

  //switch (mode)
  // mode 1: 
  //lcd.write((uint8_t)0); //-
  //lcd.write((uint8_t)1);  //>
  //lcd.write((uint8_t)2);  //degree symbol
  

}

void loadCustomProgram(byte customProgramNumber)
{
    byte stepaddress, steploop, addressOffset;
    
    if(customProgramNumber==1){
      addressOffset=0;
    }
    if(customProgramNumber==2){
      addressOffset=21;
    }
    clearlcd();    
    lcd.setCursor(0,1);
    lcd.print(" Reading EEPROM ");    
    
    steps=EEPROM.read(stepaddress+addressOffset);
    // DEBUG 
    /*
    Serial.print(stepaddress+addressOffset);
    Serial.print(",");
    Serial.print((byte)steps);
    Serial.print("\n");
    */
    for (steploop=0;steploop<MAXSTEPS;steploop++){
      stepaddress=(steploop*2)+addressOffset+1;
      steptemps[steploop]=(double)EEPROM.read(stepaddress);
      // DEBUG
      /*
      Serial.print(stepaddress);
      Serial.print(",");
      Serial.print((byte)steptemps[steploop]);
      Serial.print("\n");
      */
      steptimes[steploop]=(double)EEPROM.read(stepaddress+1);
      /*
      Serial.print(stepaddress+1);
      Serial.print(",");
      Serial.print((byte)steptimes[steploop]);
      Serial.print("\n");
      */
    }
    /*
    lowB = ((array[loop] >> 0) & 0xFF);
    highB = ((array[loop] >> 8) & 0xFF);
    */
}

void saveCustomProgram(byte customProgramNumber)
{
    byte stepaddress, steploop, addressOffset;
    
    if(customProgramNumber==1){
      addressOffset=0;
    }
    if(customProgramNumber==2){
      addressOffset=21;
    }
    clearlcd();    
    lcd.setCursor(0,1);
    lcd.print(" Writing EEPROM ");    
    EEPROM.write(stepaddress+addressOffset, steps);
    // DEBUG
    /*
    Serial.print(stepaddress+addressOffset);
    Serial.print(",");
    Serial.print((byte)steps);
    Serial.print("\n");*/
    for (steploop=0;steploop<MAXSTEPS;steploop++){
      stepaddress=(steploop*2)+addressOffset+1;
      EEPROM.write(stepaddress, (byte)steptemps[steploop]);
      // DEBUG
      /*
      Serial.print(stepaddress);
      Serial.print(",");
      Serial.print((byte)steptemps[steploop]);
      Serial.print("\n");
      */
      EEPROM.write(stepaddress+1, (byte)steptimes[steploop]);
      /*
      Serial.print(stepaddress+1);
      Serial.print(",");
      Serial.print((byte)steptimes[steploop]);
      Serial.print("\n");
      */
    }
    /*
    lowB = ((array[loop] >> 0) & 0xFF);
    highB = ((array[loop] >> 8) & 0xFF);
    */
}

void printCustomConfigLabels(byte loadsave, byte programNumber)
{
  int steploop;
  lcd.setCursor(0,0); // cursor to line 0, char 0
  switch (loadsave){
    case (1): //temp
      lcd.print("    Custom ");
      lcd.print(programNumber);
      lcd.print("    ");
      lcd.setCursor(0,1);
      lcd.write((uint8_t)3);
      lcd.print("     LOAD     ");
      lcd.write((uint8_t)1);
      lcd.print(" ");
      break;
    case (2): //time
      lcd.print("    Custom ");
      lcd.print(programNumber);
      lcd.print("    ");
      lcd.setCursor(0,1);
      lcd.write((uint8_t)3);
      lcd.print("     SAVE     ");
      lcd.write((uint8_t)1);
      lcd.print(" ");
      break;
    default:
      break;
  }
  //lcd.write((uint8_t)0);  //-
  //lcd.write((uint8_t)1);  //>
  //lcd.write((uint8_t)2);  //degree symbol
  //lcd.write((uint8_t)3);  //<
}

void customConfigMode(byte programNumber){
  byte maxModes=2;
  byte configexit=0;
  byte mode=1;
  byte enter=0;
  
  clearlcd();
  while(!configexit){
    while(!enter){
      printCustomConfigLabels(mode,programNumber);
      //mode=selectMode();
      getInput();
      //delay(500);
      switch (inputval){
        case (1):  //up/right
          //Serial.print("RIGHT\n");
          if(mode==maxModes){
            mode=1;
          } else {
            mode++;
          }
          inputval=0;
          break;
        case (2):  //down/left
          //Serial.print("LEFT\n");
          if(mode==0){
            mode=maxModes;
          } else {
            mode--;
          }
          inputval=0;
          break;
        case (3):  //enter/select
          //Serial.print("SELECT\n");
          enter=1;
          inputval=0;
          break;
        default:
          break;  
      }   
    }
    enter=0;   
    //getInput();
    //delay(500);
    switch (mode){
      case(1):  //load
        //Serial.print("LOAD\n");
        loadCustomProgram(programNumber);
        //Serial.print("LOAD RETURNING\n");
        clearlcd();
        configexit=1;
        break;
      case(2):  //save
        //Serial.print("SAVE\n");
        saveCustomProgram(programNumber);
        //Serial.print("SAVE RETURNING\n");
        clearlcd();
        configexit=1;
        break;
      default:
        break;
    }
  }
  clearlcd();
}

void PIDconfigLabels(byte mode)
{
}

void PIDconfigMode()
{
  byte settingsNumber=8;
  byte labelIndent, settingsloop, settingsdataloop, settingsexit=0;
  double valueIncrement, value;
  char* settingLabel[] ={"Proportional","Integral","Derivitive","Window Size","Sample Interval","I Clamp Low","I Clamp High","SAVE/EXIT"};
  clearlcd();
  for (settingsloop=0;settingsloop<settingsNumber;settingsloop++){      //loop through settingss
    //PIDconfigLabels(settingsdataloop);          // print appropriate labels
    switch (settingsloop){
      case (0):  // Kp
        valueIncrement=0.1;
        value=Kp;
        labelIndent=2;
        break;
      case (1):  // Ki
        valueIncrement=0.01;
        value=Ki;
        labelIndent=4;
        break;
      case (2):  // Kd
        valueIncrement=1;
        value=Kd;
        labelIndent=3;
        break;
      case (3):  // Window Size
        valueIncrement=500;
        value=(double)WindowSize;
        labelIndent=2;
        break;
      case (4):  // Sample Interval
        valueIncrement=500;  // in milliseconds
        value=(double)sampleTime;
        labelIndent=0;
        break;
      case (5):  // I Clamp lower limit
        valueIncrement=0.01;
        value=KiClampLow;
        labelIndent=2;
        break;
      case (6):  // I Clmap higher limit
        valueIncrement=1;
        value=KiClampHigh;
        labelIndent=2;
        break;
      case (7): // save/exit
        valueIncrement=1;
        labelIndent=4;
        value=0;
        break;
      default:  // THIS SHOULDN'T HAPPEN!
      break;
    }
    clearlcd();
    while(!settingsexit){                                  // do the actual config editing
      lcd.setCursor(labelIndent,0);
      lcd.print(settingLabel[settingsloop]);
      lcd.setCursor(3,1);
      lcd.write((uint8_t)1);
      lcd.setCursor(6,1);
      if(settingsloop!=7){
        lcd.print(value);
      } else {
        if (value==0) {
          lcd.print("EXIT");
        } else {
          if(value==1) {
            lcd.print("SAVE");
          } else {
            value=0;
          }
        }
      }
      lcd.setCursor(12,1);
      lcd.write((uint8_t)3);
      getInput();
      switch (inputval){
        case (1):  //up/right
          value+=valueIncrement;
          inputval=0;
          break;
        case (2):  //down/left
          value-=valueIncrement;
          inputval=0;
          break;
        case (3):  //enter/select
          switch (settingsloop){
            case (0):  // Kp
              Kp=value;
              break;
            case (1):  // Ki
              Ki=value;
              break;
            case (2):  // Kd
              Kd=value;
              break;
            case (3):  // Window Size
              WindowSize=(uint16_t)value;
              break;
            case (4):  // Sample Interval
              sampleTime=(int)value;
              break;
            case (5):  // I Clamp lower limit
              KiClampLow=value;
              break;
            case (6):  // I Clmap higher limit
              KiClampHigh=value;
              break;
            case (7):  // SAVE/EXIT
              if(value==1){
                savePIDparameters();
              }
              
            default:  // THIS SHOULDN'T HAPPEN!
              break;
          }
          settingsexit=1;
          inputval=0;
          //Serial.write("  SEL  ");
          break;
        default:
          break;  
      }
    }
    settingsexit=0;
  }
}  


void loadPIDparameters()
{
  EEPROM_readAnything(487, Kp);
  EEPROM_readAnything(491, Ki);
  EEPROM_readAnything(495, Kd);
  EEPROM_readAnything(499, WindowSize);
  EEPROM_readAnything(501, sampleTime);
  EEPROM_readAnything(503, KiClampLow);
  EEPROM_readAnything(507, KiClampHigh);
  /*
  Serial.print("LOADING FROM EEPROM\n");
  Serial.print(Kp);
  Serial.print("\n");
  Serial.print(Ki);
  Serial.print("\n");
  Serial.print(Kd);
  Serial.print("\n");
  Serial.print(WindowSize);
  Serial.print("\n");
  Serial.print(sampleTime);
  Serial.print("\n");
  Serial.print(KiClampLow);
  Serial.print("\n");
  Serial.print(KiClampHigh);
  Serial.print("\n");
  */
}

void savePIDparameters()
{
  EEPROM_writeAnything(487, Kp);
  EEPROM_writeAnything(491, Ki);
  EEPROM_writeAnything(495, Kd);
  EEPROM_writeAnything(499, WindowSize);
  EEPROM_writeAnything(501, sampleTime);
  EEPROM_writeAnything(503, KiClampLow);
  EEPROM_writeAnything(507, KiClampHigh);
  /*
  Serial.print("SAVING TO EEPROM\n");
  Serial.print(Kp);
  Serial.print("\n");
  Serial.print(Ki);
  Serial.print("\n");
  Serial.print(Kd);
  Serial.print("\n");
  Serial.print(WindowSize);
  Serial.print("\n");
  Serial.print(sampleTime);
  Serial.print("\n");
  Serial.print(KiClampLow);
  Serial.print("\n");
  Serial.print(KiClampHigh);
  Serial.print("\n");
  */
}


// Sensor wiring:
// Tip: 5V - Red
// Ring: GND - Black
// Sleeve: Data - Green
//
// Socket view, SVHS Mini DIN
//        --
//  D           5V
//   GND      NC
//
// DATA - GREEN
// 5V - WHITE
// GND - SHIELD

// PINS
// 0, Serial Port
// 1, Serial Port
// 2, LCD
// 3, 
// 4, RELAYPIN  // HERMS heater relay switch
// 5, 
// 6, OneWire Bus // DS18S20's
// 7, LCD
// 8, LCD
// 9, LCD
//10, LCD
//11, 
//12, LCD
//13, 
//14, BUTTON             //A0
//15, BUTTON             //A1
//16, BUTTON             //A2
//17,              //A3
//18,              //A4
//19,              //A5

// EEPROM
// EEPROM.write(int address, byte value); // address: 0-511 , value: 0-255
//

