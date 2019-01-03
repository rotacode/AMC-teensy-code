//Receiver Code
//code for teensy 3.6 slave
#include <ResponsiveAnalogRead.h> //Library for fader touch sensitivity
#include <PID_v1.h>
int INT2HEX[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
int inByte1 = 0;  
int inByte2 = 0; 
int rec = 0;

// FADER PINS AND SETUP
//Arduino Pin Assignments
const int motorUp[8]    = {9,29,5,7,4,2,35,37};   //H-Bridge control to make the motor go down
const int motorDown[8]  = {10,30,6,8,3,14,36,38};   //H-Bridge control to make the motor go up

//Inputs
const int wiper[8]     = {32,31,39,20,21,15,A11,A10};   //Position of fader relative to GND (Analog 0)
const int touchSend[8]    = {1,0,16,17,18,19,22,23};   //Send pin for Capacitance Sensing Circuit (Digital 7)
const int touchReceive[8] = {1,0,16,17,18,19,22,23};   //Receive pin for Capacitance Sensing Circuit (Digital 8)

//Variables
double   faderMax[8]     = {0,0,0,0,0,0,0,0};     //Value read by fader's maximum position (0-1023)
double   faderMin[8]        = {0,0,0,0,0,0,0,0};  //Value read by fader's minimum position (0-1023)

int      touchpin[8] = {1,0,16,17,18,19,22,23}; 
int      faderChannel    = 1;                     //Value from 1-8

const int faderTouchThreshold =4850 ;
bool     touched[8]         ={false,false,false,false,false,false,false,false};                 //Is the fader currently being touched?
 
int position[8] = {-99,-99,-99,-99,-99,-99,-99,-99};

ResponsiveAnalogRead touchLine0     = ResponsiveAnalogRead(touchSend[0], touchReceive[0]);
ResponsiveAnalogRead touchLine1     = ResponsiveAnalogRead(touchSend[1], touchReceive[1]);
ResponsiveAnalogRead touchLine2     = ResponsiveAnalogRead(touchSend[2], touchReceive[2]);
ResponsiveAnalogRead touchLine3     = ResponsiveAnalogRead(touchSend[3], touchReceive[3]);
ResponsiveAnalogRead touchLine4     = ResponsiveAnalogRead(touchSend[4], touchReceive[4]);
ResponsiveAnalogRead touchLine5     = ResponsiveAnalogRead(touchSend[5], touchReceive[5]);
ResponsiveAnalogRead touchLine6     = ResponsiveAnalogRead(touchSend[6], touchReceive[6]);
ResponsiveAnalogRead touchLine7     = ResponsiveAnalogRead(touchSend[7], touchReceive[7]);
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//PID LINKS AND TUNING
//Specify the links and initial tuning parameters
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1

int SampleTime = 1000; //1 sec
int controllerDirection = DIRECT;
double ITerm, lastInput;
double outMin, outMax;
bool inAuto = false;

double Kp=0, Ki=0, Kd=0;
double Setpoint, Input, Output;
double errSum, lastErr;
double kp, ki, kd;
unsigned long lastTime;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void setup() {
   //PID SETUP
// initialize the variables we're linked to
  Input = analogRead(31);
  Setpoint = inByte2*8.06;
 // turn the PID on
  myPID.SetMode(AUTOMATIC);

//ANALOG WIRTE AND AREF SETUP
  analogReference(EXTERNAL); 
  analogWriteResolution(12); 
  analogWriteFrequency(motorUp[1],14648.437);  // 14648.437 ideal frequency for 180MHz 12bit PWM
  analogWriteFrequency(motorDown[1],14648.437);  //14648.437 ideal frequency for 180MHz 12bit PWM   analogWriteFrequency(motorUp[5],14648.437);  // 14648.437 ideal frequency for 180MHz 12bit PWM
  analogWriteFrequency(motorUp[5],14648.437);  // 14648.437 ideal frequency for 180MHz 12bit PWM
  analogWriteFrequency(motorDown[5],14648.437);  //14648.437 ideal frequency for 180MHz 12bit PWM   analogWriteFrequency(motorUp[5],14648.437);  // 14648.437 ideal frequency for 180MHz 12bit PWM
  analogWriteFrequency(motorUp[6],14648.437);  // 14648.437 ideal frequency for 180MHz 12bit PWM
  analogWriteFrequency(motorDown[6],14648.437);  //14648.437 ideal frequency for 180MHz 12bit PWM   analogWriteFrequency(motorUp[5],14648.437);  // 14648.437 ideal frequency for 180MHz 12bit PWM

  calibrateFader();

  // start serial port at 31250 bps
  Serial5.begin(31250);
  // wait for a while till the serial port is ready
  delay(100);

}

void loop()
{
   //PID COMPUTE
  Input = analogRead(31);
  myPID.Compute();
  analogWrite(motorUp[1], Output);
  analogWrite(motorDown[1], Output);

  if(Serial5.available()>2)
  {
    // get incoming byte:   
     
     inByte1 = Serial5.read(); 
     inByte2 = Serial5.read();        
     if(inByte1==33){
       updateFader1(inByte2*8.06);
       Serial.println(inByte2); 
     }
     if(inByte1==37){
       updateFader5(inByte2*8.06);
       Serial.println(inByte2); 
     }  
     if(inByte1==38){
       updateFader6(inByte2*8.06);
       Serial.println(inByte2); 
     }  
  }
}
void calibrateFader() { 
 
  //CALIBRATE ALL FADERS AT START
  analogWrite(9, 3200);
  analogWrite(29, 3200);
  analogWrite(5, 3200);
  analogWrite(7, 3200);
  analogWrite(4, 3200);
  analogWrite(2, 3850);
  analogWrite(35, 3200);
  analogWrite(37, 3200);
  delay(150);
  analogWrite(9, 0);
  analogWrite(29, 0);
  analogWrite(5, 0);
  analogWrite(7, 0);
  analogWrite(4, 0);
  analogWrite(2, 0);
  analogWrite(35, 0);
  analogWrite(37, 0);
  
  //Send fader to the top and read max position
  analogWrite(10, 3200);
  analogWrite(30, 3200);
  analogWrite(6, 3200);
  analogWrite(8, 3200);
  analogWrite(3, 3200);
  analogWrite(14, 3850);
  analogWrite(36, 3200);
  analogWrite(38, 3200);
  delay(320);
  analogWrite(10, 0);
  analogWrite(30, 0);
  analogWrite(6, 0);
  analogWrite(8, 0);
  analogWrite(3, 0);
  analogWrite(14, 0);
  analogWrite(36, 0);
  analogWrite(38, 0);
  
  //FADERS MAX READ
  faderMax[0] = analogRead(wiper[0]) - 5;
  faderMax[1] = analogRead(wiper[1]) - 5;
  faderMax[2] = analogRead(wiper[2]) - 5;
  faderMax[3] = analogRead(wiper[3]) - 5;
  faderMax[4] = analogRead(wiper[4]) - 5;
  faderMax[5] = analogRead(wiper[5]) - 5;
  faderMax[6] = analogRead(wiper[6]) - 5;
  faderMax[7] = analogRead(wiper[7]) - 5;

  //Send fader to the bottom and read min position
  analogWrite(9, 3200);
  analogWrite(29, 3200);
  analogWrite(5, 3200);
  analogWrite(7, 3200);
  analogWrite(4, 3200);
  analogWrite(2, 3850);
  analogWrite(35, 3200);
  analogWrite(37, 3200);
  delay(300);
  analogWrite(9, 0);
  analogWrite(29, 0);
  analogWrite(5, 0);
  analogWrite(7, 0);
  analogWrite(4, 0);
  analogWrite(2, 0);
  analogWrite(35, 0);
  analogWrite(37, 0);

  //FADERS MIN READ
  faderMin[0] = analogRead(wiper[0]) + 5;
  faderMin[1] = analogRead(wiper[1]) + 5;
  faderMin[2] = analogRead(wiper[2]) + 5;
  faderMin[3] = analogRead(wiper[3]) + 5;
  faderMin[4] = analogRead(wiper[4]) + 5;
  faderMin[5] = analogRead(wiper[5]) + 5;
  faderMin[6] = analogRead(wiper[6]) + 5;
  faderMin[7] = analogRead(wiper[7]) + 5;
for(int i=0; i< 8; i++){
    Serial.print("faderMax[");
    Serial.print(i);
    Serial.print("] : ");
    Serial.println(faderMax[i]);
 
  }
for(int i=0; i< 8; i++){
    Serial.print("faderMin[");
    Serial.print(i);
    Serial.print("] : ");
    Serial.println(faderMin[i]);
}
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//tell us where the fader is
float faderPosition(int index) {

    position[index] = analogRead(wiper[index]);   
    
    if (position[index] <= faderMin[index]) {
       return 0.0;
    }
     else if (position[index] >= faderMax[index]) {
       return 16383.0;
    }
    else {
       return ((float)(position[index] - faderMin[index]) / (faderMax[index] - faderMin[index])) * 16383;
    }
}
void checkTouch() {
    // SEND FADERS TOUCH AND RELEASE MIDI MASSAGES
    //For the capSense comparison below,
    //700 is arbitrary and may need to be changed
    //depending on the fader cap used (if any).
    for(int i=0; i< 8; i++){
       if (!touched[i] && touchRead(touchpin[i]) > faderTouchThreshold) {
          touched[i] = true;

          //Send MIDI Touch On Message         
          byte fadertouch[6] = { 0xB0,0x0F,INT2HEX[i], 0xB0,0x2F,0x40}; // ACK msg - should be safe for any device even if listening for 7D
          Serial5.write(fadertouch[i]);

        }
        else {
        touched[i] = false;
       // Serial.println(touchRead(touchpin[i]));
        byte faderrelease[6] = { 0xB0,0x0F,INT2HEX[i], 0xB0,0x2F,0x00}; // ACK msg - should be safe for any device even if listening for 7D
        Serial5.write(faderrelease[i]);
        }
    
        
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//UPDATE FADERS MOTORS//
void updateFader0(int position) {
  if (position < analogRead(wiper[0]) - 5 && position > faderMax[0] && !touched[0]) {
    analogWrite(motorDown[0], 3100);
    while (position < analogRead(wiper[0]) - 5 && !touched) {};  //Loops until motor is done moving
    analogWrite(motorDown[0], 0);
     
  }else if 
  (position > analogRead(wiper[0]) + 5 && position < faderMin[0] && !touched[0]) {
  
    analogWrite(motorUp[0], 3100 );
    while (position > analogRead(wiper[0]) + 5 && !touched[0]) {}; //Loops until motor is done moving
    analogWrite(motorUp[0], 0);  
  }
}
void updateFader1(int position) {
     if (position < analogRead(wiper[1]) - 50 && position > faderMax[1] && !touched[1]) {
    analogWrite(motorDown[1], 3100);
    while (position < analogRead(wiper[1]) - 50 && !touched) {};  //Loops until motor is done moving
    analogWrite(motorDown[1], 0);
     
  }else if 
  (position > analogRead(wiper[1]) + 50 && position < faderMin[1] && !touched[1]) {
  
    analogWrite(motorUp[1], 3100 );
    while (position > analogRead(wiper[1]) + 50 && !touched[1]) {}; //Loops until motor is done moving
    analogWrite(motorUp[1], 0);
  
  }
    if (position < analogRead(wiper[1]) - 8 && position > faderMax[1] && !touched[1]) {
    analogWrite(motorDown[1], 2950);
    while (position < analogRead(wiper[1]) - 8 && !touched[1]) {};  //Loops until motor is done moving
    analogWrite(motorDown[1], 0);
     
  }else if 
  (position > analogRead(wiper[1]) + 8 && position < faderMin[1] && !touched[1]) {
  
    analogWrite(motorUp[1], 2950 );
    while (position > analogRead(wiper[1]) + 8 && !touched[1]) {}; //Loops until motor is done moving
    analogWrite(motorUp[1], 0);
  
  }
    if (position < analogRead(wiper[1]) - 1 && position > faderMax[1] && !touched[1]) {
    analogWrite(motorDown[1], 2700);
    while (position < analogRead(wiper[1]) - 1 && !touched[1]) {};  //Loops until motor is done moving
    analogWrite(motorDown[1], 0);
     
  }else if 
  (position > analogRead(wiper[1]) + 1 && position < faderMin[1] && !touched[1]) {
  
    analogWrite(motorUp[1], 2700 );
    while (position > analogRead(wiper[1]) + 1 && !touched[1]) {}; //Loops until motor is done moving
    analogWrite(motorUp[1], 0);  
  }

}
void updateFader2(int position) {
  
  if (position < analogRead(wiper[2]) - 5 && position > faderMax[2] && !touched[2]) {
    analogWrite(motorDown[2], 3100);
    while (position < analogRead(wiper[2]) - 5 && !touched[2]) {};  //Loops until motor is done moving
    analogWrite(motorDown[2], 0);
     
  }else if 
  (position > analogRead(wiper[2]) + 5 && position < faderMin[2] && !touched[2]) {
  
    analogWrite(motorUp[2], 3100 );
    while (position > analogRead(wiper[2]) + 5 && !touched[2]) {}; //Loops until motor is done moving
    analogWrite(motorUp[2], 0);  
  }
}
void updateFader3(int position) {
  
  if (position < analogRead(wiper[3]) - 5 && position > faderMax[3] && !touched[3]) {
    analogWrite(motorDown[3], 3100);
    while (position < analogRead(wiper[3]) - 5 && !touched[3]) {};  //Loops until motor is done moving
    analogWrite(motorDown[3], 0);
     
  }else if 
  (position > analogRead(wiper[3]) + 5 && position < faderMin[3] && !touched[3]) {
  
    analogWrite(motorUp[3], 3100 );
    while (position > analogRead(wiper[3]) + 5 && !touched[3]) {}; //Loops until motor is done moving
    analogWrite(motorUp[3], 0);  
  }
}
void updateFader4(int position) {
  
  if (position < analogRead(wiper[4]) - 5 && position > faderMax[4] && !touched[4]) {
    analogWrite(motorDown[4], 3100);
    while (position < analogRead(wiper[1]) - 5 && !touched[4]) {};  //Loops until motor is done moving
    analogWrite(motorDown[4], 0);
     
  }else if 
  (position > analogRead(wiper[4]) + 5 && position < faderMin[4] && !touched[4]) {
  
    analogWrite(motorUp[4], 3100 );
    while (position > analogRead(wiper[4]) + 5 && !touched[4]) {}; //Loops until motor is done moving
    analogWrite(motorUp[4], 0);  
  }
}
void updateFader5(int position) {
  
  if (position < analogRead(wiper[5]) - 5 && position > faderMax[5] && !touched[5]) {
    analogWrite(motorDown[5], 3100);
    while (position < analogRead(wiper[5]) - 5 && !touched[5]) {};  //Loops until motor is done moving
    analogWrite(motorDown[5], 0);
     
  }else if 
  (position > analogRead(wiper[5]) + 5 && position < faderMin[5] && !touched[5]) {
  
    analogWrite(motorUp[5], 3100);
    while (position > analogRead(wiper[5]) + 5 && !touched[5]) {}; //Loops until motor is done moving
    analogWrite(motorUp[5], 0);  
  }
}
void updateFader6(int position) {
  
  if (position < analogRead(wiper[6]) - 5 && position > faderMax[6] && !touched[6]) {
    analogWrite(motorDown[6], 3100);
    while (position < analogRead(wiper[6]) - 5 && !touched[6]) {};  //Loops until motor is done moving
    analogWrite(motorDown[6], 0);
     
  }else if 
  (position > analogRead(wiper[6]) + 5 && position < faderMin[6] && !touched[6]) {
  
    analogWrite(motorUp[6], 3100 );
    while (position > analogRead(wiper[6]) + 5 && !touched[6]) {}; //Loops until motor is done moving
    analogWrite(motorUp[6], 0);   
  }
}
void updateFader7(int position) {
  
  if (position < analogRead(wiper[7]) - 5 && position > faderMax[7] && !touched[7]) {
    analogWrite(motorDown[7], 3100);
    while (position < analogRead(wiper[7]) - 5 && !touched[7]) {};  //Loops until motor is done moving
    analogWrite(motorDown[7], 0);
     
  }else if 
  (position > analogRead(wiper[7]) + 5 && position < faderMin[7] && !touched[7]) {
  
    analogWrite(motorUp[7], 3100 );
    while (position > analogRead(wiper[7]) + 5 && !touched) {}; //Loops until motor is done moving
    analogWrite(motorUp[7], 0);  
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//PID COMPUTE ERROR DISTANCE AND SPEED
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
// The selectMuxPin function sets the S0, S1, and S2 pins
// accordingly, given a pin from 0-7.

void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
 
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void serialhandle(){
    if(inByte1==32){
       updateFader0(inByte2*8.06);
       //Serial.println(inByte1);
    }
    if(inByte1==33){
       updateFader1(inByte2*8.06);
       //Serial.println(inByte1);
    }
    if(inByte1==34){
       updateFader2(inByte2*8.06);
       //Serial.println(inByte1);
    }
    if(inByte1==35){
       updateFader3(inByte2*8.06);
       //Serial.println(inByte1);
    }
    if(inByte1==36){
       updateFader4(inByte2*8.06);
       //Serial.println(inByte1);
    }
    if(inByte1==37){
       updateFader5(inByte2*8.06);
       //Serial.println(inByte1);
    }
    if(inByte1==38){
       updateFader6(inByte2*8.06);
       //Serial.println(inByte1);
    }
    if(inByte1==39){
       updateFader7(inByte2*8.06);
       //Serial.println(inByte1);
    }
}
