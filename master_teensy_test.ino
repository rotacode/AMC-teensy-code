//code for teensy 3.6 master
#include <MIDIUSB.h>
#define ControlChange  0xB0


//INT GAINS SCREENS AND FADERS 

//ScreansGain int
int hi[8] = {0,0,0,0,0,0,0,0};
int lo[8] = {0,0,0,0,0,0,0,0};
//ScreansGain To DB
int Gain[9] = {0,0,0,0,0,0,0,0,0,};


void setup() {
  Serial5.begin(31250);
  usbMIDI.setHandleControlChange(myControlChange); 

}

void loop(){
  usbMIDI.read();
  
  if(usbMIDI.getData1() == 33){
     Serial5.write(usbMIDI.getData1()); // send a byte with the value 45
     delay(1);
     Serial5.write(Gain[1]);
     delay(1);
     //delayMicroseconds(100);

     //Serial5.flush();
  }
  if(usbMIDI.getData1() == 37){
     Serial5.write(usbMIDI.getData1()); // send a byte with the value 45
     delay(1);
     Serial5.write(Gain[5]);
     delay(1);
     //delayMicroseconds(100);

     //Serial5.flush();
  }  
  if(usbMIDI.getData1() == 38){
     Serial5.write(usbMIDI.getData1()); // send a byte with the value 45
     delay(1);
     Serial5.write(Gain[6]);
     delay(1);
     //delayMicroseconds(100);

     //Serial5.flush();
  }  
}

void myControlChange(byte channel, byte control, byte value){                // blink the LED a number of times
  
    //INT FADERS GAINS 
    if(control == 32){
      hi[1] = value;       
      }
    if(control == 0){
      lo[1] = value;
      Gain[0] = (hi[0]>>7) + lo[0];
    }
    
    if(control == 33){
      hi[1] = value;       
      }
    if(control == 1){
      lo[1] = value;
      Gain[1] = (hi[1]>>7) + lo[1];
    }
    
    if(control == 34){
      hi[2] = value;       
      }
    if(control == 2){
      lo[2] = value;
      Gain[2] = (hi[2]>>7) + lo[2];
    }   
    
    if(control == 35){
      hi[3] = value;       
      }
    if(control == 3){
      lo[3] = value;
      Gain[3] = (hi[3]>>7) + lo[3];
    }      
  
    if(control == 36){
      hi[4] = value;       
      }
    if(control == 4){
      lo[4] = value;
      Gain[4] = (hi[4]>>7) + lo[4];
    } 
   
    if(control == 37){
      hi[5] = value;       
      }
    if(control == 5){
      lo[5] = value;
      Gain[5] = (hi[5]>>7) + lo[5];
    }
    
    if(control == 38){
      hi[6] = value;       
      }
    if(control == 6){
      lo[6] = value;
      Gain[6] = (hi[6]>>7) + lo[6];
    }
    
    if(control == 39){ 
      hi[7] = value;       
      }
    if(control == 7){
      lo[7] = value;
      Gain[7] = (hi[7]>>7) + lo[7];
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

