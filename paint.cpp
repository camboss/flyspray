#include "paint.h"
#include "sbus2.h"

autospray autosprayer;

void autospray::set_pos_on(uint8_t new_pos){
  pos_on = new_pos;  
}

uint8_t autospray::get_pos_on(){
  return pos_on;  
}
    
void autospray::set_pos_off(uint8_t new_pos){
  pos_off = new_pos;  
}
    
uint8_t autospray::get_pos_off(){
  return pos_off;
}

void autospray::set_pos_parked(uint8_t new_pos){
  pos_parked = new_pos;  
}
    
uint8_t autospray::get_pos_parked(){
  return pos_parked;
}  
    
void autospray::set_duration(uint32_t new_duration){
  duration = new_duration;  
}
    
uint32_t autospray::get_duration(){
  return duration;
}  
    
void autospray::startup(){
  myservo.attach(9);  
}

void autospray::set_armed(bool state){
  armed = state;  
}

bool autospray::get_armed(){
  return armed;  
}

void autospray::set_trigger(){
  trigger = true;  
}

bool autospray::get_trigger(){
  return trigger;  
}
    
void autospray::main(){

  if(!armed){
    trigger = false;
    myservo.write(pos_parked);
    step = 0;    
  }
  
  switch(step){
    
    case 0:
      if(armed){
        myservo.write(pos_off);
        step = 1;  
      }
      break;
      
    case 1:
      if(trigger){
        trigger = false;
        myservo.write(pos_on);
        timer = millis();
        step = 2;  
      }
      break;
      
    case 2:
      if(millis() >= (timer+duration)){
        myservo.write(pos_off);
        step = 0;  
      }
      break;
  
  }
  
}
