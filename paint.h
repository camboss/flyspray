#ifndef paint_h
#define paint_h

#include "Arduino.h"
#include <Servo.h>

class autospray{
    uint8_t pos_on;
    uint8_t pos_off;
    uint8_t pos_parked;
    uint32_t duration;
    uint32_t timer;
    uint8_t step;
    bool armed;
    bool trigger;
    Servo myservo;

  public:
    autospray(){
      step = 0;
      pos_on = 130;
      pos_off = 110;
      pos_parked = 80;
      duration = 500;
    }
    
    void set_pos_on(uint8_t new_pos);
    uint8_t get_pos_on();
    
    void set_pos_off(uint8_t new_pos);
    uint8_t get_pos_off();
  
    void set_pos_parked(uint8_t new_pos);
    uint8_t get_pos_parked();    
    
    void set_duration(uint32_t new_duration);
    uint32_t get_duration();
  
    void set_armed(bool state);
    bool get_armed();
   
    void set_trigger();
    bool get_trigger();  
    
    void startup();
    
    void main();
};

extern autospray autosprayer;

#endif
