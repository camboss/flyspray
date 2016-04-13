#include "lidar.h"

lidar lidar_left(0x62,Wire);
lidar lidar_right(0x62,Wire1);

uint16_t lidar::get_reading(){
  return reading_output;  
}

uint32_t lidar::get_errors(){
  return error_count;  
}

void lidar::set_offset(int offset_in){
  offset = offset_in;
}

int lidar::get_offset(){
  return offset;
}

void lidar::lidar_main() {
  
  if(watchdog != watchdog_prev){
    watchdog_count = 0;
    watchdog_prev = watchdog;  
  }
  else{
    watchdog_count++;  
  }
  if(watchdog_count > 10000){
    error_count++;
    reading_output = 0;
    watchdog_count = 0;
    ticks = 0;
    step = 1;
  }
  
  switch(step){
    case 1:
      if(millis() >= ticks){
        //Request the Lidar to take a new reading
        wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)
        wire.write((int)MeasureValue);
        if(wire.endTransmission() == 0){
          watchdog++;
          step = 2;
        }
        ticks = millis() + 20;
      }
      break;
    case 2:
      if(millis() >= ticks){
        //Set the register pointer to the measurement result
        wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
        wire.write((int)RegisterHighLowB); // sets register pointer to  (0x04)
        if(wire.endTransmission() == 0){
          watchdog++;
          step = 3;
        }
      }
      break;
    case 3:
      //Request 2 bytes from the previously set register
      wire.requestFrom((int)LIDARLite_ADDRESS, 2);
      watchdog++;
      step = 4;
      break;
    case 4:
      //Read the measurement result
      if (wire.available() >= 2){
        reading_raw = wire.read(); // receive high byte (overwrites previous reading)
        reading_raw = reading_raw << 8; // shift high byte to be high 8 bits
        reading_raw |= wire.read(); // receive low byte as lower 8 bits
        reading_output = reading_raw + offset;
        //av_buffer.push(reading_output);
        watchdog++;
        step = 1;
        ticks = millis();
      }
      break;    
  }
}
