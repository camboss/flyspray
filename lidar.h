#ifndef lidar_h
#define lidar_h

#include "Arduino.h"
#include <Wire.h>

#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

class lidar{
    TwoWire& wire;
    int LIDARLite_ADDRESS;
    uint32_t ticks;
    uint16_t reading_raw;
    uint16_t reading_output;
    uint8_t step;
    uint32_t offset = 0;
    uint32_t watchdog, watchdog_count, watchdog_prev, error_count;

  public:
    lidar(int a, TwoWire& b):LIDARLite_ADDRESS(a),wire(b){
      step = 1;
      ticks = 0;
      watchdog = 0;
      watchdog_count = 0;
      watchdog_prev = 0;
      error_count = 0;
    }
    static void lidar_startup();
    void lidar_main();
    uint16_t get_reading();
    uint32_t get_errors();
    int get_offset();
    void set_offset(int offset_in);
    //buffer av_buffer;
};

extern lidar lidar_left;
extern lidar lidar_right;

#endif
