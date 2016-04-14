#ifndef adxl345_h
#define adxl345_h

#include "Arduino.h"
#include <Wire.h>
#include "Math.h"

#define    ADXL345_ADDRESS  0x53        // I2C Address of adxl345
#define    POWER_CTL_REG    0x2D        // Power CTL Register
#define    BW_RATE_REG      0x2C
#define    DATA_FORMAT_REG  0x31
#define    INT_ENABLE_REG   0x2E
#define    X_OFFSET_REG     0x1E
#define    Y_OFFSET_REG     0x1F
#define    Z_OFFSET_REG     0x20
#define    X_REG            0x32
#define    Y_REG            0x34
#define    Z_REG            0x36
#define    INT1             51

class adxl345{
  
  private:
    int16_t x, y, z;
    void read_accel(int16_t &x, int16_t &y, int16_t &z);
    bool read_complete;
    double pitch;

  public:
    void init();
    void main();
    double get_pitch();
    double calc_pitch();
};

struct axis{
  int16_t value;
  int32_t sum = 0;
  int16_t average = 0;
  int8_t offset = 0;    
};

extern adxl345 accel;

#endif
