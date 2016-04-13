#ifndef pid_struct_h
#define pid_struct_h

#include "Arduino.h"
#include <PID_v1.h>

class pid_struct{
  
  private:
    double setpoint;
    
  public:
    PID pid;
    double input, output;
    pid_struct(double Kp, double Ki, double Kd, int dir):pid(&input, &output, &setpoint,Kp,Ki,Kd, dir){};
    void set_setpoint(double value);
    double get_setpoint();
  
};

extern pid_struct PitchPID;
extern pid_struct YawPID;

#endif
