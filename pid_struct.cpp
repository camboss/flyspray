#include "pid_struct.h"

pid_struct PitchPID (10.0,0.0,0.0,DIRECT);
pid_struct YawPID (15.0,0.0,0.0,REVERSE);

void pid_struct::set_setpoint(double value){
  setpoint = value;  
}

double pid_struct::get_setpoint(){
  return setpoint;  
}
