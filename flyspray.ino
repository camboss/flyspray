#include <Servo.h>
#include <PID_v1.h>
#include <Streaming.h>
#include "sbus2.h"
#include "lidar.h"
#include <Wire.h>
#include "input.h"
#include "pid_struct.h"
#include "paint.h"
#include "adxl345.h"

uint16_t calculatedDistance(uint16_t raw, double angle);

bool print_status = false;
int pos = 0;
bool toggle = false;
int YawOutput_Adjusted;
int PitchOutput_Adjusted;
boolean stringComplete = false;  // whether the string is complete
boolean latch = false;

int16_t xval, yval, zval;

void setup() {

  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();
  pinMode(13, OUTPUT);
  
  delay(5000);

  Serial.println("Starting accelerometer initialisation");
  accel.init();
  Serial.println("Accelerometer initialisation complete");

  Serial3.begin(57600);

  Serial1.begin(100000, SERIAL_8E2); //Receive SBUS data
  Serial2.begin(100000, SERIAL_8E2); //Transmit SBUS data

  sbusRx.begin();

  lidar_left.set_offset(0);
  lidar_right.set_offset(0);

  YawPID.set_setpoint(0);
  PitchPID.set_setpoint(50);

  YawPID.pid.SetSampleTime(100);
  PitchPID.pid.SetSampleTime(100);

  //turn the PID on
  YawPID.pid.SetOutputLimits(-500, 500);
  YawPID.pid.SetMode(MANUAL);

  PitchPID.pid.SetOutputLimits(-500, 500);
  PitchPID.pid.SetMode(MANUAL);

  autosprayer.startup();

  delay(1000);

}

void loop() {
  
  /*if((millis()%1000==0)&!print_status){

    print_status = true;
    accel.readAll(xval, yval, zval);
    Serial.print("X Val: ");
    Serial.println(xval);
    Serial.print("Y Val: ");
    Serial.println(yval);
    Serial.print("Z Val: ");
    Serial.println(zval);
    Serial.print("Calculated pitch: ");
    Serial.println(accel.calculatedPitch());
    Serial.print("Raw distance: ");
    Serial.println(lidar_left.get_reading());
    Serial.print("Corrected distance: ");
    Serial.println(calculatedDistance());
    Serial.println("");
    
  }
  else{
    print_status = false;  
  }*/

  input::inputHandler();
  
  autosprayer.main();

  sbusRx.mainloop();

  lidar_left.lidar_main();
  lidar_right.lidar_main();
  
  accel.main();

  YawPID.input = calculatedDistance(lidar_left.get_reading(),accel.get_pitch()) - calculatedDistance(lidar_right.get_reading(),accel.get_pitch());
  YawPID.pid.Compute();

  PitchPID.input = (calculatedDistance(lidar_left.get_reading(),accel.get_pitch()) + calculatedDistance(lidar_right.get_reading(),accel.get_pitch())) / 2;
  PitchPID.pid.Compute();

  //If the yaw PID is in manual mode, set the output to 0 so it starts from a known point next time it is set to automatic mode
  if (YawPID.pid.GetMode() == MANUAL) {
    YawPID.output = 0;
  }

  //If the pitch PID is in manual mode, set the output to 0 so it starts from a known point next time it is set to automatic mode
  if (PitchPID.pid.GetMode() == MANUAL) {
    PitchPID.output = 0;
  }

  //Add 1020 to the yaw and pitch PID outputs, which is the equivalent of "sticks centered" on the transmitter
  YawOutput_Adjusted = (int)YawPID.output + 1020;
  PitchOutput_Adjusted = (int)PitchPID.output + 1020;

  if (YawPID.pid.GetMode() == AUTOMATIC) {
    sbusRx.setChannel(4, YawOutput_Adjusted);
  }
  else {
    sbusRx.setChannel(4, 0);
  }

  if (PitchPID.pid.GetMode() == AUTOMATIC) {
    sbusRx.setChannel(2, PitchOutput_Adjusted);
  }
  else {
    sbusRx.setChannel(2, 0);
  }

}

uint16_t calculatedDistance(uint16_t raw, double angle){
  
  return round(cos(angle)*raw);
  
}
