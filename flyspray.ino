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
#include "flyspray.h"

bool oneshot1000ms, oneshot20ms = false;
uint32_t count20ms, count1000ms, count = 0;
int pos = 0;
bool toggle = false;
int YawOutput_Adjusted;
int PitchOutput_Adjusted;
boolean stringComplete = false;  // whether the string is complete
boolean latch = false;

void setup() {

  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();
  pinMode(13, OUTPUT);
  
  delay(3000);

  Serial.println("Starting accelerometer initialisation");
  accel.init();
  Serial.println("Accelerometer initialisation complete");
  
  Serial.println("Starting lidar initialisation");
  lidar_left.init();
  lidar_right.init();
  Serial.println("Lidar initialisation complete");

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
  
  count++;
  
  if((millis()%20<10)&!oneshot20ms){
    
    oneshot20ms = true;
    count20ms++;
    
    //Get a new pitch value from the accelerometer
    accel.calc_pitch();
    
    //Get a new distance value from each lidar
    lidar_left.read_lidar();
    lidar_right.read_lidar();
    
    //Set a new input for the yaw PID and execute once
    YawPID.input = calculatedDistance(lidar_left.get_reading(),accel.get_pitch()) - calculatedDistance(lidar_right.get_reading(),accel.get_pitch());
    YawPID.pid.Compute();
  
    //Set a new input for the pitch PID and execute once
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
  else if(millis()%20>=10){
    
    oneshot20ms = false;
    
  }
  
  if((millis()%1000<500)&!oneshot1000ms){

    oneshot1000ms = true;
    Serial.print("Main loop executing at ");
    Serial.print(count);
    Serial.println("Hz");
    Serial.print("Control loop executing at ");
    Serial.print(count20ms);
    Serial.println("Hz");
    Serial.print("Pitch: ");
    Serial.println(accel.get_pitch());    
    Serial.print("Right lidar raw distance: ");
    Serial.println(lidar_right.get_reading());
    Serial.print("Right lidar corrected distance: ");
    Serial.println(calculatedDistance(lidar_right.get_reading(),accel.get_pitch()));
    Serial.print("Left lidar raw distance: ");
    Serial.println(lidar_left.get_reading());
    Serial.print("Left lidar corrected distance: ");
    Serial.println(calculatedDistance(lidar_left.get_reading(),accel.get_pitch()));
    Serial.println("");
    count20ms = 0;
    count = 0;
    
  }
  else if(millis()%1000>=500){
    oneshot1000ms = false;  
  }

  //Process any input requests received via the radio
  input::inputHandler();
  
  //Update the position of the paint servo
  autosprayer.main();
  
  //Execute the main loop of the SBUS code. This is all clocked from the incoming serial stream so this loop should be executed as often as possible
  sbusRx.mainloop();

}

uint16_t calculatedDistance(uint16_t raw, double angle){
  
  return round(cos(angle)*raw);
  
}
