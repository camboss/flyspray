#include "input.h"
#include "lidar.h"
#include "pid_struct.h"
#include "sbus2.h"
#include "paint.h"
#include "adxl345.h"
#include "flyspray.h"

uint16_t calculatedDistance(uint16_t raw, double angle);

String input::inputString = "";
int input::state = 0;
String input::tokens[6] = {};

void input::read_parameter(const String& parameter){
  
  int value;
  bool found = true;
  
  if (parameter == "LLO") {
    Serial3.println(lidar_left.get_offset());
  }
  
  else if (parameter == "LLV") {
    Serial3.println(lidar_left.get_reading());
  }
  
  else if (parameter == "LCV") {
    Serial3.println(calculatedDistance(lidar_left.get_reading(),accel.get_pitch()));
  }
  
  else if (parameter == "RLO") {
    Serial3.println(lidar_right.get_offset());
  }
  
  else if (parameter == "RLV") {
    Serial3.println(lidar_right.get_reading());  
  }
  
  else if (parameter == "RCV") {
    Serial3.println(calculatedDistance(lidar_right.get_reading(),accel.get_pitch()));
  }
  
  else if (parameter == "PITCH") {
    Serial3.println(accel.get_pitch());
  }

  else if (parameter == "YAW_MODE") {
    if (YawPID.pid.GetMode() == AUTOMATIC){
      Serial3.println("AUTOMATIC");
    }
    else if(YawPID.pid.GetMode() == MANUAL){
      Serial3.println("MANUAL");
    }
    else{
      Serial3.println("Unknown mode returned"); 
    }  
  }
  
  else if (parameter == "YAW_DIR") {
    if (YawPID.pid.GetDirection() == DIRECT){
      Serial3.println("DIRECT");
    }
    else if(YawPID.pid.GetDirection() == REVERSE){
      Serial3.println("REVERSE");
    }
    else{
      Serial3.println("UNKNOWN"); 
    }  
  }
  
  else if (parameter == "YAW_TUNE") {
    Serial3.print(YawPID.pid.GetKp());
    Serial3.print(",");
    Serial3.print(YawPID.pid.GetKi());
    Serial3.print(",");
    Serial3.print(YawPID.pid.GetKd());
    Serial3.println();
  }
  
  else if (parameter == "YAW_SP") {
    Serial3.println(YawPID.get_setpoint());  
  }
  
  else if (parameter == "YAW_OUTPUT") {
    Serial3.println(YawPID.output); 
  }
  
  else if (parameter == "YAW_ERROR") {
    Serial3.println(YawPID.input);
  }
  
  else if (parameter == "PITCH_MODE") {
    if (PitchPID.pid.GetMode() == AUTOMATIC){
      Serial3.println("AUTOMATIC");
    }
    else if(PitchPID.pid.GetMode() == MANUAL){
      Serial3.println("MANUAL");
    }
    else{
      Serial3.println("Unknown mode returned"); 
    }  
  }
  
  else if (parameter == "PITCH_DIR") {
    if (PitchPID.pid.GetDirection() == DIRECT){
      Serial3.println("DIRECT");
    }
    else if(PitchPID.pid.GetDirection() == REVERSE){
      Serial3.println("REVERSE");
    }
    else{
      Serial3.println("Unknown mode returned"); 
    }  
  }
  
  else if (parameter == "PITCH_SP") {
    Serial3.println(PitchPID.get_setpoint());  
  }
  
  else if (parameter == "PITCH_TUNE") {
    Serial3.print(PitchPID.pid.GetKp());
    Serial3.print(",");
    Serial3.print(PitchPID.pid.GetKi());
    Serial3.print(",");
    Serial3.print(PitchPID.pid.GetKd());
    Serial3.println();
  }
  
  else if (parameter == "PITCH_OUTPUT") {
    Serial3.println(PitchPID.output); 
  }
  
  else if (parameter == "PITCH_ERROR") {
    Serial3.println(PitchPID.input);
  }
  
  else if (parameter == "SPRAY_PARK") {
    Serial3.println(autosprayer.get_pos_parked());
  }
  
  else if (parameter == "SPRAY_OFF") {
    Serial3.println(autosprayer.get_pos_off());
  }
  
  else if (parameter == "SPRAY_ON") {
    Serial3.println(autosprayer.get_pos_on());
  }
  
  else if (parameter == "SPRAY_DUR") {
    Serial3.println(autosprayer.get_duration());
  }
  
  else if (parameter == "CHAN_1") {
    Serial3.println(sbusRx.getChannel(1));  
  }
  
  else if (parameter == "CHAN_2") {
    Serial3.println(sbusRx.getChannel(2));  
  }
  
  else if (parameter == "CHAN_3") {
    Serial3.println(sbusRx.getChannel(3));  
  }
 
  else if (parameter == "CHAN_4") {
    Serial3.println(sbusRx.getChannel(4));  
  }
  
  else if (parameter == "CHAN_5") {
    Serial3.println(sbusRx.getChannel(5));  
  } 
  
  else if (parameter == "CHAN_6") {
    Serial3.println(sbusRx.getChannel(6));  
  }
  
  else if (parameter == "CHAN_7") {
    Serial3.println(sbusRx.getChannel(7));  
  }
  
  else if (parameter == "CHAN_8") {
    Serial3.println(sbusRx.getChannel(8));  
  }
  
  else if (parameter == "BAD_FRAMES") {
    Serial3.println(sbusRx.getBadFrames());  
  }
  
  else if (parameter == "FAILSAFE") {
    Serial3.println(sbusRx.getFailsafeStatus());  
  }
  
  else if (parameter == "SPRAY_ARMED") {
    if (autosprayer.get_armed()){
      Serial3.println("ARMED");
    }
    else{
      Serial3.println("DISARMED"); 
    } 
  }
  
  else{
    found = false;
  }
  
  if(!found){
    Serial3.println("Parameter not found");  
  }
  Serial3.println("");
}

void input::write_parameter(const String& parameter, const String& value1, const String& value2, const String& value3, const String& value4){
  
  bool found = true;
  
  if (parameter == "LLO") {
    lidar_left.set_offset(value1.toInt());
  }

  else if (parameter == "RLO") {
    lidar_right.set_offset(value1.toInt());
  }
  
  else if (parameter == "YAW_MODE") {
    if (value1 == "AUTOMATIC"){
      YawPID.pid.SetMode(AUTOMATIC);
    }
    else if (value1 == "MANUAL"){
      YawPID.pid.SetMode(MANUAL);
    }
    else{
      Serial3.println("Invalid mode setting");  
    }
  }   
  
  else if (parameter == "YAW_SP") {
    YawPID.set_setpoint(value1.toFloat());  
  }
  
  //Set yaw PID values with SetTunings(Kp, Ki, Kd)
  else if (parameter == "YAW_TUNE") {
    YawPID.pid.SetTunings(value1.toFloat(),value2.toFloat(),value3.toFloat()); 
  }
  
  else if (parameter == "PITCH_MODE") {
    if (value1 == "AUTOMATIC"){
      PitchPID.pid.SetMode(AUTOMATIC);
    }
    else if (value1 == "MANUAL"){
      PitchPID.pid.SetMode(MANUAL);
    }
    else{
      Serial3.println("Invalid mode setting");  
    }
  }
  
  else if (parameter == "PITCH_DIR") {
    if (value1 == "DIRECT"){
      PitchPID.pid.SetControllerDirection(DIRECT);
    }
    else if (value1 == "REVERSE"){
      PitchPID.pid.SetControllerDirection(REVERSE);
    }
    else{
      Serial3.println("Invalid mode setting");  
    }
  }
  
  else if (parameter == "PITCH_TUNE") {
    PitchPID.pid.SetTunings(value1.toFloat(),value2.toFloat(),value3.toFloat()); 
  }
  
  else if (parameter == "PITCH_SP") {
    PitchPID.set_setpoint(value1.toFloat()); 
  }
  
  else if (parameter == "SPRAY_PARK_POS") {
    autosprayer.set_pos_parked(value1.toInt());
  }
  
  else if (parameter == "SPRAY_OFF_POS") {
    autosprayer.set_pos_off(value1.toInt());
  }
  
  else if (parameter == "SPRAY_ON_POS") {
    autosprayer.set_pos_on(value1.toInt());
  }
  
  else if (parameter == "SPRAY_DUR") {
    autosprayer.set_duration(value1.toInt());
  }
  
  else if (parameter == "SPRAY_TRIGGER") {
    autosprayer.set_trigger();  
  }
  
  else if (parameter == "SPRAY_ARM") {
    autosprayer.set_armed(true);  
  }
  
  else if (parameter == "SPRAY_DISARM") {
    autosprayer.set_armed(false);  
  }
  
  else{
    found = false;
  }
  
  if(!found){
    Serial3.println("Parameter not found");
    Serial3.println(""); 
  }
  Serial3.println("");

}

void input::inputHandler() {
  
  switch (state){
    
    //Wait for a complete string
    case 0:
      if(inputString.endsWith("\r\n")){
        inputString.toUpperCase();
        Serial3.print(inputString);
        state = 1;  
      }
      break;
      
    //Split out tokens  
    case 1:
    {
        int pos;
      
        if(inputString.indexOf(" ")>0){
          tokens[0] = inputString.substring(0,inputString.indexOf(" "));
          inputString.remove(0,inputString.indexOf(" ")+1);
          
          if(tokens[0] == "READ"){
            tokens[1] = inputString.substring(0,inputString.indexOf("\r\n"));
            read_parameter(tokens[1]);
            pos = 1;
          }
          
          else if(tokens[0] == "WRITE"){
            if(inputString.indexOf(":")>0){
              tokens[1] = inputString.substring(0,inputString.indexOf(":"));
              inputString.remove(0,inputString.indexOf(":")+1);
              pos = 2;
              while(inputString.indexOf(",")>0){
                tokens[pos++] = inputString.substring(0,inputString.indexOf(","));
                inputString.remove(0,inputString.indexOf(",")+1);
              }
              tokens[pos] = inputString.substring(0,inputString.indexOf("\r\n"));
              write_parameter(tokens[1],tokens[2],tokens[3],tokens[4],tokens[5]);
            }
            else{
              Serial3.println("Invalid instruction. Must be of the format '[READ/WRITE] [PARAMETER]:[VALUE1],[VALUE2],[VALUE3]'");
            }
          }
        }
        else{
          Serial3.println("Invalid instruction. Must be of the format '[READ/WRITE] [PARAMETER]:[VALUE1],[VALUE2],[VALUE3]'");
        }
        state = 99;
        break;
    }
    
    //Clear the input string  
    case 99:
      for(int i=0; i<6; i++){
        tokens[i] = "";  
      }
      inputString.remove(0);
      state = 0;
      break;
  }    
}

void serialEvent3() {
  while (Serial3.available()) {
    // get the new byte:
    char inChar = (char)Serial3.read();
    // add it to the inputString:
    input::inputString += inChar;
  }
}
