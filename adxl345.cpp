#include "adxl345.h"

adxl345 accel;

void adxl345::init(){
  
  uint8_t max_readings = 100;
  uint8_t count = 0;
  axis x;
  axis y;
  axis z;
  
  pinMode(INT1, INPUT);
  
  Serial.println("Set to +/-16g, 13-bit mode");
  //Set to +/-16g, 13-bit mode
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(DATA_FORMAT_REG);
  Wire1.write(0x0B);
  Wire1.endTransmission();
  
  Serial.println("Start measurement");
  //Start measurement
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(POWER_CTL_REG);
  Wire1.write(0x08);
  Wire1.endTransmission();
  
  Serial.println("Enable DATA_READY interrupt");
  //Enable DATA_READY interrupt
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(INT_ENABLE_REG);
  Wire1.write(0x80);
  Wire1.endTransmission();
  
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(X_OFFSET_REG);
  Wire1.write(0);
  Wire1.endTransmission();
  
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(Y_OFFSET_REG);
  Wire1.write(0);
  Wire1.endTransmission();
  
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(Z_OFFSET_REG);
  Wire1.write(0);
  Wire1.endTransmission();
  
  delay(20);
   
  while(count < max_readings){
    if(digitalRead(INT1)){
      read_accel(x.value,y.value,z.value);
      Serial.println(x.value);
      x.sum = x.sum + x.value;
      y.sum = y.sum + y.value;
      z.sum = z.sum + z.value;
      Serial.println(x.sum);
      count++;
    }  
  }
  
  x.average = x.sum / max_readings;
  y.average = y.sum / max_readings;
  z.average = z.sum / max_readings;
  
  x.offset = -round(x.average/4);
  y.offset = -round(y.average/4);
  z.offset = -round((z.average-256)/4);

  Serial.print("X Offset: ");
  Serial.println(x.offset);
  Serial.print("Y Offset: ");
  Serial.println(y.offset);
  Serial.print("Z Offset: ");
  Serial.println(z.offset);
  Serial.println("");
  
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(X_OFFSET_REG);
  Wire1.write(x.offset);
  Wire1.endTransmission();
  
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(Y_OFFSET_REG);
  Wire1.write(y.offset);
  Wire1.endTransmission();
  
  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write(Z_OFFSET_REG);
  Wire1.write(z.offset);
  Wire1.endTransmission();

}

void adxl345::main(){
  
  if((millis()%20==0)&!read_complete){
    read_complete = true;
    pitch = calc_pitch();  
  }
  else{
    read_complete = false;  
  }
  
}

double adxl345::get_pitch(){
  return pitch;  
}

void adxl345::read_accel(int16_t &x, int16_t &y, int16_t &z){
  
  while(!digitalRead(INT1)){};

  Wire1.beginTransmission((int)ADXL345_ADDRESS);
  Wire1.write((int)X_REG);
  if(Wire1.endTransmission() == 0){
    Wire1.requestFrom((int)ADXL345_ADDRESS, 6);
  }
  else
    return;
  
  while(Wire1.available() < 6){};
  
  x = Wire1.read(); // receive high byte (overwrites previous reading)
  x |= Wire1.read() << 8; // receive low byte as lower 8 bits
  y = Wire1.read(); // receive high byte (overwrites previous reading)
  y |= Wire1.read() << 8; // receive low byte as lower 8 bits
  z = Wire1.read(); // receive high byte (overwrites previous reading)
  z |= Wire1.read() << 8; // receive low byte as lower 8 bits

}

double adxl345::calc_pitch(){
  
  int16_t xout, yout, zout;
  
  read_accel(xout,yout,zout);
  
  return atan2(yout,zout);
  
}
