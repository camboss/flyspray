#include "sbus2.h"

SBUS sbusRx (Serial1,Serial2);

void SBUS::begin() {

  step = 1;
  
  for (byte i = 0; i<18; i++) {
    channels_in[i] = 0;
    channels_out[i] = 0;
    channels_alt[i] = 0;
  }
  for (byte i = 0; i<18; i++) {
    _channels_max[i] = 1680;
  }
  for (byte i = 0; i<18; i++) {
    _channels_min[i] = 366;
  }
  _channels_min[4] = 140;
  _channels_max[4] = 1910;
  
  _channels_min[5] = 140;
  _channels_max[5] = 1910;
  
  _channels_min[6] = 140;
  _channels_max[6] = 1910;
  
  _channels_min[7] = 140;
  _channels_max[7] = 1910;
  
  _failsafe = SBUS_FAILSAFE_INACTIVE;
  
  badFrames = 0;
  
}

void SBUS::mainloop(){

      if(receiveFrame() > 0){
        decodeFrame();
        copyChannels();
        substituteChannels();
        encodeFrame();
        transmitFrame();
      }
  
}

void SBUS::substituteChannels(){
  for (int i = 0; i<18; i++) {
    if(channels_alt[i] > 0){
      channels_out[i] = channels_alt[i];
    }
  }    
}

void SBUS::copyChannels(){
  for (int i = 0; i<18; i++) {
    channels_out[i] = channels_in[i];
  } 
}

int8_t SBUS::receiveFrame(){
  
  static uint8_t i = 0;
  
  while((serialIn.available() > 0) && (i<25)){
    sbusFrameRx[i] = serialIn.read();
    i++;
  }
  
  if((sbusFrameRx[0] == SBUS_STARTBYTE1) || (sbusFrameRx[0] == SBUS_STARTBYTE2)){
    if(i==25){ //and if we have read 25 bytes, a full packet length
      i = 0;
      if(sbusFrameRx[24] == SBUS_ENDBYTE){ //and if the end byte is correct
        return 1;
      }
    }
  }
  else{
    i = 0; //first byte should always be a start byte, anything else dump the packet and keep receiving
  }
  return -1;
  
}

uint32_t SBUS::transmitFrame(){
  return serialOut.write(sbusFrameTx,25);  
}

void SBUS::decodeFrame(){
  
  int temp_channel;
  bool error = false;
  
  
  if(!checkChannel(1,((sbusFrameRx[1] | sbusFrameRx[2] << 8) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(2,((sbusFrameRx[2] >> 3 | sbusFrameRx[3] << 5) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(3,((sbusFrameRx[3] >> 6 | sbusFrameRx[4] << 2 | sbusFrameRx[5] << 10) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(4,((sbusFrameRx[5] >> 1 | sbusFrameRx[6] << 7) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(5,((sbusFrameRx[6] >> 4 | sbusFrameRx[7] << 4) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(6,((sbusFrameRx[7] >> 7 | sbusFrameRx[8] << 1 | sbusFrameRx[9] << 9) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(7,((sbusFrameRx[9] >> 2 | sbusFrameRx[10] << 6) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(8,((sbusFrameRx[10] >> 5 | sbusFrameRx[11] << 3) & 0x07FF))){
    error = true;
  }
  
  /*if(!checkChannel(8,((sbusFrameRx[12] | sbusFrameRx[13] << 8) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(9,((sbusFrameRx[13] >> 3 | sbusFrameRx[14] << 5) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(10,((sbusFrameRx[14] >> 6 | sbusFrameRx[15] << 2 | sbusFrameRx[16] << 10) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(11,((sbusFrameRx[16] >> 1 | sbusFrameRx[17] << 7) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(12,((sbusFrameRx[17] >> 4 | sbusFrameRx[18] << 4) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(13,((sbusFrameRx[18] >> 7 | sbusFrameRx[19] << 1 | sbusFrameRx[20] << 9) & 0x07FF))){
    error = true;
  }
  
  if(!checkChannel(14,((sbusFrameRx[20] >> 2 | sbusFrameRx[21] << 6) & 0x07FF))){
    error = true;
  }

  if(!checkChannel(15,((sbusFrameRx[21] >> 5 | sbusFrameRx[22] << 3) & 0x07FF))){
    error = true;
  }*/
  
  if ((sbusFrameRx[23] >> 3) & 0x0001) {
    _failsafe = SBUS_FAILSAFE_ACTIVE;
  } else {
    _failsafe = SBUS_FAILSAFE_INACTIVE;
  }
  
  if(error){
    badFrames++;
  }

}

bool SBUS::checkChannel(uint8_t channel, uint32_t value){
  if((value <= _channels_max[channel-1]) && (value >= _channels_min[channel-1])){  
    channels_in[channel-1] = value;
    return true;
  }
  else{
    return false;
  }   
}

void SBUS::encodeFrame(){
  sbusFrameTx[0] = sbusFrameRx[0];
  sbusFrameTx[1] = (channels_out[0] & 0xFF);
  sbusFrameTx[2] = ((channels_out[0]>>8 | channels_out[1]<< 3) & 0xFF);
  sbusFrameTx[3] = ((channels_out[1]>>5 | channels_out[2]<< 6) & 0xFF);
  sbusFrameTx[4] = (channels_out[2]>>2 & 0xFF);
  sbusFrameTx[5] = ((channels_out[2]>>10 | channels_out[3]<<1) & 0xFF);
  sbusFrameTx[6] = ((channels_out[3]>>7 | channels_out[4]<<4) & 0xFF);
  sbusFrameTx[7] = ((channels_out[4]>>4 | channels_out[5]<<7) & 0xFF);
  sbusFrameTx[8] = (channels_out[5]>>1 & 0xFF);
  sbusFrameTx[9] = ((channels_out[5]>>9 | channels_out[6]<<2) & 0xFF);
  sbusFrameTx[10] = ((channels_out[6]>>6 | channels_out[7]<<5) & 0xFF);
  sbusFrameTx[11] = (channels_out[7]>>3 & 0xFF);
  sbusFrameTx[12] = (channels_out[8] & 0xFF);
  sbusFrameTx[13] = ((channels_out[8]>>8 | channels_out[9]<<3) & 0xFF);
  sbusFrameTx[14] = ((channels_out[9]>>5 | channels_out[10]<<6) & 0xFF);
  sbusFrameTx[15] = (channels_out[10]>>2 & 0xFF);
  sbusFrameTx[16] = ((channels_out[10]>>10 | channels_out[11]<<1) & 0xFF);
  sbusFrameTx[17] = ((channels_out[11]>>7 | channels_out[12]<<4) & 0xFF);
  sbusFrameTx[18] = ((channels_out[12]>>4 | channels_out[13]<<7) & 0xFF);
  sbusFrameTx[19] = (channels_out[13]>>1 & 0xFF);
  sbusFrameTx[20] = ((channels_out[13]>>9 | channels_out[14]<<2) & 0xFF);
  sbusFrameTx[21] = ((channels_out[14]>>6 | channels_out[15]<<5) & 0xFF);
  sbusFrameTx[22] = (channels_out[15]>>3 & 0xFF);
  sbusFrameTx[23] = sbusFrameRx[23];
  sbusFrameTx[24] = sbusFrameRx[24];
}

uint32_t SBUS::getBadFrames(){
  return badFrames;  
}

int SBUS::getChannel(int channel) {
  if (channel < 1 or channel > 18) {
    return 0;
  } else {
    return channels_in[channel - 1];
  }
}

int SBUS::getChannelMin(int channel) {
  if (channel < 1 or channel > 18) {
    return 0;
  } else {
    return _channels_min[channel - 1];
  }
}

int SBUS::getChannelMax(int channel) {
  if (channel < 1 or channel > 18) {
    return 0;
  } else {
    return _channels_max[channel - 1];
  }
}

void SBUS::setChannel(uint32_t channel, uint32_t value){
  if(channel >= 1 and channel <= 18){
    channels_alt[channel-1] = value;
  }  
}

bool SBUS::getFailsafeStatus(){
  return _failsafe;
}

/*void SBUS::setChannelLimits(void){
  for(int i = 0; i < 18; i++){
    if(channels_in[i] < _channels_min[i]){
      _channels_min[i] = channels_in[i];
    }
    if(channels_in[i] > _channels_max[i]){
      _channels_max[i] = channels_in[i];
    }  
  }
}*/
