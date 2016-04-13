#ifndef SBUS2_h
#define SBUS2_h

#include "Arduino.h"
#include "timers.h"

#define SBUS_FAILSAFE_INACTIVE 0
#define SBUS_FAILSAFE_ACTIVE   1
#define SBUS_STARTBYTE1        0x0f
#define SBUS_STARTBYTE2        0x8f
#define SBUS_ENDBYTE           0x00

class SBUS {
	public:
                SBUS(HardwareSerial& in, HardwareSerial& out):serialIn(in),serialOut(out){

                }
		void begin();
                void mainloop();
		int getChannel(int channel);
                int getChannelMin(int channel);
                int getChannelMax(int channel);
                void setChannel(int value);
		bool getFailsafeStatus();
                uint32_t getBadFrames();
                void setChannel(uint32_t channel, uint32_t value);
                void setChannelLimits();
                
	private:
                int channels_in[18];
                int channels_out[18];
                int channels_alt[18];
                int _channels_max[18];
                int _channels_min[18];
		bool _failsafe;
                uint32_t badFrames;
                uint8_t sbusFrameRx[25];
                uint8_t sbusFrameTx[25];
                uint32_t decodeStep = 1;
                uint32_t step;
                HardwareSerial& serialIn;
                HardwareSerial& serialOut;
                uint32_t transmitFrame();
                int8_t receiveFrame();
                void decodeFrame();
                void encodeFrame();
                void substituteChannels();
                void copyChannels();
                bool checkChannel(uint8_t channel, uint32_t value);
};

extern SBUS sbusRx;

#endif
