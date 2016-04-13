#ifndef input_h
#define input_h

#include "Arduino.h"

namespace input{
  
  extern String inputString;
  extern String tokens[6];
  extern int state;
  
  void parser(const String& input);
  void read_parameter(const String& parameter);
  void write_parameter(const String& parameter, const String& value1, const String& value2, const String& value3, const String& value4);
  void inputHandler();
  
}

#endif
