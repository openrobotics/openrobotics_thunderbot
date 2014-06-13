#ifndef TALOS_BASE_CONTROLLER_PARSER_H
#define TALOS_BASE_CONTROLLER_PARSER_H

#include <string>
#include <sstream>
#include <stdlib.h>
#include "talos_base_controller/ArduinoMessage.h"

class Parser
{
  public:
    Parser();
    ArduinoMessage parse(const std::string& message);
};
#endif
