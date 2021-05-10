#include <stdlib.h>

enum class Direction : uint8_t
{
  clockwise,
  counterclockwise
};

enum class Button : uint8_t{
  left,
  up,
  down,
  right,
  exec,
  none,
  unknown
};

enum ApplicationMode : uint8_t
{
  initialising,
  speed_controller
};

struct HardwareConfiguration
{
  uint16_t digitPotMaxValue;
  uint16_t digitPotMinValue;
};

struct ApplicationConfiguration
{
  uint16_t encoderDivisor; // Scale down the rate of pulses received from the rotary encoder by this figure
  unsigned long serialBaudRate;
};

struct Configuration
{
  HardwareConfiguration hardware;
  ApplicationConfiguration software;
};

struct ApplicationState
{
  ApplicationMode mode;
  bool executePressedThisLoop;

  int keypadAdcLevel;
  float speedPercent;
};

