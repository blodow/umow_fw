#include "Arduino.h"

//#include "tc.h"
//#include "tc_interrupt.h"

class uMowPWMTimers {
  public:
    uMowPWMTimers();

    void init();

    void setDutyCycle(uint16_t c1, uint16_t c2);
    void setDutyCycle1(uint16_t c);
    void setDutyCycle2(uint16_t c);

    void setDutyCycleMow(uint16_t c);

  private:
    void initMowMotor();
    void initDriveMotors();
};
