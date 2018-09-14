#include "uMowPWMTimers.h"


uMowPWMTimers::uMowPWMTimers() {}

void
uMowPWMTimers::init() {
  initDriveMotors();                              // set up 16Khz on TCC0
  initMowMotor();                                 // set up 50Hz on TC4
}

void
uMowPWMTimers::initMowMotor() {
//  // Output 50Hz PWM on timer TC4 (14-bit resolution)
////  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(7) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
////                    GCLK_GENDIV_ID(5);            // Select Generic Clock (GCLK) 5
////  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
////
////  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
////                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
////                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
////                     GCLK_GENCTRL_ID(5);          // Select GCLK4
////  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
//
//  // Enable the port multiplexer for the PWM channels: timer TCC1 outputs
//  PORT->Group[g_APinDescription[15].ulPort].PINCFG[g_APinDescription[15].ulPin].bit.PMUXEN = 1;
//
//  // Connect the TCC1 timer to the port output - port pins are paired odd PMUO and even PMUXE
//  // F & E specify the timers: TCC0, TCC1 and TCC2
//  PORT->Group[g_APinDescription[15].ulPort].PMUX[g_APinDescription[15].ulPin >> 1].reg = PORT_PMUX_PMUXE_E;
//
//  // Feed GCLK5 to TCC0 and TCC1
//  // TODO can i router GCLK4 to all my timers?
//  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable
//                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK5
//                     GCLK_CLKCTRL_ID_TC4_TC5;   // Feed it to TC4 and TC5
//  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
//
//  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
//  while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
//
//  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 |
//                            TC_CTRLA_WAVEGEN_NPWM |
//                            TC_CTRLA_PRESCALER_DIV16 |
//                            TC_CTRLA_ENABLE;
//  //TC4->COUNT16.PER.reg = 20000;
//  TC4->COUNT16.CC[0].reg = 1500;
//  while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
//
//
////  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
////  REG_TC4_WAVE |= TCC_WAVE_POL(0xF) |           // Revers the output polarity on all TCC1 outputs
////                   TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC1
////  while (TC4->SYNCBUSY.bit.WAVE);               // Wait for synchronization
//
////  // Each timer counts up to a maximum or TOP value set by the PER register,
////  // this determines the frequency of the PWM operation:
////  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
////  REG_TC4_PER = 20000;      // Set the frequency of the PWM on TCC1 to 50Hz
////  while(TC4->SYNCBUSY.bit.PER);
//
////  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
////  REG_TC4_CCB0 = 1500;       // TCC1 CCB0 - center the servo on 2
////  while(TC4->SYNCBUSY.bit.CCB0);
//
////  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
////  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK5 by 8
////                   TC_CTRLA_ENABLE;             // Enable the TC4 output
////  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void
uMowPWMTimers::initDriveMotors() {
  // Output 16KHz PWM on timer TCC0 (16-bit resolution)
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the 2 PWM channels: timer TCC0 outputs
  const uint8_t CHANNELS = 2;
  const uint8_t pwmPins[] = {5, 10};
  for (uint8_t i = 0; i < CHANNELS; i++)
  {
     PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
  }
  // Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg  = PORT_PMUX_PMUXO_F ;
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg = PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;    // Setup normal PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC0_PER = 512;      // Set the frequency of the PWM on TCC0 to 50Hz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB1 = 0;       // TCC0 CCB1
  while(TCC0->SYNCBUSY.bit.CCB1);
  REG_TCC0_CCB2 = 0;       // TCC0 CCB2
  while(TCC0->SYNCBUSY.bit.CCB2);

  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV2 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void
uMowPWMTimers::setDutyCycle(uint16_t c1, uint16_t c2) {
  setDutyCycle1(c1);
  setDutyCycle2(c2);
}

void
uMowPWMTimers::setDutyCycle1(uint16_t c) {
  REG_TCC0_CCB1 = c;
  while(TCC0->SYNCBUSY.bit.CCB1);
}

void
uMowPWMTimers::setDutyCycle2(uint16_t c) {
  REG_TCC0_CCB2 = c;
  while(TCC0->SYNCBUSY.bit.CCB2);
}

void
uMowPWMTimers::setDutyCycleMow(uint16_t c) {
//  TC4->COUNT16.CC[0].reg = 1500;
//  while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}

