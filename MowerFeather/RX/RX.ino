// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <uMowMotorControl.h>
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////
// RADIO
/* for feather m0 RFM9x */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

RH_RF95 rf95_(RFM95_CS, RFM95_INT);

////////////////////////////////////////////////////////////////////////////////
// LED
bool toggleLed_ = false;
#define LED 13

////////////////////////////////////////////////////////////////////////////////
// DISPLAY
Adafruit_SSD1306 display_ = Adafruit_SSD1306();
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

////////////////////////////////////////////////////////////////////////////////
// DRIVE MOTORS
uMowMotorControl motors_;
int16_t lastMotor1_ = 0;
int16_t lastMotor2_ = 0;
int16_t targetMotor1_ = 0;
int16_t targetMotor2_ = 0;
const int16_t maxDelta_ = 40;

////////////////////////////////////////////////////////////////////////////////
// TRIM
int8_t trim_ = 0;
bool trimLeftButton_ = false;
bool trimRightButton_ = false;

////////////////////////////////////////////////////////////////////////////////
// MOWING MOTOR
const int MOW_SPEED_ON = 1900;
const int MOW_SPEED_OFF = 1500;
Servo mowingMotor_;
int mowingSpeedTarget_ = MOW_SPEED_OFF;
int mowingSpeedLast_ = MOW_SPEED_OFF;
bool mowing_ = false;
bool mowButton_ = false;

////////////////////////////////////////////////////////////////////////////////
// DEAD MAN SWITCH
const unsigned long deadManInterval_ = 500; // ms = half a second;
static unsigned long lastMillisJoy_ = 0;

////////////////////////////////////////////////////////////////////////////////
// Helper functions

void on(int ms) {
  digitalWrite(LED, HIGH); delay(ms);
}

void off(int ms) {
  digitalWrite(LED, LOW); delay(ms);
}

void drawJoy(uint8_t x, uint8_t y, uint8_t width, int8_t value) {
  display_.fillRect(x, y, width, 32, 0);
  if (value > 0) {
    display_.fillRect(x, y + 16 - value, width, value, 1);
  } else {
    display_.fillRect(x, y + 16, width, -value, 1);
  }
}

void setDirectMotorFromJoy(int16_t x, int16_t y) {
  char buf[10];

  display_.fillRect(0, 0, 32, 32, 0);
  display_.setCursor(0, 0);
  sprintf(buf, "x%4d", x); display_.println(buf);
  sprintf(buf, "y%4d", y); display_.println(buf);

  int16_t temp = x; x = -y; y = -temp;
  const uint16_t black = 0, white = 1;

  const int maxVal = 250; // 512 would be 24V, but motors are rated for 12V !!
  double diff = (-y) / 1024.0;
  double mean = (-x) / 1024.0;

  double l = (mean + diff) * 2 * maxVal;
  double r = (mean - diff) * 2 * maxVal;

  targetMotor1_ = constrain(l, -maxVal, maxVal);
  targetMotor2_ = constrain(r, -maxVal, maxVal);

  if (trim_ > 0) {
    targetMotor2_ *= (1.0 - 0.01 * abs(trim_));
  } else if (trim_ < 0) {
    targetMotor1_ *= (1.0 - 0.01 * abs(trim_));
  }

  int16_t delta = targetMotor1_ - lastMotor1_;
  if (delta > maxDelta_) delta = maxDelta_;
  if (delta < -maxDelta_) delta = -maxDelta_;
  lastMotor1_ += delta;

  delta = targetMotor2_ - lastMotor2_;
  if (delta > maxDelta_) delta = maxDelta_;
  if (delta < -maxDelta_) delta = -maxDelta_;
  lastMotor2_ += delta;

  motors_.setSpeeds(lastMotor1_, lastMotor2_);

  // print l, r  values
  sprintf(buf, "l%4d", lastMotor1_); display_.println(buf);
  sprintf(buf, "r%4d", lastMotor2_); display_.println(buf);

  // draw direction indicator
  {
    uint16_t drawX = -diff * 16;
    uint16_t drawY = -mean * 16;
    display_.fillRect(0, 32, 32, 32, black);
    // display_.drawFastVLine(16, 32, 32, white);
    // display_.drawFastHLine(0, 48, 32, white);
    display_.drawLine(16, 48, 16 - drawX, 48 + drawY, white);
  }

  // draw speed "bar chart"
  drawJoy(0, 32, 5, (lastMotor1_ * 16.0) / maxVal);   // thick line: actual, "smoother" speed
  drawJoy(6, 32, 1, (targetMotor1_ * 16.0) / maxVal);             // thin line: target speed
  drawJoy(25, 32, 1, (targetMotor2_ * 16.0) / maxVal);
  drawJoy(27, 32, 5, (lastMotor2_ * 16.0) / maxVal);
}

void emergencyStop() {
  toggleMowing(false);
  mowingMotor_.writeMicroseconds(MOW_SPEED_OFF);
  lastMotor1_ = 0; // prevent ramping
  lastMotor2_ = 0; // prevent ramping
  setDirectMotorFromJoy(0, 0);
  display_.fillScreen(0);
  display_.setCursor(0, 96);
  display_.print("no\nradio");
  displayConnection();
  display_.display();
  // Beep
}

void toggleMowing(bool enable) {
  if (enable) {
    mowingSpeedTarget_ = MOW_SPEED_ON;
  } else {
    mowingSpeedTarget_ = MOW_SPEED_OFF;
  }
}

void setMowingMotorSpeed() {
  int delta = mowingSpeedTarget_ - mowingSpeedLast_;
  if (delta > maxDelta_) delta = maxDelta_;
  if (delta < -maxDelta_) delta = -maxDelta_;
  mowingSpeedLast_ += delta;
  mowingMotor_.writeMicroseconds(mowingSpeedLast_);
}

void displayTrim() {
  display_.fillRect(0, 64, 32, 8, 0);
  display_.setCursor(1, 64);
  if (trim_ == 0) {
    display_.println("- T -");
    return;
  }
  char buf[10];
  if (trim_ < 0) {
    sprintf(buf, "%-2dT", abs(trim_)); display_.println(buf);
  } else if (trim_ > 0) {
    sprintf(buf, "  T%2d", trim_); display_.println(buf);
  }
}

void displayConnection() {
  display_.fillRect(0, 112, 32, 8, 0);
  display_.setCursor(0, 112);
  display_.println(rf95_.lastRssi(), DEC);
}

void displayMowing() {
  display_.fillRect(0, 120, 32, 8, 0);
  display_.setCursor(0, 120);
  int w = 32 - 6;
  if (mowingSpeedLast_ == MOW_SPEED_ON) {
    display_.print ("M");
    display_.fillRect(6, 120, w, 8, WHITE);
    display_.setTextColor(BLACK);
    display_.print (mowing_ ? " on" : " 1!!");
    display_.setTextColor(WHITE);
  } else if (mowingSpeedLast_ == MOW_SPEED_OFF) {
    display_.print (!mowing_ ? "M off" : "M 0!!");
  } else {
    display_.print ("M");
    w *= (mowingSpeedLast_ - MOW_SPEED_OFF) / (float)(MOW_SPEED_ON - MOW_SPEED_OFF);
    display_.fillRect(6, 120, w, 8, 1);
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa RX Test!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95_.init()) {
    Serial.println("LoRa radio init failed");
    on(1000);
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95_.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    on(1000); off(1000); on(1000);
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95_.setTxPower(23, false);

  // setup display
  display_.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display_.setRotation(3);
  display_.setTextSize(1);
  display_.setTextColor(WHITE);
  display_.fillScreen(0);
  display_.display(); // actually display all of the above

  // setup dual motor driver
  motors_.init();
  // setup mowing motor as a simple servo
  mowingMotor_.attach(15); // A1
}

void loop() {
  unsigned long now = millis() + deadManInterval_;
  if (rf95_.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    toggleLed_ = !toggleLed_;
    digitalWrite(LED, toggleLed_);
    if (rf95_.recv(buf, &len)) {
      if (len == 10) {
        lastMillisJoy_ = now;
        uint8_t counter = buf[0];

        // Handle joystick / drive motors
        int16_t joyX = buf[1] << 8 | buf[2];
        int16_t joyY = buf[3] << 8 | buf[4];

        setDirectMotorFromJoy(joyX, joyY);

        // Handle buttons / mowing toggle
        bool mowButtonMsg = buf[5] & 0x2; // let's see
        if (mowButton_ == false && mowButtonMsg) {
          mowButton_ = true;
          mowing_ = !mowing_;
          toggleMowing(mowing_);
        } else if (mowButton_ == true && !mowButtonMsg) {
          mowButton_ = false;
        }

        // Handle buttons / trim left
        bool trimLeftButtonMsg = buf[5] & 0x4;
        if (trimLeftButton_ == false && trimLeftButtonMsg) {
          trimLeftButton_ = true;
          --trim_;
        } else if (trimLeftButton_ == true && !trimLeftButtonMsg) {
          trimLeftButton_ = false;
        }

        // Handle buttons / trim right
        bool trimRightButtonMsg = buf[5] & 0x8;
        if (trimRightButton_ == false && trimRightButtonMsg) {
          trimRightButton_ = true;
          ++trim_;
        } else if (trimRightButton_ == true && !trimRightButtonMsg) {
          trimRightButton_ = false;
        }
      }
      //Serial.println(rf95_.lastRssi(), DEC);
      //// Send a reply
      //uint8_t data[] = "And hello back to you";
      //rf95_.send(data, sizeof(data));
      //rf95_.waitPacketSent();
      //Serial.println("Sent a reply");
    } else {
      Serial.println("Receive failed");
    }
    toggleLed_ = !toggleLed_;
    digitalWrite(LED, toggleLed_);
  }

  if (now - lastMillisJoy_ > deadManInterval_) {
    emergencyStop();
  } else {
    display_.fillRect(0, 96, 32, 16, 0);
    setMowingMotorSpeed();
    displayTrim();
    displayConnection();
    displayMowing();
    display_.display();
  }
}
