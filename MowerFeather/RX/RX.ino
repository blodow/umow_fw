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

RH_RF95 rf95(RFM95_CS, RFM95_INT);

////////////////////////////////////////////////////////////////////////////////
// LED
bool toggleLed_ = false;
#define LED 13

////////////////////////////////////////////////////////////////////////////////
// DISPLAY
Adafruit_SSD1306 display = Adafruit_SSD1306();
#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

////////////////////////////////////////////////////////////////////////////////
// DRIVE MOTORS
uMowMotorControl motors;
int16_t lastMotor1_ = 0;
int16_t lastMotor2_ = 0;
int16_t targetMotor1_ = 0;
int16_t targetMotor2_ = 0;
const int16_t maxDelta_ = 40;

////////////////////////////////////////////////////////////////////////////////
// MOWING MOTOR
Servo mowingMotor;
bool mowing = false;
bool mowButton = false;

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

void drawJoy(uint8_t x, uint8_t width, int8_t value) {
  display.fillRect(x,0,width,32,0);
  if (value > 0) {
    display.fillRect(x,16-value,width,value,1);
  } else {
    display.fillRect(x,16,width,-value,1);
  }
}

void setDirectMotorFromJoy(int16_t x, int16_t y) {
  const uint16_t black = 0, white = 1;
  
  const int maxVal = 250; // 512 would be 24V, but motors are rated for 12V !!
  double diff = (-y)/540.0;
  double mean = (-x)/540.0;

  {
    uint16_t drawX = -diff * 16;
    uint16_t drawY = -mean * 16;
    display.fillRect(60,0,32,32,black);
    display.drawFastVLine(76,0,32,white);
    display.drawFastHLine(60,16,32,white);
    display.drawLine(76,16,76+drawX,16+drawY, white);
  }
    
  double l = (mean+diff) * 2 * maxVal;
  double r = (mean-diff) * 2 * maxVal;

  targetMotor1_ = constrain(l, -maxVal, maxVal);
  targetMotor2_ = constrain(r, -maxVal, maxVal);

  int16_t delta = targetMotor1_ - lastMotor1_;
  if (delta > maxDelta_) delta = maxDelta_;
  if (delta < -maxDelta_) delta = -maxDelta_;
  lastMotor1_ += delta;
  
  delta = targetMotor2_ - lastMotor2_;
  if (delta > maxDelta_) delta = maxDelta_;
  if (delta < -maxDelta_) delta = -maxDelta_;
  lastMotor2_ += delta;
  motors.setSpeeds(lastMotor1_, lastMotor2_);

        display.fillRect(30,16,30,16,0);
        display.setCursor(30,16);
        display.print(lastMotor1_);
        display.setCursor(30,24);
        display.print(lastMotor2_);

  drawJoy(122, 1, (l * 16.0) / maxVal);             // thin line: target speed
  drawJoy(123, 5, (lastMotor1_ * 16.0) / maxVal);   // thick line: actual, "smoother" speed

  drawJoy(112, 1, (r * 16.0) / maxVal);
  drawJoy(113, 5, (lastMotor2_ * 16.0) / maxVal);
}

void emergencyStop() {
  toggleMowing(false);
  lastMotor1_ = 0; // prevent ramping
  lastMotor2_ = 0; // prevent ramping
  setDirectMotorFromJoy(0,0);
    display.fillScreen(0);
  display.setCursor(50,12);
  display.print("no radio");
  display.display();
  // Beep
//  analogWrite(A0, 1023); delay(500); analogWrite(A0, 0); delay(500); 
}

void toggleMowing(bool enable) {
  if (enable) {
    mowingMotor.writeMicroseconds(1900);
  } else {
    mowingMotor.writeMicroseconds(1500);
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

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    on(1000);
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    on(1000);off(1000);on(1000);
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // setup display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.fillScreen(0);
  display.display(); // actually display all of the above

  // setup dual motor driver
  motors.init();
  // setup mowing motor as a simple servo
  mowingMotor.attach(15); // A1
}

void loop() {
  unsigned long now = millis();
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    toggleLed_ = !toggleLed_;
    digitalWrite(LED, toggleLed_);
    if (rf95.recv(buf, &len)) {
      if (len == 10) {
        lastMillisJoy_ = now;
        uint8_t counter = buf[0];
        
        // Handle joystick / drive motors
        int16_t joyX = buf[1] << 8 | buf[2];
        int16_t joyY = buf[3] << 8 | buf[4];

        display.fillScreen(0);
        display.setCursor(0,0);
        display.println("joyX");
        display.println("joyY");
      
        display.fillRect(30,0,30,16,0);
        display.setCursor(30,0);
        display.print(joyX);
        display.setCursor(30,8);
        display.print(joyY);

        setDirectMotorFromJoy(joyX, joyY);

        // Handle buttons / mowing toggle
        bool mowButtonMsg = buf[5] & 0x2; // let's see
        if (mowButton == false && mowButtonMsg) {
          mowButton = true;
          mowing = !mowing;
          toggleMowing(mowing);
          display.fillRect(0,16,40,8,0);
          display.setCursor(0,16);
          display.print (mowing?"mow on":"mow off");
        } else if (mowButton == true && !mowButtonMsg) {
          mowButton = false;
        }

        display.display();
      }
      //Serial.println(rf95.lastRssi(), DEC);
      //// Send a reply
      //uint8_t data[] = "And hello back to you";
      //rf95.send(data, sizeof(data));
      //rf95.waitPacketSent();
      //Serial.println("Sent a reply");
    } else {
      Serial.println("Receive failed");
    }
    toggleLed_ = !toggleLed_;
    digitalWrite(LED, toggleLed_);
  }
  if (now - lastMillisJoy_ > deadManInterval_) {
    emergencyStop();
  }
}


