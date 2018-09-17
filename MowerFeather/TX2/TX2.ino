// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 RFM95_RSTclass. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

// struct from FeatherJoyWing.h
typedef struct {
  uint8_t pinId;
  bool pressed;
  bool hasChanged;
} FJBUTTON;

#define BEEP         9
#define LED         13
#define PULSE1      10
#define PULSE2      12

#define BUT_SJOY    19
#define BUT_S2       0
#define BUT_S3      17
#define BUT_S5      17
#define BUT_TRIGGER 18

#define JOY_X       14
#define JOY_Y       15

/* for feather m0  
*/
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95_(RFM95_CS, RFM95_INT);

// Singleton of joystick helper
int16_t joyX_ = 0;
int16_t joyY_ = 0;

uint8_t buttons_ = 0x0;

const int8_t numZeroingReadings_ = 20;
int8_t firstCounter_ = 0;
int16_t firstX_[100] = {0};
int16_t firstY_[100] = {0};
int16_t zeroX_ = 0, zeroY_ = 0;
const int16_t joystickDeadX_ = 20;
const int16_t joystickDeadY_ = 10;

int16_t joyDead(int16_t v, int16_t joystickDead) {
  if (v > 0) {
    if (v < joystickDead) {
      return 0;      
    } else {
      return v - joystickDead;
    }
  } else {
    if (v > -joystickDead) {
      return 0;      
    } else {
      return v + joystickDead;
    }
  }
}

void joystickCallback(int16_t x, int16_t y)
{
  if (firstCounter_ < numZeroingReadings_) {
    firstX_[firstCounter_] = x;
    firstY_[firstCounter_] = y;
    firstCounter_++;
  } else if (firstCounter_ == numZeroingReadings_) {
    int32_t sumX = 0, sumY = 0;
    for (uint8_t i = 0; i < numZeroingReadings_; ++i) {
      sumX += firstX_[i];
      sumY += firstY_[i];
    }
    zeroX_ = sumX / (float)numZeroingReadings_;
    zeroY_ = sumY / (float)numZeroingReadings_;
    firstCounter_++;
  } else {
//    Serial.print("joy zerp: ");
//    Serial.print(zeroX_); Serial.print(" : "); Serial.println(zeroY_);
//    Serial.print("joy before: ");
//    Serial.print(x); Serial.print(" : "); Serial.println(y);
    joyX_ = joyDead(x - zeroX_, joystickDeadX_);
    joyY_ = joyDead(y - zeroY_, joystickDeadY_);
//    Serial.print("joy after: ");
//    Serial.print(joyX_); Serial.print(" : "); Serial.println(joyY_);
  }
}

void buttonCallback(FJBUTTON* buttons, uint8_t count)
{
  uint8_t but = 0x0;
  for(int i = 0; i < count; i++) {
    but |= (!!buttons[i].pressed) << i;
  }    
  buttons_ = but;
}

void on(int ms) {
  digitalWrite(LED, HIGH);
  delay(ms);
}

void off(int ms) {
  digitalWrite(LED, LOW);
  delay(ms);
}

void setup() 
{
  // This prevents the device from starting from battery (no USB)
  //while ( ! Serial ) { delay( 1 ); }
  //Serial.begin( 9600 );

  pinMode(BEEP, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PULSE1, OUTPUT);
  pinMode(PULSE2, OUTPUT);
  
  pinMode(BUT_SJOY, INPUT_PULLUP);
  pinMode(BUT_S2, INPUT_PULLUP);
  pinMode(BUT_S3, INPUT_PULLUP);
  pinMode(BUT_S5, INPUT_PULLUP);
  pinMode(BUT_TRIGGER, INPUT_PULLUP);

  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  analogReference(AR_EXTERNAL);
  
     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa TX Test!");
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95_.init()) {
    Serial.println("LoRa radio init failed");
    on(1000); // LED ON
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95_.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95_.setTxPower(23, false);

//  joy_.begin();
//
//  joy_.registerJoystickCallback(joystickCallback);
//  joy_.registerButtonCallback(buttonCallback);
}

uint8_t packetNum_ = 0;  // packet counter, we increment per xmission
char radioPacket_[10] = {0};
bool sent_ = false;

void sendMessage() {
  if (sent_) {
    rf95_.waitPacketSent();
  }
  // Serial.println("Transmitting..."); // Send a message to rf95_server
     
  radioPacket_[0] = packetNum_++;
  radioPacket_[1] = joyX_ >> 8;
  radioPacket_[2] = joyX_ & 0xff;
  radioPacket_[3] = joyY_ >> 8;
  radioPacket_[4] = joyY_ & 0xff;
  radioPacket_[5] = buttons_;
  
  // Serial.println("Sending...");
  delay(10);
  rf95_.send((uint8_t *)radioPacket_, 10);

  sent_ = true;
//  Serial.println("Waiting for packet to complete..."); 
//  delay(10);
//  rf95.waitPacketSent();
//  // Now wait for a reply
//  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//  uint8_t len = sizeof(buf);
//
//  Serial.println("Waiting for reply...");
//  if (rf95.waitAvailableTimeout(1000))
//  { 
//    // Should be a reply message for us now   
//    if (rf95.recv(buf, &len))
//   {
//      Serial.print("Got reply: ");
//      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);    
//    }
//    else
//    {
//      Serial.println("Receive failed");
//    }
//  }
//  else
//  {
//    Serial.println("No reply, is there a listener around?");
//  }
}

void checkButton(int pin, FJBUTTON* buttons, int buttonIndex, const char* name) {
  bool p = (digitalRead(pin) == LOW); 
  buttons[buttonIndex].pinId = buttonIndex;
  buttons[buttonIndex].pressed = p;
  if (p) {
    Serial.println(name);
  }
}

void checkButtons() {
  const static int buttonCount = 5;
  static FJBUTTON buttons[buttonCount] = {0};
  
  digitalWrite(PULSE1, LOW);
  delay(10);
  // checkButton(BUT_S4, buttons, 5, "S4");
  // checkButton(BUT_S6, buttons, 6, "S6");
  checkButton(BUT_S5, buttons, 4, "S5");
  checkButton(BUT_TRIGGER, buttons, 1, "TRIGGER");
  digitalWrite(PULSE1, HIGH);
  
  digitalWrite(PULSE2, LOW);
  delay(10);
  checkButton(BUT_SJOY, buttons, 0, "JOY");
  checkButton(BUT_S2, buttons, 2, "S2");
  checkButton(BUT_S3, buttons, 3, "S3");
  digitalWrite(PULSE2, HIGH);

  buttonCallback(buttons, buttonCount);
}

void checkJoy() {
  int16_t x = analogRead(JOY_X);
  int16_t y = analogRead(JOY_Y);

  Serial.print("x: ");
  Serial.print(x - 512);
  Serial.print("\ty: ");
  Serial.println(y - 512);

  joystickCallback(x - 512, y - 512);
}

void loop()
{
//  joy_.update();
  delay(10);

  checkButtons();
  checkJoy();

  sendMessage();
}
