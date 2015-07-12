#include <DualVNH5019MotorShield.h>

short RSSIdecode(unsigned char rssiEnc){
    short rssi;
    unsigned char rssiOffset=74;  // is actually dataRate dependant, but for simplicity assumed to be fixed.

    // RSSI is coded as 2's complement see section 17.3 RSSI of the cc1100 datasheet
    if (rssiEnc >= 128) {
      rssi = (short)((short)(rssiEnc - 256) / 2) - rssiOffset;
    } else {
      rssi = (rssiEnc / 2) - rssiOffset;
    }
    return rssi;
}

void ramp(DualVNH5019MotorShield& motors, int duration, int from, int to) {
  int diff = abs(to - from);
  int sign = (to - from) >= 0 ? 1 : -1;

  unsigned long period = duration / diff;
  unsigned long start_time = millis();

  int step_size = 1;
  while (duration < diff / step_size) {
    step_size *= 2;
  }

  for (int i = 0 ; i < diff; i += step_size) {
    int speed = from + sign * i;
    motors.setSpeeds(speed, speed);
    unsigned long elapsed = millis() - start_time;
    int wait = (i * period) - elapsed;
    if (wait > 0) {
      delay(wait);
    }
  }
}

DualVNH5019MotorShield motors;

void setup() {
  // Serial1 is UART to RFBee
  Serial1.begin(115200);
  // Serial is UART to Host PC
  Serial.begin(115200);

  motors.init();
  motors.setSpeeds(0, 0);

  Serial.print("intialized.");
}

#define println(x) Serial.println(x)
#define print(x) Serial.print(x)

void setDirectMotorFromJoy(short x, short y) {
  int speedLeft = 0;
  int speedRight = 0;
  
  // go ahead or go back
  if (y == 0) {
    speedLeft  = x;
    speedRight = speedLeft;
    if (x >= 0) {
      print("forward:       ");
    } else {
      print("goback:        ");
    }
  } else if (x == 0) {
    speedLeft  = map(abs(y), 0, 100, 0, 50);
    speedRight = speedLeft;
    if (y > 0) {
      print("right-rev:     ");
      speedRight  = -speedRight;
    } else {
      print("left-rev:      ");
      speedLeft  = -speedLeft;
    }
  } else if (x >= 0 && y <= 0 && abs(x) >= abs(y)) {
    print("forward-left:  ");
    speedLeft   = x + y;
    speedRight  = x;
  } else if (x >= 0 && y <= 0 && abs(x) < abs(y)) {
    print("left-rev:      ");
    speedLeft  = 100 - min(abs(x), abs(y));
    speedLeft  = map(speedLeft, 0, 100, 0, 60);
    speedRight = speedLeft;
    speedLeft  = -speedLeft;
  } else if (x >= 0 && y >= 0 && abs(x) >= abs(y)) {
    print("forward-right: ");
    speedLeft   = x;
    speedRight  = x - y;
  } else if (x >= 0 && y >= 0 && abs(x) < abs(y)) {
    print("right-rev:     ");
    speedLeft   = 100 - min(abs(x), abs(y));
    speedLeft   = map(speedLeft, 0, 100, 0, 60);
    speedRight  = speedLeft;
    speedRight  = -speedRight;
  } else if (x <= 0 && y <= 0 && abs(x) > abs(y)) {
    print("back-left:     ");
    speedRight = 0x80 + abs(x);
    speedLeft  = 0x80 + abs(x) - abs(y);
  } else if (x <= 0 && y >= 0 && abs(x) > abs(y)) {
    print("back-right:    ");
    speedLeft  = 0x80 + abs(x);
    speedRight = 0x80 + abs(x) - abs(y);
  } else {
    return ;
  }

  print("left, right speed: ");
  print(speedLeft);
  print(", ");
  println(speedRight);
  
  motors.setSpeeds(speedLeft, speedRight);
}

// count is the state index for the reading state machine
int count = 0;
unsigned char xlo = 0, xhi = 0;
unsigned char ylo = 0, yhi = 0;

#define START1      0x53
#define START2      0x01
#define END1        0x2f
#define END2        0x45

int rssi;
int lqi;
int len;
char incomingByte;

void loop() {
  // send motor command only when we receive data:
  if (Serial1.available() > 0) {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    // read the incoming byte:
    int incomingByte = Serial1.read();
    if (count == 0 && incomingByte == 0x69) {
      count = 1;
    } else if (count == 1 && incomingByte == 0x69) {
      count = 2;
    } else if (count == 2) {
      rssi = incomingByte;
      count = 3;
    } else if (count == 3) {
      lqi = incomingByte;
      count = 4;
    } else if (count == 4) {
      len = incomingByte;
      count = 5;
    } else if (count == 5 && len > 0 && incomingByte == START1) {
      len--;
      count = 6;
    } else if (count == 6 && len > 0 && incomingByte == START2) {
      len--;
      count = 7;
    } else if (count == 7 && len > 0) {
      len--;
      xlo = incomingByte;
      count = 8;
    } else if (count == 8 && len > 0) {
      len--;
      xhi = incomingByte;
      count = 9;
    } else if (count == 9 && len > 0) {
      len--;
      ylo = incomingByte;
      count = 10;
    } else if (count == 10 && len > 0) {
      len--;
      yhi = incomingByte;
      
      Serial.print("read: xhi, xlo, yhi, ylo: ");
      Serial.print(xhi, HEX);
      Serial.print(", ");
      Serial.print(xlo, HEX);
      Serial.print(", ");
      Serial.print(yhi, HEX);
      Serial.print(", ");
      Serial.print(ylo, HEX);
      Serial.print(" --> signal strength: ");
      Serial.print(RSSIdecode(rssi));
      Serial.println("dB");
      
      count = 11;
    } else if (count == 11 && len > 0 && incomingByte == END1) {
      count = 12;
    } else if (count == 12 && incomingByte == END2) {
      short x = xlo | (xhi << 8);
      short y = ylo | (yhi << 8);
      setDirectMotorFromJoy(x, y);
      count = 0;
    } else {
      count = 0;
    } 
  }
}

