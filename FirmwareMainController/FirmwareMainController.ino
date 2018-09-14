#include <Servo.h>

#include <TimerOne.h>

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
Servo mowingMotor;
bool mowing;

void setup() {
  // Serial1 is UART to RFBee
  Serial1.begin(115200);
  // Serial is UART to Host PC
  Serial.begin(115200);

  mowingMotor.attach(5);
  mowing = false;
  toggleMowing(mowing);
  
  motors.init();
  stopMotors();
//  Timer1.initialize(150000);
//  Timer1.attachInterrupt(stopMotors);
//  Timer1.start();

  Serial.print("intialized.");
}

void stopMotors(void) {
    motors.setSpeeds(0, 0);
}

void toggleMowing(bool enable) {
  if (enable) {
    mowingMotor.writeMicroseconds(1900);
  } else {
    mowingMotor.writeMicroseconds(1500);
  }
}

#define println(x) Serial.println(x)
#define print(x) Serial.print(x)

void setDirectMotorFromJoy(int x, int y) {
  const int maxVal = 200;
  double diff = y/540.0;
  double mean = x/540.0;
  double l = (mean+diff) * 2 * maxVal;
  double r = (mean-diff) * 2 * maxVal;
  
  print("mean, diff: ");
  print(mean);
  print(", ");
  print(diff);
  print("\tl, r: ");
  print(l);
  print(", ");
  println(r);
  motors.setSpeeds(constrain(l, -maxVal, maxVal), constrain(r, -maxVal, maxVal));
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

bool lastMowingClickState = false;

void loop() {
  println("hello world");
  delay(1000);
  motors.setSpeeds(100, -100);
  delay(1000);
  motors.setSpeeds(-100, 100);
  delay(1000);
  motors.setSpeeds(0, 0);
//  delay(/1000);
//  motors.setSpeeds(100, -100);
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
      
      print("read: xhi, xlo, yhi, ylo: ");
      print(xhi);
      print(", ");
      print(xlo);
      print(", ");
      print(yhi);
      print(", ");
      print(ylo);
      print(" --> signal strength: ");
      print(RSSIdecode(rssi));
      println("dB");
      
      count = 11;
    } else if (count == 11 && len > 0 && incomingByte == END1) {
      count = 12;
    } else if (count == 12 && incomingByte == END2) {
      short x = xlo | (xhi << 8);
      short y = ylo | (yhi << 8);
      println(x);
      println(y);
  
      if (y > 510) {
        if (!lastMowingClickState) {
          lastMowingClickState = true;
          mowing = !mowing;
          toggleMowing(mowing);
        }
      } else {
        lastMowingClickState = false;
        setDirectMotorFromJoy(x, y);
      }
//      Timer1.restart();
      count = 0;
    } else {
      count = 0;
    } 
  }
}

