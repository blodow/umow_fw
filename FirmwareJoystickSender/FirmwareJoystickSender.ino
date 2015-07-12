#include <EEPROM.h>

#include <RFBeeCCxCfg.h>
#include <RFBeeGlobals.h>
#include <RFBeeCore.h>
#include <RFBeeCCx.h>
#include <RFBeeSendRev.h>
#include <RFBeeConfig.h>
#include <RFBeeSerial.h>
#include <RFBeeSpi.h>

#define START1      0x53
#define START2      0x01
#define END1        0x2f
#define END2        0x45
#define DTALEN      6

unsigned char dtaSend[8] = {START1, START2, 0, 0, 0, 0, END1, END2};

// initial values for x and y, regarded as zero position
short zeroX;
short zeroY;

void InitializeConfig (bool force) {
  if (force || !Config.initialized()) {
    Config.reset();
    Config.set(CONFIG_MY_ADDR, 1);
    Config.set(CONFIG_DEST_ADDR, 2);
    Config.set(CONFIG_ADDR_CHECK, 0x00); // later: 0x01 (check dst==address) or 0x02 (check dst==address or dst==0 (broadcast))
    
    Config.set(CONFIG_TX_THRESHOLD, 0x01); // 0x01: transmit for each single byte
    Config.set(CONFIG_BDINDEX, 0x00); // 0x00: 9600
    Config.set(CONFIG_PAINDEX, 0x07); // 0x07: 10dB transmitting power
    Config.set(CONFIG_CONFIG_ID, 0x04); // 0x04: 868mhz, 4.8kbps, sensitivity
    Config.set(CONFIG_OUTPUT_FORMAT, 0x00); // 0x00: payload only, 0x01: src,dst,payload, 0x02: payloadlen,src,dst,payload,rssi,lqi
    Config.set(CONFIG_RFBEE_MODE, 0x01); // 0x00: transceive, 0x01: transmit, 0x02: receive, 0x03: lowpower (wake on radio)
    Serial.println("initialized config.");
  }
}

// read analog pin, averaged over 32 samples
int getAnalog(int pin)
{
  unsigned int sum = 0;
  for (int i = 0; i < 32; i++)
  {
    sum += analogRead(pin);
  }
  sum = ((sum >> 5) & 0x03ff); // divide by 32

  return sum;
}

void setup() {
  Serial.begin(115200);

  // without this, the joystick drifts, getting ever higher numbers
  // without stick movement: "turn on power regulation MOSFET" ??
  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);

  InitializeConfig(true);
  
  zeroX = getAnalog(A4);
  zeroY = getAnalog(A5);
  
  CCx.PowerOnStartUp();
  setCCxConfig();
  
  Serial.println("initialized.");
}

void loop() {
  short x = getAnalog(A4) - zeroX;
  short y = getAnalog(A5) - zeroY;
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(", y: ");
  Serial.println(y);

  unsigned char xlo =  x & 0x00FF;
  unsigned char xhi = (x & 0xFF00) >> 8;
  unsigned char ylo =  y & 0x00FF;
  unsigned char yhi = (y & 0xFF00) >> 8;

  if (dtaSend[2] != xlo || dtaSend[3] != xhi ||
      dtaSend[4] != ylo || dtaSend[5] != yhi) {
    dtaSend[2] = xlo;
    dtaSend[3] = xhi;
    dtaSend[4] = ylo;
    dtaSend[5] = yhi;

    transmitData(dtaSend,8,Config.get(CONFIG_MY_ADDR),Config.get(CONFIG_DEST_ADDR)); 
    Serial.println("Sent vie RFBee");
  }
  delay(50);
}

