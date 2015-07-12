#include <EEPROM.h>

#include <RFBeeCCxCfg.h>
#include <RFBeeGlobals.h>
#include <RFBeeCore.h>
#include <RFBeeCCx.h>
#include <RFBeeSendRev.h>
#include <RFBeeConfig.h>
#include <RFBeeSerial.h>
#include <RFBeeSpi.h>

// used for polling the RF received data
#define GDO0 2 

#define START1      0x53
#define START2      0x01
#define END1        0x2f
#define END2        0x45
#define DTALEN      6

void InitializeConfig (bool force) {
  if (force || !Config.initialized()) {
    Config.reset();
    Config.set(CONFIG_MY_ADDR, 2);
    Config.set(CONFIG_DEST_ADDR, 1);
    Config.set(CONFIG_ADDR_CHECK, 0x00); // later: 0x01 (check dst==address) or 0x02 (check dst==address or dst==0 (broadcast))
    
    Config.set(CONFIG_TX_THRESHOLD, 0x01); // 0x01: transmit for each single byte
    Config.set(CONFIG_BDINDEX, 0x00); // 0x00: 9600
    Config.set(CONFIG_PAINDEX, 0x07); // 0x07: 10dB transmitting power
    Config.set(CONFIG_CONFIG_ID, 0x04); // 0x04: 868mhz, 4.8kbps, sensitivity
    Config.set(CONFIG_OUTPUT_FORMAT, 0x00); // 0x00: payload only, 0x01: src,dst,payload, 0x02: payloadlen,src,dst,payload,rssi,lqi
    Config.set(CONFIG_RFBEE_MODE, 0x02); // 0x00: transceive, 0x01: transmit, 0x02: receive, 0x03: lowpower (wake on radio)
  }
}

void setup() {
  Serial.begin(115200);
  
  InitializeConfig(true);
  
  CCx.PowerOnStartUp();
  setCCxConfig();
  pinMode(GDO0, INPUT);
}

void loop() {
  receiveJoystick();
  delay(20);
}

unsigned char rxData[200];
unsigned char len;
unsigned char srcAddress;
unsigned char destAddress;
char rssi;
unsigned char lqi;
int result;

// message to be sent on via UART
char data[100] = {0x69, 0x69};

void receiveJoystick() {
  if (digitalRead(GDO0) == HIGH) {
    result = receiveData(rxData, &len, &srcAddress, &destAddress, (unsigned char *)&rssi, &lqi);
    data[2] = rssi;
    data[3] = lqi;
    data[4] = len;
    for (unsigned char i = 0; i < len; i++) {
      data[5+i] = (rxData[i]);
    }
    
    Serial.write(data, len + 5);
  } 
}


