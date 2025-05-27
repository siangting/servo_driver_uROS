#include <Arduino.h>

// UART2 → GPIO18,19 （保持你现在跑得通的配置）
#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);

// STBus 协议
#define HDR       0x55
#define CMD_READ  0x1C
#define CMD_MOVE  0x01
#define CMD_LOAD  0x1F

uint8_t chk(const uint8_t* b){
  uint16_t s=0;
  for(uint8_t i=2;i<b[3]+2;i++) s+=b[i];
  return ~s;
}

void sendPack(uint8_t id, uint8_t cmd, const uint8_t* p, uint8_t n){
  uint8_t buf[6+n];
  buf[0]=buf[1]=HDR;
  buf[2]=id; buf[3]=n+3; buf[4]=cmd;
  for(uint8_t i=0;i<n;i++) buf[5+i]=p[i];
  buf[5+n]=chk(buf);
  BusSerial.write(buf,6+n);
}

void scan(){
  Serial.println("scan 1~30...");
  for(uint8_t i=1;i<=30;i++){
    while(BusSerial.available()) BusSerial.read();
    sendPack(i,CMD_READ,nullptr,0);
    delay(30);
    if(BusSerial.available()>=8){
      Serial.printf("  ID=%d OK\n",i);
      while(BusSerial.available()) BusSerial.read();
    }
  }
}

void setup(){
  Serial.begin(115200);
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);

  // 上载扭力
  uint8_t on=1;
  sendPack(1,CMD_LOAD,&on,1);
  delay(50);

  // 扫描 + 初次摆动
  scan();

  uint8_t p0[]={0,0,100,0};
  sendPack(1,CMD_MOVE,p0,4);
  sendPack(2,CMD_MOVE,p0,4);
  delay(500);
  uint8_t p90[]={0x77,0x01,100,0};
  sendPack(1,CMD_MOVE,p90,4);
  sendPack(2,CMD_MOVE,p90,4);
}

void loop() {
  static bool dir = false;
  uint8_t p0[] = { 0x00,0x00, 100,0x00 };
  uint8_t p90[] = { 0x77,0x01, 100,0x00 };

  if (dir) {
    sendPack(1, CMD_MOVE, p0, 4);
    Serial.println("Servo 1 → Move to 0°");
    delay(500);
    sendPack(1, CMD_MOVE, p90, 4);
    Serial.println("Servo 1 → Move to 90°");
    delay(500);
  } else {
    sendPack(2, CMD_MOVE, p0, 4);
    Serial.println("Servo 2 → Move to 0°");
    delay(500);
    sendPack(2, CMD_MOVE, p90, 4);
    Serial.println("Servo 2 → Move to 90°");
    delay(500);
  }

  dir = !dir;
  delay(800);
}
