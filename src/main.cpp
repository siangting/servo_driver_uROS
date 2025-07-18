#include <Arduino.h>

// ===== UART2 → GPIO18,19 =====
#define RX_PIN    18
#define TX_PIN    19
HardwareSerial BusSerial(2);

// ===== Bus 協議 =====
#define HDR       0x55
#define CMD_MOVE  0x01
#define CMD_LOAD  0x1F

// ===== 參數設定 =====
const uint8_t NUM_SERVOS   = 12;
const uint8_t INTER_STEPS  = 20;    // 內插步數
const uint16_t INTER_DELAY = 10;    // 每步延遲 (ms)
const uint16_t CYCLE_DELAY = 100;  // 姿勢之間停留 (ms)

// ===== 初始姿勢 =====
const float initialPose[NUM_SERVOS] = {
  120.0,  65.0, 120.0,  65.0, 120.0, 120.0,
  120.0,  95.0, 120.0,  65.0, 120.0, 120.0
};

// ===== Move forward 四組姿勢 =====
const float forwardPose[4][NUM_SERVOS] = {
  {  90.0,  65.0, 120.0, 100.0, 120.0, 120.0,  90.0, 130.0, 120.0,  65.0, 120.0, 150.0 },
  {  90.0,  65.0, 120.0,  65.0, 120.0, 120.0,  90.0,  95.0, 120.0,  65.0, 120.0, 120.0 },
  { 120.0,  90.0, 180.0,  65.0,  90.0, 150.0, 120.0,  95.0, 150.0, 100.0,  90.0, 120.0 },
  { 120.0,  65.0, 180.0,  65.0,  90.0, 120.0, 120.0,  95.0, 150.0,  65.0,  90.0, 120.0 }
};

// ===== 當前姿勢緩存 =====
float currentPose[NUM_SERVOS];

// ==== Bus 封包 & 校驗 ====
uint8_t calcCHK(const uint8_t *b) {
  uint16_t s = 0;
  for (uint8_t i = 2; i < b[3] + 2; i++) s += b[i];
  return ~s;
}

void sendPack(uint8_t id, uint8_t cmd, const uint8_t *p, uint8_t n) {
  uint8_t buf[6 + n];
  buf[0] = buf[1] = HDR;
  buf[2] = id; 
  buf[3] = n + 3;
  buf[4] = cmd;
  for (uint8_t i = 0; i < n; i++) buf[5 + i] = p[i];
  buf[5 + n] = calcCHK(buf);
  BusSerial.write(buf, 6 + n);
}

void enableTorque(uint8_t id) {
  uint8_t on = 1;
  sendPack(id, CMD_LOAD, &on, 1);
}

// 角度 → 0~1000 量測值
void moveServoDeg(uint8_t id, float deg) {
  deg = constrain(deg, 0.0f, 240.0f);
  uint16_t pos = (uint16_t)(deg / 240.0f * 1000.0f);
  uint8_t p[4] = {
    (uint8_t)(pos & 0xFF),
    (uint8_t)(pos >> 8),
    0x64, 0x00    // 移動時間 100 ms
  };
  sendPack(id, CMD_MOVE, p, 4);
}

// 平滑內插到 targetPose
void moveToPose(const float targetPose[NUM_SERVOS]) {
  // 計算每步增量
  float delta[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    delta[i] = (targetPose[i] - currentPose[i]) / INTER_STEPS;
  }

  // 分步送指令
  for (uint8_t step = 1; step <= INTER_STEPS; step++) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      float ang = currentPose[i] + delta[i] * step;
      moveServoDeg(i + 1, ang);
    }
    delay(INTER_DELAY);
  }
  // 更新當前姿勢
  memcpy(currentPose, targetPose, sizeof(currentPose));
}

void setup() {
  Serial.begin(115200);
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  // 啟用所有伺服扭力
  for (uint8_t id = 1; id <= NUM_SERVOS; id++) {
    enableTorque(id);
    delay(20);
  }
  delay(200);

  // 初始化 currentPose，並先回到初始姿勢
  memcpy(currentPose, initialPose, sizeof(initialPose));
  moveToPose(initialPose);
  delay(500);
}

void loop() {
  static uint8_t idx = 0;
  // 到下一組前進姿勢
  moveToPose(forwardPose[idx]);
  idx = (idx + 1) % 4;
  delay(CYCLE_DELAY);
}
