#include <Arduino.h>
#include <micro_ros_utils.hpp>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

// ======= 可調整舵機數量 =======
#define NUM_SERVOS 18

// ======= UART Bus Servo 設定 =======
#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);
#define HDR      0x55
#define CMD_MOVE 0x01
#define CMD_LOAD 0x1F

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

// ==========================================================
// ✔ S-curve smoothing 資料
// ==========================================================
float currentDeg[NUM_SERVOS] = {0};
float startDeg[NUM_SERVOS]   = {0};
float targetDeg[NUM_SERVOS]  = {0};
bool  moving[NUM_SERVOS]     = {false};

float T_total = 0.5f;    // 移動總時間 0.5 秒
float dt      = 0.02f;   // 每 20ms 更新一次

TaskHandle_t motionTaskHandle = NULL;

float s_curve(float a0, float a1, float t, float T) {
  float d = a1 - a0;
  return a0 + 0.5f * d * (1 - cos(PI * t / T));
}

// ==========================================================
// ✔ 非阻塞背景絲滑 task（修正版）
// ==========================================================
void MotionTask(void *p) {
  static float t[NUM_SERVOS] = {0};

  for (;;) {

    for (int id = 0; id < NUM_SERVOS; id++) {

      if (!moving[id]) continue;  // 該 servo 沒有要動

      t[id] += dt;
      if (t[id] > T_total) t[id] = T_total;

      // S-curve 計算
      float deg = s_curve(startDeg[id], targetDeg[id], t[id], T_total);
      currentDeg[id] = deg;

      // output to servo
      uint16_t pos = (uint16_t)(deg / 240.0f * 1000.0f);
      uint8_t pbuf[4] = {
        (uint8_t)(pos & 0xFF),
        (uint8_t)(pos >> 8),
        0x64, 0x00
      };
      sendPack(id + 1, CMD_MOVE, pbuf, 4);

      // 完成
      if (t[id] >= T_total) {
        currentDeg[id] = targetDeg[id];
        moving[id] = false;
        t[id] = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 非阻塞
  }
}

// ==========================================================
// ✔ 改動：moveServoDeg → 設 target，不直接動舵機
// ==========================================================
void moveServoDeg(uint8_t id, float deg) {
  uint8_t idx = id - 1;
  targetDeg[idx] = constrain(deg, 0.0f, 240.0f);

  startDeg[idx]  = currentDeg[idx];   // 固定此段 S-curve 起點
  moving[idx]    = true;              // 啟動該軸運動
}

// ======= micro-ROS 變數 =======
rcl_subscription_t         subscription;
rclc_executor_t            executor;
rcl_node_t                 node;
rclc_support_t             support;
rcl_allocator_t            allocator;
trajectory_msgs__msg__JointTrajectory traj_msg;

// callback：取 positions
void traj_callback(const void * msgin) {
  auto *t = (const trajectory_msgs__msg__JointTrajectory *)msgin;
  if (t->points.size == 0) return;

  auto &pt = t->points.data[0];
  size_t n = pt.positions.size < NUM_SERVOS ? pt.positions.size : NUM_SERVOS;

  Serial.printf("Got JointTrajectory point with %u positions\n", (unsigned)n);

  for (size_t i = 0; i < n; i++) {
    moveServoDeg(i + 1, (float)pt.positions.data[i]);
  }
}

void setup() {
  // ===== 1. 序列埠 & 舵機初始化 =====
  Serial.begin(115200);
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  for (uint8_t id = 1; id <= NUM_SERVOS; id++) {
    enableTorque(id);
    delay(20);
  }
  delay(200);

  // ===== 2. 啟動絲滑背景 task =====
  xTaskCreatePinnedToCore(
    MotionTask, "motion", 4096, NULL, 1, &motionTaskHandle, 1);

  // ===== 3. micro-ROS 初始化 =====
set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();

  // 1. 初始化 rcl_init_options_t
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);

  // 2. 🌟 設定 ROS_DOMAIN_ID 為 1 🌟
  rcl_init_options_set_domain_id(&init_options, 1); 

  // 3. 使用 init_options 初始化 support (使用 rclc_support_init_with_options)
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator); 
  
  // 4. 釋放 init_options
  rcl_init_options_fini(&init_options);
  // ===== 4. 建立 node、subscription、executor =====
  rclc_node_init_default(&node, "servo_node", "", &support);

  trajectory_msgs__msg__JointTrajectory__init(&traj_msg);

  rosidl_runtime_c__String__Sequence__init(&traj_msg.joint_names, NUM_SERVOS);
  char tmp[16];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    sprintf(tmp, "servo_%u", i + 1);
    rosidl_runtime_c__String__assign(&traj_msg.joint_names.data[i], tmp);
  }

  trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(&traj_msg.points, 1);
  traj_msg.points.data[0].positions.data =
    (double *)malloc(NUM_SERVOS * sizeof(double));
  traj_msg.points.data[0].positions.size =
    traj_msg.points.data[0].positions.capacity = NUM_SERVOS;

  traj_msg.points.data[0].time_from_start.sec = 0;
  traj_msg.points.data[0].time_from_start.nanosec = 0;

  rclc_subscription_init_default(
    &subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    "/servo_trajectory"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &subscription, &traj_msg,
    traj_callback, ON_NEW_DATA
  );
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
