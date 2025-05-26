#include <Arduino.h>
#include <micro_ros_utils.hpp>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <trajectory_msgs/msg/joint_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

// ======= UART Bus Servo 設定 =======
#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);
#define HDR      0x55
#define CMD_MOVE 0x01
#define CMD_LOAD 0x1F

uint8_t calcCHK(const uint8_t*b){
  uint16_t s=0;
  for(uint8_t i=2;i<b[3]+2;i++) s+=b[i];
  return ~s;
}
void sendPack(uint8_t id,uint8_t cmd,const uint8_t*p,uint8_t n){
  uint8_t buf[6+n];
  buf[0]=buf[1]=HDR;
  buf[2]=id; buf[3]=n+3; buf[4]=cmd;
  for(uint8_t i=0;i<n;i++) buf[5+i]=p[i];
  buf[5+n]=calcCHK(buf);
  BusSerial.write(buf,6+n);
}
void enableTorque(uint8_t id){
  uint8_t on=1; sendPack(id,CMD_LOAD,&on,1);
}
void moveServoDeg(uint8_t id, float deg){
  deg = constrain(deg, 0.0f, 240.0f);
  uint16_t pos = (uint16_t)(deg/240.0f*1000.0f);
  uint8_t p[4] = {
    (uint8_t)(pos & 0xFF),
    (uint8_t)(pos >> 8),
    0x64, 0x00    // 100 ms
  };
  sendPack(id,CMD_MOVE,p,4);
}

// ======= micro-ROS 變數 =======
rcl_subscription_t         subscription;
rclc_executor_t            executor;
rcl_node_t                 node;
rclc_support_t             support;
rcl_allocator_t            allocator;
trajectory_msgs__msg__JointTrajectory traj_msg;

// callback：取 points[0].positions 裡的前兩個值
void traj_callback(const void * msgin) {
  auto *t = (const trajectory_msgs__msg__JointTrajectory *)msgin;
  if (t->points.size == 0) return;

  auto &pt = t->points.data[0];
  size_t n = pt.positions.size < 2 ? pt.positions.size : 2;
  Serial.printf("Got JointTrajectory, point0 with %u positions\n", (unsigned)n);
  for (size_t i = 0; i < n; i++) {
    moveServoDeg(i+1, (float)pt.positions.data[i]);
  }
}

void setup() {
  // 1. 序列埠 & 舵機初始化
  Serial.begin(115200);
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  enableTorque(1);
  enableTorque(2);
  delay(200);

  // 2. micro-ROS 初始化
  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "servo_node", "", &support);

  // 3. 建立並初始化 JointTrajectory 訊息
  trajectory_msgs__msg__JointTrajectory__init(&traj_msg);

  // 3.1 joint_names 長度 2
  rosidl_runtime_c__String__Sequence__init(&traj_msg.joint_names, 2);
  rosidl_runtime_c__String__assign(&traj_msg.joint_names.data[0], "servo_1");
  rosidl_runtime_c__String__assign(&traj_msg.joint_names.data[1], "servo_2");

  // 3.2 points 序列長度 1
  trajectory_msgs__msg__JointTrajectoryPoint__Sequence__init(&traj_msg.points, 1);
  // positions 長度 2
  traj_msg.points.data[0].positions.data =
    (double*)malloc(2 * sizeof(double));
  traj_msg.points.data[0].positions.size =
  traj_msg.points.data[0].positions.capacity = 2;
  // time_from_start (可留 0)
  traj_msg.points.data[0].time_from_start.sec = 0;
  traj_msg.points.data[0].time_from_start.nanosec = 0;

  // 4. 訂閱 /servo_trajectory
  rclc_subscription_init_default(
    &subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectory),
    "/servo_trajectory"
  );

  // 5. Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &subscription, &traj_msg,
    traj_callback, ON_NEW_DATA
  );
}

void loop() {
  // 每 10 ms spin 一次
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
