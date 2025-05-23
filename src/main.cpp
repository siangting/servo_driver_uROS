#include <Arduino.h>
#include <micro_ros_utils.hpp>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

// ======= UART Bus Servo 設定 =======
#define RX_PIN 18
#define TX_PIN 19
HardwareSerial BusSerial(2);
#define HDR      0x55
#define CMD_MOVE 0x01
#define CMD_LOAD 0x1F

uint8_t calcCHK(const uint8_t*b){
  uint16_t s=0; for(uint8_t i=2;i<b[3]+2;i++) s+=b[i];
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
    (uint8_t)(pos&0xFF),(uint8_t)(pos>>8),
    0x64,0x00  // 100 ms
  };
  sendPack(id,CMD_MOVE,p,4);
}

// ======= micro-ROS =======
std_msgs__msg__Float32MultiArray multi_msg;
rcl_subscription_t subscription;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

void multi_callback(const void *msgin) {
  auto *m = (const std_msgs__msg__Float32MultiArray *)msgin;
  size_t len = m->data.size;
  Serial.printf(">> got %zu angles\n", len);
  for (size_t i = 0; i < len && i < 2; i++) {
    moveServoDeg(i+1, m->data.data[i]);
  }
}

void setup() {
  Serial.begin(115200);
  BusSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  enableTorque(1);
  enableTorque(2);
  delay(200);

  // micro-ROS init
  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "servo_node","", &support);

  multi_msg.data.data = (float*)malloc(2 * sizeof(float));
  multi_msg.data.size = multi_msg.data.capacity = 2;

  rclc_subscription_init_default(
    &subscription, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/servo_angles"
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &subscription, &multi_msg,
    multi_callback, ON_NEW_DATA
  );
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
