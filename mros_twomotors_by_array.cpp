/*
  ESP32 보드 사용을 상정
*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <micro_ros_utilities/type_utilities.h>
#include "CytronMotorDriver.h"

const uint8_t MOTORl_PWM = 23;
const uint8_t MOTORl_DIR = 22;
const uint8_t MOTORr_PWM = 21;
const uint8_t MOTORr_DIR = 19;
const MODE MOTOR_MODE = PWM_DIR;

CytronMD motorl(MOTOR_MODE, MOTORl_PWM, MOTORl_DIR); // 좌
CytronMD motorr(MOTOR_MODE, MOTORr_PWM, MOTORr_DIR); // 우


rcl_subscription_t subscriber_motors;
std_msgs__msg__Int32MultiArray motors_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13
#define MOTOR_COUNT 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void motors_cmd_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  // Check if the message contains the right number of elements
  if (msg->data.size >= 2) {
    // Apply speed values to each motor
    int16_t motorl_speed = constrain((int16_t)msg->data.data[0], -255, 255);
    int16_t motorr_speed = constrain((int16_t)msg->data.data[1], -255, 255);

    
    motorl.setSpeed(motorl_speed);
    motorr.setSpeed(motorr_speed);

  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  digitalWrite(LED_PIN, LOW); 

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_ground_motors", "", &support));

  // Initialize the array message
  micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_basic_type_sequence_capacity = MOTOR_COUNT;
  conf.max_ros2_type_sequence_capacity = 1;
  conf.max_string_capacity = 0;

 bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    &motors_msg,
    conf);

  if (!success) {
    error_loop();
  }


  // Initialize the MultiArray layout structure
  motors_msg.layout.dim.capacity = 1;
  motors_msg.layout.dim.size = 1;
  motors_msg.layout.dim.data[0].size = MOTOR_COUNT;
  motors_msg.layout.dim.data[0].stride = 1;
  motors_msg.layout.data_offset = 0;

  // Initialize the data array
  motors_msg.data.capacity = MOTOR_COUNT;
  motors_msg.data.size = MOTOR_COUNT;
  for(size_t i = 0; i < MOTOR_COUNT; i++) {
    motors_msg.data.data[i] = 0;
  }
    
  // Create subscriber for the array message
  RCCHECK(rclc_subscription_init_default(
    &subscriber_motors,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motors/speed"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber_motors,
    &motors_msg,
    &motors_cmd_callback,
    ON_NEW_DATA));

  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
