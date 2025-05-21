#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <micro_ros_utilities/type_utilities.h>
#include "CytronMotorDriver.h"

const uint8_t MOTOR1_PWM = 23;
const uint8_t MOTOR1_DIR = 22;
const uint8_t MOTOR2_PWM = 21;
const uint8_t MOTOR2_DIR = 19;
const uint8_t MOTOR3_PWM = 18;
const uint8_t MOTOR3_DIR = 5;
const uint8_t MOTOR4_PWM = 4;
const uint8_t MOTOR4_DIR = 15;
const MODE MOTOR_MODE = PWM_DIR;

CytronMD motor1(MOTOR_MODE, MOTOR1_PWM, MOTOR1_DIR); // 우후
CytronMD motor2(MOTOR_MODE, MOTOR2_PWM, MOTOR2_DIR); // 우전
CytronMD motor3(MOTOR_MODE, MOTOR3_PWM, MOTOR3_DIR); // 좌후
CytronMD motor4(MOTOR_MODE, MOTOR4_PWM, MOTOR4_DIR); // 좌전

rcl_subscription_t subscriber_motors;
std_msgs__msg__Int32MultiArray motors_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13
#define MOTOR_COUNT 4

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
  if (msg->data.size >= 4) {
    // Apply speed values to each motor
    int16_t motor1_speed = constrain((int16_t)msg->data.data[0], -255, 255);
    int16_t motor2_speed = constrain((int16_t)msg->data.data[1], -255, 255);
    int16_t motor3_speed = constrain((int16_t)msg->data.data[2], -255, 255);
    int16_t motor4_speed = constrain((int16_t)msg->data.data[3], -255, 255);
    
    motor1.setSpeed(motor1_speed);
    motor2.setSpeed(motor2_speed);
    motor3.setSpeed(motor3_speed);
    motor4.setSpeed(motor4_speed);
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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_motors", "", &support));

  // Initialize the array message
  micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_basic_type_sequence_capacity = MOTOR_COUNT;
  RCCHECK(micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    &motors_msg,
    conf));
    
  // Create subscriber for the array message
  RCCHECK(rclc_subscription_init_default(
    &subscriber_motors,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motors/cmd"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber_motors,
    &motors_msg,
    &motors_cmd_callback,
    ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
