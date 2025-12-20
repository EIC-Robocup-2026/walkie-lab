#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>

// Create an instance for the PCA9685 module
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

rcl_node_t node;
rcl_subscription_t subscriber_joint_cmd;
rcl_publisher_t publisher_servo_pos;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

sensor_msgs__msg__JointState joint_cmd_msg_in;
sensor_msgs__msg__JointState servo_position_array_msg;
rosidl_runtime_c__String__Sequence name_sequence;

#define NUM_JOINTS 6
int servoChannels[NUM_JOINTS] = {0, 1, 2, 3, 4, 5};

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

double pos[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
double vel[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
double effort[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};

#define LED_PIN 13
unsigned long last_publish_time = 0;
const unsigned long publish_interval = 20; // 50 Hz

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

int calculatePWM(double degrees) {
    const double min_us = 600.0;   // ~0°
    const double max_us = 2400.0;  // ~180°
    double pulse_us = min_us + (degrees / 180.0) * (max_us - min_us);
    return (int)pulse_us;
}

double roundToPrecision(double value, int decimals) {
  double scale = pow(10.0, decimals);
  return round(value * scale) / scale;
}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState*)msgin;

  if (msg == NULL || msg->position.size < NUM_JOINTS) {
    return;
  }
  
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    double rad = msg->position.data[i];
    double angle_deg = rad * 180.0 / M_PI;
    if (fabs(rad) < 1e-3) rad = 0.0;
    angle_deg = constrain(angle_deg, 0.0, 180.0);
    pos[i] = rad;   // keep 3 decimals
    servo_position_array_msg.position.data[i] = pos[i];
    int pulse = calculatePWM(angle_deg);
    pwm.writeMicroseconds(servoChannels[i], pulse);
  }
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher_servo_pos, &node);
  rcl_subscription_fini(&subscriber_joint_cmd, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  rosidl_runtime_c__String__Sequence__fini(&name_sequence);
  rosidl_runtime_c__String__Sequence__fini(&joint_cmd_msg_in.name);
  if (servo_position_array_msg.position.data) free(servo_position_array_msg.position.data);
  if (servo_position_array_msg.velocity.data) free(servo_position_array_msg.velocity.data);
  if (servo_position_array_msg.effort.data) free(servo_position_array_msg.effort.data);
  if (servo_position_array_msg.header.frame_id.data) free(servo_position_array_msg.header.frame_id.data);
  if (joint_cmd_msg_in.position.data) free(joint_cmd_msg_in.position.data);
  if (joint_cmd_msg_in.velocity.data) free(joint_cmd_msg_in.velocity.data);
  if (joint_cmd_msg_in.effort.data) free(joint_cmd_msg_in.effort.data);
  if (joint_cmd_msg_in.header.frame_id.data) free(joint_cmd_msg_in.header.frame_id.data);
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);
  
  const char* joint_names[NUM_JOINTS+1] = {"joint1", "joint2", "joint3", "joint4", "joint5", "left_finger_joint", "right_finger_joint"};

  if (!rosidl_runtime_c__String__Sequence__init(&name_sequence, NUM_JOINTS)) {
    error_loop();
  }
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    if (!rosidl_runtime_c__String__assignn(&name_sequence.data[i], joint_names[i], strlen(joint_names[i]))) {
      error_loop();
    }
  }

  joint_cmd_msg_in.position.capacity = NUM_JOINTS;
  joint_cmd_msg_in.position.size = NUM_JOINTS;
  joint_cmd_msg_in.position.data = (double*)malloc(NUM_JOINTS * sizeof(double));
  joint_cmd_msg_in.velocity.capacity = NUM_JOINTS;
  joint_cmd_msg_in.velocity.size = 0;
  joint_cmd_msg_in.velocity.data = (double*)malloc(NUM_JOINTS * sizeof(double));
  joint_cmd_msg_in.effort.capacity = NUM_JOINTS;
  joint_cmd_msg_in.effort.size = 0;
  joint_cmd_msg_in.effort.data = (double*)malloc(NUM_JOINTS * sizeof(double));
  rosidl_runtime_c__String__Sequence__init(&joint_cmd_msg_in.name, NUM_JOINTS+1);
  for (size_t i = 0; i < NUM_JOINTS+1; i++) {
    rosidl_runtime_c__String__assignn(&joint_cmd_msg_in.name.data[i], joint_names[i], strlen(joint_names[i]));
  }
  joint_cmd_msg_in.header.frame_id.capacity = 20;
  joint_cmd_msg_in.header.frame_id.data = (char*)malloc(20 * sizeof(char));
  strcpy(joint_cmd_msg_in.header.frame_id.data, "");
  joint_cmd_msg_in.header.frame_id.size = strlen(joint_cmd_msg_in.header.frame_id.data);

  servo_position_array_msg.position.capacity = NUM_JOINTS;
  servo_position_array_msg.position.size = NUM_JOINTS;
  servo_position_array_msg.position.data = (double*)malloc(NUM_JOINTS * sizeof(double));
  servo_position_array_msg.velocity.capacity = NUM_JOINTS;
  servo_position_array_msg.velocity.size = 0;
  servo_position_array_msg.velocity.data = (double*)malloc(NUM_JOINTS * sizeof(double));
  servo_position_array_msg.effort.capacity = NUM_JOINTS;
  servo_position_array_msg.effort.size = 0;
  servo_position_array_msg.effort.data = (double*)malloc(NUM_JOINTS * sizeof(double));
  servo_position_array_msg.name = name_sequence;
  servo_position_array_msg.header.frame_id.capacity = 20;
  servo_position_array_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
  strcpy(servo_position_array_msg.header.frame_id.data, "base_link");
  servo_position_array_msg.header.frame_id.size = strlen(servo_position_array_msg.header.frame_id.data);
  
  double initial_positions[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    pos[i] = initial_positions[i]; // radians
    servo_position_array_msg.position.data[i] = pos[i];
    double angle_deg = pos[i] * 180.0 / PI;
    int pulse = calculatePWM(angle_deg);
    pwm.writeMicroseconds(servoChannels[i], pulse);
    delay(10);
  }

  state = WAITING_AGENT;
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(&publisher_servo_pos, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/MyArmControl/joint_states"));
  RCCHECK(rclc_subscription_init_best_effort(&subscriber_joint_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/MyArmControl/joint_commands"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_joint_cmd, &joint_cmd_msg_in, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      break;
    case AGENT_AVAILABLE:
      state = AGENT_CONNECTED;
      break;
    case AGENT_CONNECTED:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      if (state == AGENT_CONNECTED) {
        unsigned long current_time = millis();
        if (current_time - last_publish_time >= publish_interval) {
          uint64_t ms = rmw_uros_epoch_millis();
          servo_position_array_msg.header.stamp.sec = (uint32_t)(ms / 1000);
          servo_position_array_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000);
          for (size_t i = 0; i < NUM_JOINTS; i++) {
            servo_position_array_msg.position.data[i] = pos[i];
          }
          RCSOFTCHECK(rcl_publish(&publisher_servo_pos, &servo_position_array_msg, NULL));
          last_publish_time = current_time;
        }
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));  
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  } 
  digitalWrite(LED_PIN, state == AGENT_CONNECTED ? HIGH : LOW);
}
