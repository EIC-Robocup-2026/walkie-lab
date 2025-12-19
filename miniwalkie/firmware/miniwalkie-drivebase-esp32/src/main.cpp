#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include "motor.h"
#include "wheel_feedback.h"
#include "motor_controller.h"
#include "buzzer.h"
#include "pitches.h"

// micro-ROS includes
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <micro_ros_utilities/string_utilities.h>

// ROS2 handles
rclc_executor_t executor;
rclc_executor_t executor_cmd_vel_sub;
rclc_executor_t executor_joint_state_pub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;

// Publishers and subscribers
rcl_publisher_t joint_state_publisher;
rcl_subscription_t velocity_command_subscriber;

// Messages
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__JointState velocity_command_msg;

// Message memory
#define JOINT_COUNT 4
double joint_positions[JOINT_COUNT];  // Change to double
double joint_velocities[JOINT_COUNT]; // Change to double
const char* joint_names[JOINT_COUNT] = {  // Change to const char*
    "fl_wheel_joint",
    "fr_wheel_joint",
    "br_wheel_joint",
    "bl_wheel_joint"
};

// ********* Encoder ***********
ESP32Encoder encoder_1;
ESP32Encoder encoder_2;
ESP32Encoder encoder_3;
ESP32Encoder encoder_4;

// Encoder pins
#define ENCODER_1_PIN1 12
#define ENCODER_1_PIN2 26

#define ENCODER_2_PIN1 27
#define ENCODER_2_PIN2 14

#define ENCODER_3_PIN1 17
#define ENCODER_3_PIN2 16

#define ENCODER_4_PIN1 19
#define ENCODER_4_PIN2 18

// Create wheel feedback objects
WheelFeedback wheel_1(encoder_1, 1228.8);
WheelFeedback wheel_2(encoder_2, 1228.8);
WheelFeedback wheel_3(encoder_3, 1228.8);
WheelFeedback wheel_4(encoder_4, 1228.8);

// ********* Smile Drive communication *************
#define MOTOR_SDA 21
#define MOTOR_SCL 22

// Smile Drive M1 M2 Address
#define FIRST_I2C_ADDRESS 0x85
#define SECOND_I2C_ADDRESS 0x86

#define FRONT_MDrive_I2C_ADDRESS 0x50
#define BACK_MDrive_I2C_ADDRESS 0x51
MotorI2C Motor_1(FRONT_MDrive_I2C_ADDRESS, SECOND_I2C_ADDRESS);
MotorI2C Motor_2(BACK_MDrive_I2C_ADDRESS, FIRST_I2C_ADDRESS); 
MotorI2C Motor_3(BACK_MDrive_I2C_ADDRESS, SECOND_I2C_ADDRESS);
MotorI2C Motor_4(FRONT_MDrive_I2C_ADDRESS, FIRST_I2C_ADDRESS);

// Create motor controllers
MotorSpeedController controller_fl(Motor_1, wheel_1);
MotorSpeedController controller_fr(Motor_2, wheel_2);
MotorSpeedController controller_br(Motor_3, wheel_3);
MotorSpeedController controller_bl(Motor_4, wheel_4);

// Task for motor control
TaskHandle_t motorControlTaskHandle = NULL;

void motor_control_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms, 100Hz

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Update all controllers
        controller_fl.update();
        controller_fr.update();
        controller_br.update();
        controller_bl.update();

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


#define ROS_LED_PIN 2  // You can change this to any GPIO pin you want to use
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Add state enum and variable
enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

Buzzer buzzer(4); // Using pin 4 for the buzzer
#define BOOT_MUSIC false // Set to false to disable boot music

void velocity_command_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    
    // Only check for velocity data presence, ignore timestamp and other fields
    if (msg->velocity.data != NULL && msg->velocity.size >= JOINT_COUNT) {
        // Map velocities regardless of timestamp
        float fl_vel = msg->velocity.data[0];
        float fr_vel = msg->velocity.data[1];
        float br_vel = msg->velocity.data[2];
        float bl_vel = msg->velocity.data[3];

        // Set motor speeds directly
        controller_fl.setTargetSpeed(fl_vel);
        controller_fr.setTargetSpeed(fr_vel);
        controller_br.setTargetSpeed(br_vel);
        controller_bl.setTargetSpeed(bl_vel);
    }
}

void stop_motors() {
    controller_fl.setTargetSpeed(0);
    controller_fr.setTargetSpeed(0);
    controller_br.setTargetSpeed(0);
    controller_bl.setTargetSpeed(0);
}

bool init_messages() {
    // Initialize and set frame_id
    rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "base_link");

    // Initialize position array
    joint_state_msg.position.capacity = JOINT_COUNT;
    joint_state_msg.position.size = JOINT_COUNT;
    joint_state_msg.position.data = joint_positions;

    // Initialize velocity array
    joint_state_msg.velocity.capacity = JOINT_COUNT;
    joint_state_msg.velocity.size = JOINT_COUNT;
    joint_state_msg.velocity.data = joint_velocities;

    // Initialize effort array (empty but initialize it)
    joint_state_msg.effort.capacity = 0;
    joint_state_msg.effort.size = 0;
    joint_state_msg.effort.data = NULL;

    // Initialize name array
    joint_state_msg.name.capacity = JOINT_COUNT;
    joint_state_msg.name.size = JOINT_COUNT;
    joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));
    
    for (size_t i = 0; i < JOINT_COUNT; i++) {
        rosidl_runtime_c__String__init(&joint_state_msg.name.data[i]);
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[i], joint_names[i]);
    }

    // Initialize velocity command message
    rosidl_runtime_c__String__init(&velocity_command_msg.header.frame_id);
    rosidl_runtime_c__String__assign(&velocity_command_msg.header.frame_id, "");

    velocity_command_msg.velocity.capacity = JOINT_COUNT;
    velocity_command_msg.velocity.size = JOINT_COUNT;
    velocity_command_msg.velocity.data = (double*)malloc(JOINT_COUNT * sizeof(double));

    velocity_command_msg.position.capacity = 0;
    velocity_command_msg.position.size = 0;
    velocity_command_msg.position.data = NULL;

    velocity_command_msg.effort.capacity = 0;
    velocity_command_msg.effort.size = 0;
    velocity_command_msg.effort.data = NULL;

    velocity_command_msg.name.capacity = JOINT_COUNT;
    velocity_command_msg.name.size = JOINT_COUNT;
    velocity_command_msg.name.data = (rosidl_runtime_c__String*)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));

    for (size_t i = 0; i < JOINT_COUNT; i++) {
        rosidl_runtime_c__String__init(&velocity_command_msg.name.data[i]);
        rosidl_runtime_c__String__assign(&velocity_command_msg.name.data[i], joint_names[i]);
    }

    return true;
}

bool create_entities() {
    allocator = rcl_get_default_allocator();
    
    if(init_messages() == false){
        return false;
    }

    // Create init_options and set domain ID
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 23));
    
    // Initialize support with custom options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    // Create node
    RCCHECK(rclc_node_init_default(&node, "drivebase_node", "", &support));

    // Initialize publishers and subscribers
    RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "drivebase_joint_states"));

    RCCHECK(rclc_subscription_init_default(
        &velocity_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "drivebase_joint_cmd"));

    // Create executors
    RCCHECK(rclc_executor_init(&executor_cmd_vel_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_init(&executor_joint_state_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_cmd_vel_sub, &velocity_command_subscriber, 
                                         &velocity_command_msg, &velocity_command_callback,
                                         ON_NEW_DATA));
    return true;
}

bool destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_ret_t rc = RCL_RET_OK;

    rc += rclc_executor_fini(&executor_cmd_vel_sub);
    rc += rclc_executor_fini(&executor_joint_state_pub);
    rc += rcl_publisher_fini(&joint_state_publisher, &node);
    rc += rcl_subscription_fini(&velocity_command_subscriber, &node);
    rc += rcl_node_fini(&node);
    rc += rclc_support_fini(&support);
    rc += rcl_init_options_fini(&init_options);

    return (rc == RCL_RET_OK);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(MOTOR_SDA, MOTOR_SCL);

    // Play boot music
    if(BOOT_MUSIC){
        buzzer.playBootMusic();
    }
    buzzer.playNote(NOTE_G5, 2); // Short pause after boot music

    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Setup encoders using the defined pins
    encoder_1.attachHalfQuad(ENCODER_1_PIN1, ENCODER_1_PIN2);
    encoder_2.attachHalfQuad(ENCODER_2_PIN1, ENCODER_2_PIN2);
    encoder_3.attachHalfQuad(ENCODER_3_PIN1, ENCODER_3_PIN2);
    encoder_4.attachHalfQuad(ENCODER_4_PIN1, ENCODER_4_PIN2);

    // Configure initial PID gains
    controller_fl.setPIDGains(0.05, 70.0, 0.07);
    controller_fl.setFeedForward(14.8311, 16.6545); 
    controller_fr.setPIDGains(0.05, 70.0, 0.07);
    controller_fr.setFeedForward(14.8311, 16.6545); 
    controller_br.setPIDGains(0.05, 70.0, 0.07);
    controller_br.setFeedForward(14.8311, 16.6545); 
    controller_bl.setPIDGains(0.05, 70.0, 0.07);
    controller_bl.setFeedForward(14.8311, 16.6545); 

    // Set output limits
    controller_fl.setOutputLimits(-255, 255);
    controller_fr.setOutputLimits(-255, 255);
    controller_br.setOutputLimits(-255, 255);
    controller_bl.setOutputLimits(-255, 255);

    // Add ROS connection LED
    pinMode(ROS_LED_PIN, OUTPUT);
    digitalWrite(ROS_LED_PIN, LOW);

    // Initialize micro-ROS transport
    set_microros_serial_transports(Serial);
    init_messages();

    // Set initial state
    state = WAITING_AGENT;

    // Create motor control task
    xTaskCreatePinnedToCore(
        motor_control_task,     /* Task function. */
        "MotorControlTask",     /* name of task. */
        4096,                   /* Stack size of task */
        NULL,                   /* parameter of the task */
        1,                      /* priority of the task */
        &motorControlTaskHandle,/* Task handle to keep track of created task */
        1);                     /* pin task to core 1 */
}

void loop() {
    switch(state) {
        case WAITING_AGENT:
            // Check for agent connection
            state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            break;
        
        case AGENT_AVAILABLE:
            // Create micro-ROS entities
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            } else {
                // Play connected sound when successfully connected
                buzzer.playConnectedSound();
            }
            break;
        
        case AGENT_CONNECTED:
            // Check connection and run normal operation
            state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor_cmd_vel_sub, RCL_MS_TO_NS(10));

                // Update timestamp
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                joint_state_msg.header.stamp.sec = ts.tv_sec;
                joint_state_msg.header.stamp.nanosec = ts.tv_nsec;

                joint_positions[0] = (double)wheel_1.getAngularPosition();
                joint_positions[1] = (double)wheel_2.getAngularPosition();
                joint_positions[2] = (double)wheel_3.getAngularPosition();
                joint_positions[3] = (double)wheel_4.getAngularPosition();

                joint_velocities[0] = (double)wheel_1.getAngularVelocity();
                joint_velocities[1] = (double)wheel_2.getAngularVelocity();
                joint_velocities[2] = (double)wheel_3.getAngularVelocity();
                joint_velocities[3] = (double)wheel_4.getAngularVelocity();

                RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

                // Handle ROS communications
                rclc_executor_spin_some(&executor_joint_state_pub, RCL_MS_TO_NS(10));
            }
            break;

        case AGENT_DISCONNECTED:
            // Play disconnected sound
            buzzer.playDisconnectedSound();
            // Stop motors for safety
            stop_motors();
            
            // Cleanup and restart
            destroy_entities();
            state = WAITING_AGENT;
            break;

        default:
            break;
    }

    // Update connection status LED
    digitalWrite(ROS_LED_PIN, state == AGENT_CONNECTED);
}