//*********************************************************************************//
//* TITLE: Closed-Loop Traction with Multi-Topic Feedback Publishing              *//
//* AUTHORS: Omar Emad & Mohamed Montasser                                        *//
//* DATE: 07/JUN/2026                                                             *//
//*********************************************************************************//

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// =================================================
// PIN CONFIGURATION
// =================================================
#define DC_ENC_A    15
#define DC_ENC_B    2
#define MOTOR_PWM   13
#define MOTOR_DIR   14

// =================================================
// CONSTANTS & CALIBRATION
// =================================================
const float DC_PPR = 23700.0; // Total pulses per full 360 rotation

// 25 degrees converted directly to absolute encoder ticks
const long ERROR_THRESHOLD_TICKS = (long)((25.0 / 360.0) * DC_PPR); 
const long RECOVERY_DEADBAND_TICKS = 80; 
const int CORRECTION_SPEED_PWM = 130; 

// Publishing rate control (10 Hz = every 100ms) to avoid flooding the network
const unsigned long PUBLISH_INTERVAL_MS = 100;
unsigned long lastPublishTime = 0;

// =================================================
// micro-ROS objects
// =================================================
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// Message & Publisher for tracking target inputs
std_msgs__msg__Float32 traction_msg;

// NEW: Publishers & Messages for system feedback
rcl_publisher_t angle_pub;
std_msgs__msg__Float32 absolute_angle_msg;

rcl_publisher_t encoder_pub;
std_msgs__msg__Float32 encoder_count_msg;

// =================================================
// DC MOTOR VARIABLES
// =================================================
volatile long dcEncoderCount = 0;
long dc_targetPulse = 0;       // Keeps tracking target internally in hardware ticks
bool correction_active = false;

// =================================================
// DC MOTOR ENCODER ISR
// =================================================
void IRAM_ATTR dcEncoderISR() {
  int b = digitalRead(DC_ENC_B);
  if (b == HIGH) dcEncoderCount++;
  else dcEncoderCount--;
}

// =================================================
// ROS CALLBACK (Converts Target Degrees to Ticks)
// =================================================
void traction_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  
  // Input arrives in degrees. Convert it to target encoder ticks:
  // Ticks = (Target Degrees / 360.0) * Total PPR
  float target_degrees = msg->data;
  dc_targetPulse = (long)((target_degrees / 360.0) * DC_PPR);
}

// =================================================
// SETUP
// =================================================
void setup()
{
  Serial.begin(115200);

  // DC Motor Configurations
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(DC_ENC_A, INPUT_PULLUP);
  pinMode(DC_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DC_ENC_A), dcEncoderISR, RISING);

  // micro-ROS WiFi Transports
  set_microros_wifi_transports(
    "EME-SIEMENS_GPS",
    "gps12345",
    "192.168.0.126",
    8888
  );

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "traction_node", "", &support);

  // 1. Initialize Subscriber with the updated topic name
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/target_angle" 
  );

  // 2. Initialize Absolute Angle Publisher (NEW)
  rclc_publisher_init_default(
    &angle_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/absolute_angle"
  );

  // 3. Initialize Raw Encoder Count Publisher (NEW)
  rclc_publisher_init_default(
    &encoder_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/caster_encoder_count"
  );

  // 4. Initialize Executor (Handles 1 subscription context)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &subscriber, &traction_msg,
    &traction_callback, ON_NEW_DATA
  );
}

// =================================================
// LOOP
// =================================================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  // =================================================
  // CONTINUOUS DISTURBANCE REJECTION ALGORITHM
  // =================================================
  long current_error = dc_targetPulse - dcEncoderCount;

  if (!correction_active) {
    if (abs(current_error) > ERROR_THRESHOLD_TICKS) {
      correction_active = true;
    }
  }

  if (correction_active) {
    if (current_error > 0) digitalWrite(MOTOR_DIR, HIGH);
    else digitalWrite(MOTOR_DIR, LOW);
    
    analogWrite(MOTOR_PWM, CORRECTION_SPEED_PWM);

    if (abs(current_error) <= RECOVERY_DEADBAND_TICKS) {
      analogWrite(MOTOR_PWM, 0); 
      correction_active = false;
    }
  }

  // =================================================
  // TIMED TELEMETRY PUBLISHING (10 Hz Loop)
  // =================================================
  unsigned long currentTime = millis();
  if (currentTime - lastPublishTime >= PUBLISH_INTERVAL_MS) {
    lastPublishTime = currentTime;

    // A. Read volatile counts atomically to prevent interrupt corruption
    long stable_counts = dcEncoderCount;

    // B. Calculate absolute angle in degrees: (counts / total_ppr) * 360
    float absolute_angle = ((float)stable_counts / DC_PPR) * 360.0;

    // C. Pack and Publish Absolute Angle
    absolute_angle_msg.data = absolute_angle;
    rcl_publish(&angle_pub, &absolute_angle_msg, NULL);

    // D. Pack and Publish Caster Encoder Count
    encoder_count_msg.data = (float)stable_counts; 
    rcl_publish(&encoder_pub, &encoder_count_msg, NULL);
  }
}
