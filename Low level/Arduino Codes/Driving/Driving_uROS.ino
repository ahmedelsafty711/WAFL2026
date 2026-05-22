//*******************************************************************************//
//*     TITLE: Distance-based movement control                                  *//
//*     AUTHORS: Omar Emad & Mohamed Montasser                                  *//
//*     DATE: 8/DEC/2025                                                        *//
//*     DESCRIPTION:                                                            *//
//*     Integrating micro-ros to connect the esp as a node subscribeng to       *//
//*     a topic via Wi-Fi                                                       *//
//*******************************************************************************//


#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

//-------------------------------------------------------------------
// USER CONSTANTS (MATCH NORMAL CODE)
//-------------------------------------------------------------------

const float WHEEL_RADIUS = 0.1;
const int PULSES_PER_REV = 600;
float GEAR_RATIO = (1885.0 / 600.0);
const int MOTOR_SPEED = 80;

// Motor pins (UPDATED)
const int M1_PWM = 32;
const int M1_DIR = 14;
const int M2_PWM = 22;
const int M2_DIR = 2;

// Encoder pins (UPDATED)
const int ENCODER_A = 17;
const int ENCODER_B = 16;

//-------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------

volatile long encoderCount = 0;
long targetPulses = 0;
bool motorRunning = false;

unsigned long lastTime = 0;
long lastCount = 0;

//-------------------------------------------------------------------
// micro-ROS variables
//-------------------------------------------------------------------

rcl_subscription_t subscriber;
std_msgs__msg__Float32 recv_msg;

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

//-------------------------------------------------------------------
// ENCODER ISR (UNCHANGED)
//-------------------------------------------------------------------

void IRAM_ATTR encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  if (a == b) encoderCount++;
  else encoderCount--;
}

//-------------------------------------------------------------------
// MOTOR CONTROL (MATCH NORMAL LOGIC)
//-------------------------------------------------------------------

void motorsStop() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void motorsForward() {
  digitalWrite(M1_DIR, LOW);    // SAME AS NORMAL
  digitalWrite(M2_DIR, HIGH);

  ledcWrite(0, MOTOR_SPEED);
  ledcWrite(1, MOTOR_SPEED);
}

void motorsBackward() {
  digitalWrite(M1_DIR, HIGH);   // SAME AS NORMAL
  digitalWrite(M2_DIR, LOW);

  ledcWrite(0, MOTOR_SPEED);
  ledcWrite(1, MOTOR_SPEED);
}

//-------------------------------------------------------------------
// RPM CALCULATION (UNCHANGED)
//-------------------------------------------------------------------

float calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastTime;

  if (dt >= 200) {
    long diff = encoderCount - lastCount;
    lastCount = encoderCount;
    lastTime = currentTime;

    float encoder_rps = (float)diff / PULSES_PER_REV;
    float wheel_rps = encoder_rps / GEAR_RATIO;

    return wheel_rps * 60.0;
  }
  return -1;
}

//-------------------------------------------------------------------
// micro-ROS Subscription Callback
//-------------------------------------------------------------------

void cmd_callback(const void *msgin) {
  if (motorRunning) return;

  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  float distanceMeters = msg->data;

  if (distanceMeters == 0) return;

  encoderCount = 0;

  float wheelRevs = distanceMeters / (2.0 * PI * WHEEL_RADIUS);
  float encoderRevsNeeded = wheelRevs * GEAR_RATIO;

  targetPulses = abs(encoderRevsNeeded * PULSES_PER_REV);

  if (distanceMeters > 0)
    motorsForward();
  else
    motorsBackward();

  motorRunning = true;
  lastTime = millis();
  lastCount = 0;
}

//-------------------------------------------------------------------
// SETUP
//-------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  set_microros_wifi_transports(
    "EME-SIEMENS_GPS",
    "gps12345",  // <-- your Wi-Fi password
    "192.168.0.107",       // <-- your PC IP where micro-ROS agent runs
    8888                   // <-- port (keep same for all ESPs)
);

  // PWM setup
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(M1_PWM, 0);
  ledcAttachPin(M2_PWM, 1);

  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, RISING);

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "robot_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/move_distance");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &recv_msg,
    &cmd_callback,
    ON_NEW_DATA);
}

//-------------------------------------------------------------------
// LOOP
//-------------------------------------------------------------------

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (motorRunning) {
    float rpm = calculateRPM();

    if (rpm >= 0) {
      Serial.print("Encoder Count = ");
      Serial.print(encoderCount);
      Serial.print(" | Wheel RPM = ");
      Serial.println(rpm);
    }

    if (abs(encoderCount) >= targetPulses) {
      motorsStop();
      motorRunning = false;

      float wheel_revs_real =
        (float)encoderCount / (PULSES_PER_REV * GEAR_RATIO);

      float distance = wheel_revs_real * (2.0 * PI * WHEEL_RADIUS);

      Serial.println("---- Movement Completed ----");
      Serial.print("Final encoder pulses: ");
      Serial.println(encoderCount);
      Serial.print("Distance moved (m): ");
      Serial.println(distance, 4);
    }
  }
}
