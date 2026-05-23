//*******************************************************************************//
//*     TITLE: Distance-based movement control                                  *//
//*     AUTHORS: Omar Emad & Mohamed Montasser                                  *//
//*     FINAL ODOM VERSION                                                      *//
//*******************************************************************************//

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

#include <WiFi.h>

//-------------------------------------------------------------------
// ROBOT CONSTANTS
//-------------------------------------------------------------------

const float WHEEL_RADIUS = 0.1;
const int PULSES_PER_REV = 600;

float GEAR_RATIO = (1885.0 / 600.0);

const int MOTOR_SPEED = 80;

//-------------------------------------------------------------------
// MOTOR PINS
//-------------------------------------------------------------------

const int M1_PWM = 32;
const int M1_DIR = 14;

const int M2_PWM = 22;
const int M2_DIR = 2;

//-------------------------------------------------------------------
// ENCODER PINS
//-------------------------------------------------------------------

const int ENCODER_A = 17;
const int ENCODER_B = 16;

//-------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------

// Used ONLY for distance movement logic
volatile long encoderCount = 0;

// Used for odometry publishing
volatile long encoderOdom = 0;

long targetPulses = 0;

bool motorRunning = false;

// direction for odom sign
int direction = 1;

// RPM calculation
unsigned long lastTime = 0;
long lastCount = 0;

//-------------------------------------------------------------------
// micro-ROS VARIABLES
//-------------------------------------------------------------------

rcl_subscription_t subscriber;
std_msgs__msg__Float32 recv_msg;

rcl_publisher_t encoder_pub;
std_msgs__msg__Float32 encoder_msg;

rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

//-------------------------------------------------------------------
// ENCODER ISR
//-------------------------------------------------------------------

void IRAM_ATTR encoderISR()
{
  encoderCount++;
  encoderOdom++;
}

//-------------------------------------------------------------------
// MOTOR CONTROL
//-------------------------------------------------------------------

void motorsStop()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void motorsForward()
{
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, HIGH);

  ledcWrite(0, MOTOR_SPEED);
  ledcWrite(1, MOTOR_SPEED);
}

void motorsBackward()
{
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, LOW);

  ledcWrite(0, MOTOR_SPEED);
  ledcWrite(1, MOTOR_SPEED);
}

//-------------------------------------------------------------------
// RPM CALCULATION
//-------------------------------------------------------------------

float calculateRPM()
{
  unsigned long currentTime = millis();

  unsigned long dt = currentTime - lastTime;

  if (dt >= 200)
  {
    long diff = encoderCount - lastCount;

    lastCount = encoderCount;
    lastTime = currentTime;

    float encoder_rps =
      (float)diff / PULSES_PER_REV;

    float wheel_rps =
      encoder_rps / GEAR_RATIO;

    return wheel_rps * 60.0;
  }

  return -1;
}

//-------------------------------------------------------------------
// micro-ROS CALLBACK
//-------------------------------------------------------------------

void cmd_callback(const void *msgin)
{
  if (motorRunning)
    return;

  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  float distanceMeters = msg->data;

  if (distanceMeters == 0)
    return;

  // Reset ONLY motion counter
  encoderCount = 0;

  float wheelRevs =
    distanceMeters /
    (2.0 * PI * WHEEL_RADIUS);

  float encoderRevsNeeded =
    wheelRevs * GEAR_RATIO;

  targetPulses =
    abs(encoderRevsNeeded * PULSES_PER_REV);

  if (distanceMeters > 0)
  {
    direction = 1;
    motorsForward();
  }
  else
  {
    direction = -1;
    motorsBackward();
  }

  motorRunning = true;

  lastTime = millis();
  lastCount = 0;
}

//-------------------------------------------------------------------
// SETUP
//-------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  delay(2000);

  Serial.println("Boot OK");

  set_microros_wifi_transports(
    "EME-SIEMENS_GPS",
    "gps12345",
    "192.168.0.107",
    8888
  );

  Serial.println("WiFi transport configured");

  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  // PWM setup
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);

  ledcAttachPin(M1_PWM, 0);
  ledcAttachPin(M2_PWM, 1);

  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Single-channel encoder mode
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_B),
    encoderISR,
    RISING
  );

  delay(2000);

  // micro-ROS init
  allocator = rcl_get_default_allocator();

  rclc_support_init(
    &support,
    0,
    NULL,
    &allocator
  );

  rclc_node_init_default(
    &node,
    "robot_node",
    "",
    &support
  );

  // Subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs,
      msg,
      Float32
    ),
    "/move_distance"
  );

  // Publisher
  rclc_publisher_init_default(
    &encoder_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs,
      msg,
      Float32
    ),
    "/encoder_count"
  );

  // Executor
  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator
  );

  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &recv_msg,
    &cmd_callback,
    ON_NEW_DATA
  );
}

//-------------------------------------------------------------------
// LOOP
//-------------------------------------------------------------------

void loop()
{
  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(10)
  );

  // Continuous odom publishing
  encoder_msg.data =
    encoderOdom * direction;

  rcl_publish(
    &encoder_pub,
    &encoder_msg,
    NULL
  );

  // ---------------- MOTOR RUNNING ----------------

  if (motorRunning)
  {
    float rpm = calculateRPM();

    if (rpm >= 0)
    {
      Serial.print("Encoder Count = ");
      Serial.print(encoderCount);

      Serial.print(" | Wheel RPM = ");
      Serial.println(rpm);
    }

    // Stop when target reached
    if (abs(encoderCount) >= targetPulses)
    {
      motorsStop();

      motorRunning = false;

      float wheel_revs_real =
        (float)encoderCount /
        (PULSES_PER_REV * GEAR_RATIO);

      float distance =
        wheel_revs_real *
        (2.0 * PI * WHEEL_RADIUS);

      Serial.println("---- Movement Completed ----");

      Serial.print("Final encoder pulses: ");
      Serial.println(encoderCount);

      Serial.print("Distance moved (m): ");
      Serial.println(distance, 4);
    }
  }
}
