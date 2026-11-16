#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6500-DMP.h>
#include <Wire.h>
#include <WiFi.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "secrets.h"

namespace {

constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 32;
constexpr uint8_t kOledAddress = 0x3C;
constexpr uint8_t kLeftXshutPin = 4;
constexpr uint8_t kCenterXshutPin = 16;
constexpr uint8_t kRightXshutPin = 17;
constexpr uint8_t kLeftTofAddress = 0x30;
constexpr uint8_t kCenterTofAddress = 0x31;
constexpr uint8_t kRightTofAddress = 0x32;
constexpr uint8_t kLeftEncoderA = 34;
constexpr uint8_t kLeftEncoderB = 35;
constexpr uint8_t kRightEncoderA = 32;
constexpr uint8_t kRightEncoderB = 33;
constexpr uint8_t kLeftMotorIn1 = 26;
constexpr uint8_t kLeftMotorIn2 = 25;
constexpr uint8_t kRightMotorIn1 = 18;
constexpr uint8_t kRightMotorIn2 = 19;
constexpr uint8_t kLeftPwmChannel1 = 0;
constexpr uint8_t kLeftPwmChannel2 = 1;
constexpr uint8_t kRightPwmChannel1 = 2;
constexpr uint8_t kRightPwmChannel2 = 3;
constexpr uint32_t kPwmFrequency = 30000;
constexpr uint8_t kPwmResolution = 8;
constexpr float kEncoderPulsesPerMotorRevolution = 11.0f;
constexpr float kReductionRatio = 30.0f;
constexpr float kRadiansToRpm = 60.0f / (2.0f * PI);
constexpr float kMinimumSetpointRadS = 0.6f;
constexpr float kMinimumSetpointRpm =
    kMinimumSetpointRadS * kRadiansToRpm;
constexpr float kGravityMetersPerSecondSquared = 9.80665f;
constexpr float kDegreesToRadians = PI / 180.0f;
constexpr uint32_t kEncoderIntervalMs = 100;
constexpr uint32_t kPidIntervalMs = kEncoderIntervalMs;
constexpr uint32_t kSensorPublishIntervalMs = 50;
constexpr uint32_t kOledIntervalMs = 200;
constexpr uint32_t kTofRetryIntervalMs = 2000;
constexpr uint32_t kCommandTimeoutMs = 1000;
constexpr uint16_t kMaximumTofDistanceMm = 700;
constexpr int kPwmMaximum = 255;

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, -1);
MPU6500_DMP imu;
Adafruit_VL53L0X leftTof;
Adafruit_VL53L0X centerTof;
Adafruit_VL53L0X rightTof;
bool displayReady = false;
String displayLogLines[4];
uint32_t lastDisplayLogMs = 0;

struct MotorControl {
  uint8_t encoderA;
  uint8_t encoderB;
  uint8_t pwmChannel1;
  uint8_t pwmChannel2;
  MotorControl(
      uint8_t encoderPinA, uint8_t encoderPinB, uint8_t pwm1, uint8_t pwm2)
      : encoderA(encoderPinA),
        encoderB(encoderPinB),
        pwmChannel1(pwm1),
        pwmChannel2(pwm2) {}
  float measuredRpm = 0.0f;
  float setpointRpm = 0.0f;
  float outputPwm = 0.0f;
  float error = 0.0f;
  float previousMeasuredRpm = 0.0f;
  float integral = 0.0f;
  uint8_t reversalCount = 0;
};

MotorControl leftMotor{
    kLeftEncoderA, kLeftEncoderB, kLeftPwmChannel1, kLeftPwmChannel2};
MotorControl rightMotor{
    kRightEncoderA, kRightEncoderB, kRightPwmChannel1, kRightPwmChannel2};

volatile int32_t leftEncoderTicks = 0;
volatile int32_t rightEncoderTicks = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

struct PidGains {
  float kp;
  float ki;
  float kd;
};

constexpr PidGains kLeftPidGains{1.7f, 1.2f, 0.02f};
constexpr PidGains kRightPidGains{1.8f, 1.2f, 0.02f};

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t wheelVelocityPublisher;
rcl_publisher_t imuPublisher;
rcl_publisher_t tofPublisher;
rcl_subscription_t setpointSubscription;
rclc_executor_t executor;
std_msgs__msg__Float32MultiArray wheelVelocityMessage;
std_msgs__msg__Float32MultiArray imuMessage;
std_msgs__msg__Int32MultiArray tofMessage;
std_msgs__msg__Float32MultiArray setpointMessage;
float wheelVelocityValues[2] = {0.0f, 0.0f};
float imuValues[3] = {0.0f, 0.0f, 0.0f};
float setpointValues[2] = {0.0f, 0.0f};
int32_t tofValues[3] = {0, 0, 0};

bool imuReady = false;
bool tofReady = false;
float gyroZDegreesPerSecond = 0.0f;
float accelXG = 0.0f;
float accelYG = 0.0f;
float gyroZOffset = 0.0f;
float accelXOffset = 0.0f;
float accelYOffset = 0.0f;
uint32_t lastSetpointMs = 0;
uint32_t lastEncoderMs = 0;
uint32_t lastPidMs = 0;
uint32_t lastPublishMs = 0;
uint32_t lastTofRetryMs = 0;
uint32_t lastOledMs = 0;
bool wheelSamplePending = false;

void renderDisplayLog() {
  if (!displayReady) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for (size_t i = 0; i < 4; ++i) {
    display.setCursor(0, i * 8);
    display.print(displayLogLines[i]);
  }
  display.display();
}

void appendDisplayLogLine(const String &line) {
  for (size_t i = 0; i < 3; ++i) {
    displayLogLines[i] = displayLogLines[i + 1];
  }
  displayLogLines[3] = line;
}

void logMessage(const String &message) {
  Serial.println(message);
  lastDisplayLogMs = millis();

  String remaining = message;
  while (remaining.length() > 0) {
    if (remaining.length() <= 21) {
      appendDisplayLogLine(remaining);
      remaining = "";
      continue;
    }

    int splitAt = 21;
    const int spaceAt = remaining.lastIndexOf(' ', splitAt);
    if (spaceAt > 0) {
      splitAt = spaceAt;
    }
    appendDisplayLogLine(remaining.substring(0, splitAt));
    remaining = remaining.substring(splitAt);
    remaining.trim();
  }
  if (message.length() == 0) {
    appendDisplayLogLine("");
  }
  renderDisplayLog();
}

[[noreturn]] void stopOnError(const char *operation, rcl_ret_t error) {
  logMessage(
      String("Error micro-ROS en ") + operation + ": " +
      String(static_cast<int>(error)));
  leftMotor.setpointRpm = 0.0f;
  rightMotor.setpointRpm = 0.0f;
  ledcWrite(kLeftPwmChannel1, 0);
  ledcWrite(kLeftPwmChannel2, 0);
  ledcWrite(kRightPwmChannel1, 0);
  ledcWrite(kRightPwmChannel2, 0);
  while (true) {
    delay(1000);
  }
}

void checkRcl(rcl_ret_t result, const char *operation) {
  if (result != RCL_RET_OK) {
    stopOnError(operation, result);
  }
}

void IRAM_ATTR onLeftEncoder() {
  const bool channelBHigh = digitalRead(kLeftEncoderB) == HIGH;
  portENTER_CRITICAL_ISR(&encoderMux);
  leftEncoderTicks += channelBHigh ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onRightEncoder() {
  const bool channelBLow = digitalRead(kRightEncoderB) == LOW;
  portENTER_CRITICAL_ISR(&encoderMux);
  rightEncoderTicks += channelBLow ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void initializeMotors() {
  ledcSetup(kLeftPwmChannel1, kPwmFrequency, kPwmResolution);
  ledcSetup(kLeftPwmChannel2, kPwmFrequency, kPwmResolution);
  ledcSetup(kRightPwmChannel1, kPwmFrequency, kPwmResolution);
  ledcSetup(kRightPwmChannel2, kPwmFrequency, kPwmResolution);
  ledcAttachPin(kLeftMotorIn1, kLeftPwmChannel1);
  ledcAttachPin(kLeftMotorIn2, kLeftPwmChannel2);
  ledcAttachPin(kRightMotorIn1, kRightPwmChannel1);
  ledcAttachPin(kRightMotorIn2, kRightPwmChannel2);
  ledcWrite(kLeftPwmChannel1, 0);
  ledcWrite(kLeftPwmChannel2, 0);
  ledcWrite(kRightPwmChannel1, 0);
  ledcWrite(kRightPwmChannel2, 0);

  pinMode(leftMotor.encoderA, INPUT_PULLUP);
  pinMode(leftMotor.encoderB, INPUT_PULLUP);
  pinMode(rightMotor.encoderA, INPUT_PULLUP);
  pinMode(rightMotor.encoderB, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(leftMotor.encoderA), onLeftEncoder, RISING);
  attachInterrupt(
      digitalPinToInterrupt(rightMotor.encoderA), onRightEncoder, RISING);
}

void updateMotorPid(
    MotorControl &motor, float dtSeconds, const PidGains &gains) {
  motor.error = motor.setpointRpm - motor.measuredRpm;
  if (std::fabs(motor.setpointRpm) < kMinimumSetpointRpm) {
    motor.integral = 0.0f;
    motor.previousMeasuredRpm = motor.measuredRpm;
    motor.outputPwm = 0.0f;
    ledcWrite(motor.pwmChannel1, 0);
    ledcWrite(motor.pwmChannel2, 0);
    return;
  }

  const bool reversing =
      (motor.setpointRpm > 40.0f && motor.measuredRpm < -10.0f) ||
      (motor.setpointRpm < -40.0f && motor.measuredRpm > 10.0f);
  if (reversing) {
    ++motor.reversalCount;
    if (motor.reversalCount >= 3) {
      motor.integral = 0.0f;
      motor.previousMeasuredRpm = motor.measuredRpm;
      motor.reversalCount = 0;
    }
  } else {
    motor.reversalCount = 0;
  }

  const float integralLimit = kPwmMaximum / gains.ki;
  const float proposedIntegral = constrain(
      motor.integral + motor.error * dtSeconds,
      -integralLimit, integralLimit);
  const float derivative =
      -(motor.measuredRpm - motor.previousMeasuredRpm) / dtSeconds;
  const float proposedOutput =
      gains.kp * motor.error + gains.ki * proposedIntegral +
      gains.kd * derivative;
  const bool integratingFurtherIntoSaturation =
      (proposedOutput > kPwmMaximum && motor.error > 0.0f) ||
      (proposedOutput < -kPwmMaximum && motor.error < 0.0f);
  if (!integratingFurtherIntoSaturation) {
    motor.integral = proposedIntegral;
  }
  motor.outputPwm =
      gains.kp * motor.error + gains.ki * motor.integral +
      gains.kd * derivative;
  motor.outputPwm = constrain(
      motor.outputPwm, -static_cast<float>(kPwmMaximum),
      static_cast<float>(kPwmMaximum));

  if (motor.outputPwm >= 0.0f) {
    ledcWrite(motor.pwmChannel1, static_cast<int>(motor.outputPwm));
    ledcWrite(motor.pwmChannel2, 0);
  } else {
    ledcWrite(motor.pwmChannel1, 0);
    ledcWrite(motor.pwmChannel2, static_cast<int>(-motor.outputPwm));
  }
  motor.previousMeasuredRpm = motor.measuredRpm;
}

bool updateEncoderSpeeds(uint32_t nowMs) {
  if (nowMs - lastEncoderMs < kEncoderIntervalMs) {
    return false;
  }

  const uint32_t elapsedMs = nowMs - lastEncoderMs;
  lastEncoderMs = nowMs;
  int32_t leftTicks;
  int32_t rightTicks;
  portENTER_CRITICAL(&encoderMux);
  leftTicks = leftEncoderTicks;
  rightTicks = rightEncoderTicks;
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
  portEXIT_CRITICAL(&encoderMux);

  const float ticksPerWheelRevolution =
      kEncoderPulsesPerMotorRevolution * kReductionRatio;
  const float minutes = elapsedMs / 60000.0f;
  leftMotor.measuredRpm = leftTicks / ticksPerWheelRevolution / minutes;
  rightMotor.measuredRpm = rightTicks / ticksPerWheelRevolution / minutes;
  return true;
}

void updatePid(uint32_t nowMs) {
  if (nowMs - lastPidMs < kPidIntervalMs) {
    return;
  }

  const uint32_t elapsedMs = nowMs - lastPidMs;
  lastPidMs = nowMs;
  if (nowMs - lastSetpointMs > kCommandTimeoutMs) {
    leftMotor.setpointRpm = 0.0f;
    rightMotor.setpointRpm = 0.0f;
  }

  const float dtSeconds = elapsedMs / 1000.0f;
  updateMotorPid(leftMotor, dtSeconds, kLeftPidGains);
  updateMotorPid(rightMotor, dtSeconds, kRightPidGains);
}

bool initializeTofSensors() {
  pinMode(kLeftXshutPin, OUTPUT);
  pinMode(kCenterXshutPin, OUTPUT);
  pinMode(kRightXshutPin, OUTPUT);
  digitalWrite(kLeftXshutPin, LOW);
  digitalWrite(kCenterXshutPin, LOW);
  digitalWrite(kRightXshutPin, LOW);
  delay(10);

  digitalWrite(kLeftXshutPin, HIGH);
  delay(10);
  if (!leftTof.begin(kLeftTofAddress)) {
    logMessage("No se pudo iniciar el ToF izquierdo");
    return false;
  }

  digitalWrite(kCenterXshutPin, HIGH);
  delay(10);
  if (!centerTof.begin(kCenterTofAddress)) {
    logMessage("No se pudo iniciar el ToF central");
    return false;
  }

  digitalWrite(kRightXshutPin, HIGH);
  delay(10);
  if (!rightTof.begin(kRightTofAddress)) {
    logMessage("No se pudo iniciar el ToF derecho");
    return false;
  }

  if (!leftTof.startRangeContinuous(50) ||
      !centerTof.startRangeContinuous(50) ||
      !rightTof.startRangeContinuous(50)) {
    logMessage("No se pudo iniciar el rango continuo de los ToF");
    return false;
  }

  logMessage("Sensores ToF listos");
  return true;
}

void readTofMillimeters(Adafruit_VL53L0X &sensor, int32_t &distance) {
  if (!sensor.isRangeComplete()) {
    return;
  }
  const uint16_t millimeters = sensor.readRangeResult();
  distance = millimeters <= kMaximumTofDistanceMm ? millimeters : 0;
}

void updateTofSensors(uint32_t nowMs) {
  if (!tofReady) {
    if (nowMs - lastTofRetryMs >= kTofRetryIntervalMs) {
      lastTofRetryMs = nowMs;
      tofReady = initializeTofSensors();
    }
    return;
  }
  readTofMillimeters(leftTof, tofValues[0]);
  readTofMillimeters(centerTof, tofValues[1]);
  readTofMillimeters(rightTof, tofValues[2]);
}

bool initializeImu() {
  if (imu.begin() != INV_SUCCESS) {
    logMessage("No se pudo iniciar el MPU6500");
    return false;
  }
  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  imu.setLPF(5);
  imu.setSampleRate(100);
  imu.configureFifo(INV_Z_GYRO | INV_XYZ_ACCEL);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Calibrando MPU6500"));
  display.println(F("Mantenga quieto"));
  display.display();
  logMessage("Calibrando MPU6500; mantenga el robot quieto...");
  delay(1000);

  constexpr uint16_t sampleCount = 200;
  uint16_t validSamples = 0;
  float gyroSum = 0.0f;
  float accelXSum = 0.0f;
  float accelYSum = 0.0f;
  const uint32_t calibrationStartMs = millis();
  while (validSamples < sampleCount &&
         millis() - calibrationStartMs < 15000) {
    if (imu.fifoAvailable() == 0) {
      delay(1);
      continue;
    }
    if (imu.updateFifo() != INV_SUCCESS) {
      continue;
    }
    gyroSum += imu.calcGyro(imu.gz);
    accelXSum += imu.calcAccel(imu.ax);
    accelYSum += imu.calcAccel(imu.ay);
    ++validSamples;
  }

  if (validSamples != sampleCount) {
    logMessage("Tiempo agotado al calibrar el MPU6500");
    return false;
  }
  gyroZOffset = gyroSum / sampleCount;
  accelXOffset = accelXSum / sampleCount;
  accelYOffset = accelYSum / sampleCount;
  logMessage("Calibracion del MPU6500 completada");
  return true;
}

void updateImu() {
  if (!imuReady) {
    return;
  }
  while (imu.fifoAvailable() > 0) {
    if (imu.updateFifo() != INV_SUCCESS) {
      continue;
    }
    gyroZDegreesPerSecond = imu.calcGyro(imu.gz) - gyroZOffset;
    accelXG = imu.calcAccel(imu.ax) - accelXOffset;
    accelYG = imu.calcAccel(imu.ay) - accelYOffset;
  }
}

void setWheelSetpoint(const void *message) {
  const auto *command =
      static_cast<const std_msgs__msg__Float32MultiArray *>(message);
  if (command->data.size != 2) {
    logMessage("Setpoint ignorado: se requieren [izq, der]");
    return;
  }

  const float leftRpm = command->data.data[0] * kRadiansToRpm;
  const float rightRpm = command->data.data[1] * kRadiansToRpm;
  if (!std::isfinite(leftRpm) || !std::isfinite(rightRpm)) {
    logMessage("Setpoint ignorado: excede el rango numerico del PID");
    return;
  }
  leftMotor.setpointRpm = leftRpm;
  rightMotor.setpointRpm = rightRpm;
  lastSetpointMs = millis();
}

void initializeMicroRos() {
  allocator = rcl_get_default_allocator();
  checkRcl(
      rclc_support_init(&support, 0, nullptr, &allocator),
      "rclc_support_init");
  checkRcl(
      rclc_node_init_default(
          &node, "differential_robot_esp32", "/robot", &support),
      "rclc_node_init_default");

  checkRcl(
      rclc_publisher_init_default(
          &wheelVelocityPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/robot/wheel_angular_velocities"),
      "publisher wheel_angular_velocities");
  checkRcl(
      rclc_publisher_init_default(
          &imuPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/robot/imu/planar"),
      "publisher imu/planar");
  checkRcl(
      rclc_publisher_init_default(
          &tofPublisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
          "/robot/tof/distances_mm"),
      "publisher tof/distances_mm");
  checkRcl(
      rclc_subscription_init_default(
          &setpointSubscription, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/robot/wheel_angular_velocity_setpoints"),
      "subscription wheel_angular_velocity_setpoints");

  std_msgs__msg__Float32MultiArray__init(&wheelVelocityMessage);
  wheelVelocityMessage.data.data = wheelVelocityValues;
  wheelVelocityMessage.data.size = 2;
  wheelVelocityMessage.data.capacity = 2;
  std_msgs__msg__Float32MultiArray__init(&imuMessage);
  imuMessage.data.data = imuValues;
  imuMessage.data.size = 3;
  imuMessage.data.capacity = 3;
  std_msgs__msg__Int32MultiArray__init(&tofMessage);
  tofMessage.data.data = tofValues;
  tofMessage.data.size = 3;
  tofMessage.data.capacity = 3;
  std_msgs__msg__Float32MultiArray__init(&setpointMessage);
  setpointMessage.data.data = setpointValues;
  setpointMessage.data.size = 2;
  setpointMessage.data.capacity = 2;

  checkRcl(
      rclc_executor_init(&executor, &support.context, 1, &allocator),
      "rclc_executor_init");
  checkRcl(
      rclc_executor_add_subscription(
          &executor, &setpointSubscription, &setpointMessage,
          setWheelSetpoint, ON_NEW_DATA),
      "rclc_executor_add_subscription");
}

void publishSensorData(bool publishWheelVelocity) {
  imuValues[0] = gyroZDegreesPerSecond * kDegreesToRadians;
  imuValues[1] = accelXG * kGravityMetersPerSecondSquared;
  imuValues[2] = accelYG * kGravityMetersPerSecondSquared;

  if (publishWheelVelocity) {
    wheelVelocityValues[0] = leftMotor.measuredRpm / kRadiansToRpm;
    wheelVelocityValues[1] = rightMotor.measuredRpm / kRadiansToRpm;
    const rcl_ret_t result =
        rcl_publish(&wheelVelocityPublisher, &wheelVelocityMessage, nullptr);
    if (result != RCL_RET_OK) {
      logMessage(
          String("No se pudieron publicar las velocidades: ") +
          String(static_cast<int>(result)));
    }
  }
  rcl_ret_t result = rcl_publish(&imuPublisher, &imuMessage, nullptr);
  if (result != RCL_RET_OK) {
    logMessage(
        String("No se pudieron publicar los datos del MPU6500: ") +
        String(static_cast<int>(result)));
  }
  result = rcl_publish(&tofPublisher, &tofMessage, nullptr);
  if (result != RCL_RET_OK) {
    logMessage(
        String("No se pudieron publicar las distancias ToF: ") +
        String(static_cast<int>(result)));
  }
}

void updateOled(uint32_t nowMs) {
  if (nowMs - lastDisplayLogMs < 5000) {
    return;
  }
  if (nowMs - lastOledMs < kOledIntervalMs) {
    return;
  }
  lastOledMs = nowMs;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  /*display.print(F("WiFi:"));
  display.print(WiFi.status() == WL_CONNECTED ? F("OK") : F("--"));
  display.print(F(" ROS:OK IMU:"));
  display.print(imuReady ? F("OK") : F("--"));
  display.setCursor(0, 8);*/
  display.print(F("L:"));
  display.print(leftMotor.measuredRpm / kRadiansToRpm, 1);
  display.setCursor(35, 0);
  display.print(F(" R:"));
  display.print(rightMotor.measuredRpm / kRadiansToRpm, 1);
  display.print(F(" rad/s"));
  display.setCursor(0, 8);
  display.print(F("GZ:"));
  display.print(gyroZDegreesPerSecond * kDegreesToRadians, 1);
  display.setCursor(0, 16);
  display.print(F("Ax:"));
  display.print(accelXG * kGravityMetersPerSecondSquared, 1);
  display.setCursor(46, 16);
  display.print(F(" Ay:"));
  display.print(accelYG * kGravityMetersPerSecondSquared, 1);
  display.setCursor(0, 24);
  display.print(F("ToF mm:"));
  for (size_t i = 0; i < 3; ++i) {
    display.print(' ');
    if (tofValues[i] == 0) {
      display.print(F("--"));
    } else {
      display.print(tofValues[i]);
    }
  }
  display.display();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setTimeOut(50);

  if (!display.begin(SSD1306_SWITCHCAPVCC, kOledAddress)) {
    Serial.println("No se pudo iniciar la pantalla OLED");
    while (true) {
      delay(1000);
    }
  }
  displayReady = true;
  logMessage("Iniciando robot...");

  pinMode(kLeftXshutPin, OUTPUT);
  pinMode(kCenterXshutPin, OUTPUT);
  pinMode(kRightXshutPin, OUTPUT);
  digitalWrite(kLeftXshutPin, LOW);
  digitalWrite(kCenterXshutPin, LOW);
  digitalWrite(kRightXshutPin, LOW);
  initializeMotors();

  imuReady = initializeImu();
  if (!imuReady) {
    logMessage("MPU6500 no disponible; se continua sin IMU");
  }
  tofReady = initializeTofSensors();
  if (!tofReady) {
    logMessage("ToF no disponibles; se reintentara la conexion");
  }
  lastTofRetryMs = millis();

  const IPAddress agentIp(
      AGENT_IP_OCTET_1, AGENT_IP_OCTET_2,
      AGENT_IP_OCTET_3, AGENT_IP_OCTET_4);
  logMessage("Conectando a Wi-Fi y al agente micro-ROS...");
  set_microros_wifi_transports(
      WIFI_SSID, WIFI_PASSWORD, agentIp, static_cast<size_t>(AGENT_PORT));
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  logMessage(String("Wi-Fi conectado. IP: ") + WiFi.localIP().toString());

  while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
    logMessage("Agente micro-ROS no disponible; reintentando...");
    delay(2000);
  }

  initializeMicroRos();
  const uint32_t startMs = millis();
  lastSetpointMs = startMs;
  lastEncoderMs = startMs;
  lastPidMs = startMs;
  lastPublishMs = startMs;
  lastOledMs = startMs;
  logMessage("micro-ROS listo; publicando sensores del robot");
}

void loop() {
  const uint32_t nowMs = millis();
  wheelSamplePending = updateEncoderSpeeds(nowMs) || wheelSamplePending;
  updatePid(nowMs);
  updateImu();
  updateTofSensors(nowMs);

  checkRcl(
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)),
      "rclc_executor_spin_some");
  if (nowMs - lastPublishMs >= kSensorPublishIntervalMs) {
    lastPublishMs = nowMs;
    publishSensorData(wheelSamplePending);
    wheelSamplePending = false;
  }
  updateOled(nowMs);
}
