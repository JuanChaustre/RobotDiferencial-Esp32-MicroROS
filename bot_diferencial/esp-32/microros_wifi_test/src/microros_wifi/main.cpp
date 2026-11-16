#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#include "secrets.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

static const IPAddress agent_ip(
    AGENT_IP_OCTET_1,
    AGENT_IP_OCTET_2,
    AGENT_IP_OCTET_3,
    AGENT_IP_OCTET_4);

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t status_publisher;
static rcl_subscription_t led_subscription;
static rcl_timer_t status_timer;
static rclc_executor_t executor;
static std_msgs__msg__Int32 status_message;
static std_msgs__msg__Bool led_command;

[[noreturn]] static void stopOnError(const char *operation, rcl_ret_t error) {
  Serial.print("micro-ROS error in ");
  Serial.print(operation);
  Serial.print(": ");
  Serial.println(static_cast<int>(error));
  while (true) {
    delay(1000);
  }
}

static void checkRcl(rcl_ret_t result, const char *operation) {
  if (result != RCL_RET_OK) {
    stopOnError(operation, result);
  }
}

static void publishStatus(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;

  ++status_message.data;
  const rcl_ret_t result = rcl_publish(&status_publisher, &status_message, nullptr);
  if (result != RCL_RET_OK) {
    Serial.print("No se pudo publicar el contador: ");
    Serial.println(static_cast<int>(result));
  }
}

static void setLed(const void *message) {
  const std_msgs__msg__Bool *command =
      static_cast<const std_msgs__msg__Bool *>(message);
  digitalWrite(LED_BUILTIN, command->data ? HIGH : LOW);
  Serial.println(command->data ? "LED encendido por ROS 2" : "LED apagado por ROS 2");
}

static void initializeMicroRos() {
  allocator = rcl_get_default_allocator();
  checkRcl(rclc_support_init(&support, 0, nullptr, &allocator), "rclc_support_init");
  checkRcl(
      rclc_node_init_default(&node, "esp32_microros_wifi_test", "/robot", &support),
      "rclc_node_init_default");
  checkRcl(
      rclc_publisher_init_default(
          &status_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "/esp32/status_counter"),
      "rclc_publisher_init_default");
  checkRcl(
      rclc_subscription_init_default(
          &led_subscription,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
          "/esp32/led_cmd"),
      "rclc_subscription_init_default");
  checkRcl(
      rclc_timer_init_default(
          &status_timer, &support, RCL_MS_TO_NS(1000), publishStatus),
      "rclc_timer_init_default");
  checkRcl(
      rclc_executor_init(&executor, &support.context, 2, &allocator),
      "rclc_executor_init");
  checkRcl(
      rclc_executor_add_timer(&executor, &status_timer),
      "rclc_executor_add_timer");
  checkRcl(
      rclc_executor_add_subscription(
          &executor, &led_subscription, &led_command, setLed, ON_NEW_DATA),
      "rclc_executor_add_subscription");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  delay(1000);

  Serial.println("Conectando a Wi-Fi...");
  set_microros_wifi_transports(
      WIFI_SSID, WIFI_PASSWORD, agent_ip, static_cast<size_t>(AGENT_PORT));
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi-Fi conectado. IP del ESP32: ");
  Serial.println(WiFi.localIP());

  Serial.print("Esperando micro-ROS Agent en ");
  Serial.print(agent_ip);
  Serial.print(":");
  Serial.println(AGENT_PORT);
  while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
    Serial.println("Agente no disponible; reintentando...");
    delay(2000);
  }

  initializeMicroRos();
  Serial.println("micro-ROS conectado. Publicando /esp32/status_counter.");
}

void loop() {
  checkRcl(
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)),
      "rclc_executor_spin_some");
}
