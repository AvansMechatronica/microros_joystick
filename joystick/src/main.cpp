#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

#include <std_msgs/msg/bool.h>
#if(defined TWIST_STAMPED)
#include <geometry_msgs/msg/twist_stamped.h>
#else
#include <geometry_msgs/msg/twist.h>
#endif

#include <cmath>

#if defined(MULTI_COLOR_LED)
  #include <WS2812FX.h>
  WS2812FX ws2812fxStatus = WS2812FX(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);
  #define RGB_BRIGHTNESS 10 // Change white brightness (max 255)
#endif

#if defined(TURTLEBOT3_BURGER)
#define WHEEL_SPEED_MAX   0.22f      // [m/s] maximale wielsnelheid
#elif defined(TURTLEBOT3_WAFFLE)
#define WHEEL_SPEED_MAX   0.22f      // [m/s] maximale wielsnelheid
#elif defined(P3DX)
#define WHEELS_Y_DISTANCE             0.34f //in meters
#define WHEEL_SPEED_MAX               0.8f      // [m/s] maximale wielsnelheid
#else
#error
#endif


// === Afgeleide limieten ===
#define LINEAR_X_MAX      (WHEEL_SPEED_MAX)                  // [m/s]
#define ANGULAR_Z_MAX     ((2.0f * WHEEL_SPEED_MAX) / WHEELS_Y_DISTANCE) // [rad/s]

#define MAX_INPUT_VOLTAGE              2.8

#define LINEAR_X_MIN (LINEAR_X_MAX * 0.1)
#define ANGULAR_Z_MIN (ANGULAR_Z_MAX * 0.1)

#define LINEAR_RESOLUTION (LINEAR_X_MAX/MAX_INPUT_VOLTAGE*2.0)// Half scale
#define ANGULAR_RESOLUTION (ANGULAR_Z_MAX/MAX_INPUT_VOLTAGE*2.0)// Half scale


#if ORIENTATION == 0
  int linear_pin = LINEAR_PIN;
  int angular_pin = ANGULAR_PIN;
  float linear_resolution = -LINEAR_RESOLUTION;
  float angular_resolution = -ANGULAR_RESOLUTION;
#elif ORIENTATION == 1
  int linear_pin = ANGULAR_PIN;
  int angular_pin = LINEAR_PIN;
  float linear_resolution = -LINEAR_RESOLUTION;
  float angular_resolution = ANGULAR_RESOLUTION;
#elif ORIENTATION == 2
  int linear_pin = LINEAR_PIN;
  int angular_pin = ANGULAR_PIN;
  float linear_resolution = LINEAR_RESOLUTION;
  float angular_resolution = ANGULAR_RESOLUTION;
#elif ORIENTATION == 3
  int linear_pin = ANGULAR_PIN;
  int angular_pin = LINEAR_PIN;
  float linear_resolution = LINEAR_RESOLUTION;
  float angular_resolution = -ANGULAR_RESOLUTION;
#else
#error invalid orientation
#endif

rcl_publisher_t twist_publisher;
#if(defined TWIST_STAMPED)
geometry_msgs__msg__TwistStamped twist_stamped;
#else
geometry_msgs__msg__Twist twist;
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define NODE_NAME "twist_publisher"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(__LINE__);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

bool errorLedState = false;

int reset_timer_cnt = 0;

void error_loop(int line){
  Serial.printf("Joystick\nError\nSystem halted\nLine: %d\n", line);
  while(1){

#if defined(MULTI_COLOR_LED)
        ws2812fxStatus.service();
#endif
        if(errorLedState){
#if defined(MULTI_COLOR_LED)
            ws2812fxStatus.setColor(0,0,0);
#else
#if defined(STATUS_LED_PIN)
            digitalWrite(STATUS_LED_PIN, HIGH);
#endif
#endif
            errorLedState = false;
        }
        else{
            //neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0, 0);
#if defined(MULTI_COLOR_LED)
            ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
#else
#if defined(STATUS_LED_PIN)
            digitalWrite(STATUS_LED_PIN, LOW);
#endif
#endif
            errorLedState = true;
        }
    delay(100);
    if(reset_timer_cnt++ >= 50){
      ESP.restart(); // Reset after 5 seconds
    }
  }
}

int angular_offset, linear_offset;

float round(float waarde, int decimalen) {
    float factor = std::pow(10, decimalen);
    return std::round(waarde * factor) / factor;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  float angular, linear;
  if (timer != NULL) {
    int linear, angular;
    linear = analogReadMilliVolts(linear_pin) - linear_offset;
    angular = analogReadMilliVolts(angular_pin) - angular_offset;
#if(defined TWIST_STAMPED)
    //Serial.printf("Linear = %02f, Angular = %02f\n", linear, angular);
    twist_stamped.twist.linear.x = linear  * linear_resolution * SPEED_FACTOR /1000.0;
    twist_stamped.twist.angular.z = angular * angular_resolution * SPEED_FACTOR /1000.0;
    twist_stamped.header.stamp.sec = 0;
    twist_stamped.header.stamp.nanosec = 0;

    RCSOFTCHECK(rcl_publish(&twist_publisher, &twist_stamped, NULL));
#else
    //Serial.printf("Linear = %02f, Angular = %02f\n", linear, angular);
    twist.linear.x = linear  * linear_resolution * SPEED_FACTOR /1000.0;
    twist.angular.z = angular * angular_resolution * SPEED_FACTOR /1000.0;
    // Publish only if above minimum thresholds
    if((twist.linear.x > -LINEAR_X_MIN 
       && twist.linear.x < LINEAR_X_MIN) && 
       (twist.angular.z > -ANGULAR_Z_MIN 
       && twist.angular.z < ANGULAR_Z_MIN)){
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    RCSOFTCHECK(rcl_publish(&twist_publisher, &twist, NULL));
#endif
  }
}

#if defined(WIFI)
  String wifiWIFI_SSID = WIFI_SSID;
  String wifiPass = WIFI_PASSWORD;
#endif

void setup() {
  // Configure serial transport
#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.init();
  ws2812fxStatus.setMode(FX_MODE_STATIC);
  ws2812fxStatus.setColor(RGB_BRIGHTNESS,0,0);
  ws2812fxStatus.setBrightness(100);
  ws2812fxStatus.setSpeed(200);
  ws2812fxStatus.start();
  ws2812fxStatus.service();
#else
#if defined(STATUS_LED_PIN)
  pinMode(STATUS_LED_PIN, OUTPUT); 
  digitalWrite(STATUS_LED_PIN, HIGH);
#endif
#endif

  Serial.begin(115200);

#if defined(WIFI)
//#define WIFI_SSID "Wifi_ssid"
//#define WIFI_PASSWORD "Wifi_Password"



  WiFi.setHostname("JoystickController");
  set_microros_wifi_transports(wifiWIFI_SSID, wifiPass, AGENT_IP_ADDRESS, (size_t)PORT);
#else
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
#endif

#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.setColor(0, 0, RGB_BRIGHTNESS);
  ws2812fxStatus.service();
#endif
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));


#if(defined TWIST_STAMPED)
  // create twist_publisher
  RCCHECK(rclc_publisher_init_default(
    &twist_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
    "cmd_vel"));
#else
  // create twist_publisher
  RCCHECK(rclc_publisher_init_default(
    &twist_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
#endif

  // create timer,
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

#if defined(MULTI_COLOR_LED)
  ws2812fxStatus.setColor(0, RGB_BRIGHTNESS,0);
#else
#if defined(STATUS_LED_PIN)
  digitalWrite(STATUS_LED_PIN, LOW);
#endif
#endif

#define NUMBER_OF_AVERAGE  10
  // Callibrate zerro offset
  delay(1000);
  int linear_offset_sum =0, angular_offset_sum = 0;
  for(int i = 0; i < NUMBER_OF_AVERAGE; i++){
    linear_offset_sum += analogReadMilliVolts(linear_pin);
    angular_offset_sum += analogReadMilliVolts(angular_pin);
    delay(100);
  }
  linear_offset = linear_offset_sum / NUMBER_OF_AVERAGE;
  angular_offset = angular_offset_sum / NUMBER_OF_AVERAGE;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

#if defined(MULTI_COLOR_LED)
    ws2812fxStatus.service();
#endif
}