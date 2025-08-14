#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

// ROS2 메시지 타입들
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <std_srvs/srv/set_bool.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// ===========================
// ROS2 관련 변수들
// ===========================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t camera_image_pub;
rcl_publisher_t mic_fft_pub;
rcl_publisher_t camera_status_pub;

rcl_subscription_t camera_auto_sub;
rcl_subscription_t camera_shotperiod_sub;
rcl_subscription_t camera_capture_sub;

rcl_service_t camera_config_srv;

// 메시지들
sensor_msgs__msg__CompressedImage camera_msg;


#include <WiFi.h>
char* ssid = "DIRECT-ZvC145x Series le";
char* password = "11111111";
char* MICRO_ROS_AGENT_IP = "10.30.100.247";
const int MICRO_ROS_AGENT_PORT = 8888;

// ESP32-S3 하드웨어
#define CAMERA_MODEL_XIAO_ESP32S3
#include "esp_camera.h"
// 카메라 핀 설정
#include "camera_pins.h"

// ESP32-S3 I2S 설정
#include <ESP_I2S.h>
I2SClass I2S;
#define FFT_SAMPLES 64  // 더 작은 크기로 변경
#define SAMPLING_FREQUENCY 16000

bool auto_camera_mode = true;
int shot_period_seconds = 5;
unsigned long last_shot_time = 0;
unsigned long last_fft_time = 0;

// ===========================
// 유틸리티 함수들
// ===========================
const char* NODE_PREFIX = "robot1";


String create_topic_name(const char* base_topic) {
    return String("/") + NODE_PREFIX + String(base_topic);
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
    static unsigned long led_time = 0;
    static bool led_state = false;
    
    while(1){
        unsigned long current_time = millis();
        if (current_time - led_time >= 100) {
            led_state = !led_state;
            digitalWrite(LED_BUILTIN, led_state);
            led_time = current_time;
        }
        yield();
    }
}

// ===========================
// 카메라 초기화 함수
// ===========================
bool init_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_QVGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 25;
    config.fb_count = 2;
    
    if(psramFound()){
        config.jpeg_quality = 20;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("카메라 초기화 실패: 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 0);
        s->set_brightness(s, 0);
        s->set_saturation(s, 0);
        Serial.printf("카메라 센서: PID=0x%x\n", s->id.PID);
    }
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 0);
      s->set_brightness(s, 1);
      s->set_saturation(s, -2);
    } else if (s->id.PID == OV2640_PID) {
      s->set_vflip(s, 0);
      s->set_brightness(s, 0);
      s->set_saturation(s, 0);
    } else if (s->id.PID == OV5640_PID) {
      s->set_vflip(s, 0);
      s->set_brightness(s, 0);
      s->set_saturation(s, 0);
    } 

    Serial.println("✅ 카메라 초기화 완료");
    return true;
}
// ===========================
// 카메라 캡처 함수
// ===========================
camera_fb_t * fb = NULL;

void capture_and_publish_image() {
  fb = esp_camera_fb_get();
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;

    if (!fb) {
        Serial.println("❌ 카메라 캡처 실패");
        return;
    }
    else{

        if (fb->format != PIXFORMAT_JPEG) 
            Serial.printf("not JPG format\n");
        else
            Serial.printf("JPG Format\n");

        if( fb->len <= camera_msg.data.capacity )
		{
		// use fb->buf to access the image
		    camera_msg.data.size = fb->len;
			memcpy(camera_msg.data.data,fb->buf,fb->len);
            Serial.printf("📷 이미지 캡처: %d bytes\n", fb->len);
        }
        else
            Serial.printf("Captured Image size %zu bytes %d %d, capacity : %zu\n", fb->len, fb->width, fb->height, camera_msg.data.capacity);

    }


    
    auto now = rmw_uros_epoch_millis();
    camera_msg.header.stamp.sec = now / 1000;
    camera_msg.header.stamp.nanosec = (now % 1000) * 1000000;
        
    camera_msg.header.frame_id = micro_ros_string_utilities_set(camera_msg.header.frame_id, "camera_link");
    // camera_msg.header.frame_id.size = strlen("camera_link");
    
    // camera_msg.format.data = (char*)"jpeg";
    // camera_msg.format.size = strlen("jpeg");
    camera_msg.format = micro_ros_string_utilities_set(camera_msg.format, "jpeg");

    // if (fb->format != PIXFORMAT_JPEG) {
    //   bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
    //   esp_camera_fb_return(fb);
    //   fb = NULL;
    //   if (!jpeg_converted) {
    //     log_e("JPEG compression failed");
    //   }
    // } else {
    //   _jpg_buf_len = fb->len;
    //   _jpg_buf = fb->buf;
    // }
    // camera_msg.data.size = _jpg_buf_len;
    // camera_msg.data.data = _jpg_buf;

    esp_camera_fb_return(fb);
    // for(int i=0;i<_jpg_buf_len;i++)
    // {
    //     Serial.printf("%16x:%16x ", camera_msg.data.data[i], _jpg_buf[i]);
    // }
    RCSOFTCHECK(rcl_publish(&camera_image_pub, &camera_msg, NULL));
}
// ===========================
// ROS2 콜백 함수들
// ===========================



// #if defined(LED_BUILTIN)
//   #define LED_PIN LED_BUILTIN
// #else
//   #define LED_PIN 13
// #endif

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// void error_loop(){
//   while(1){
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//     delay(100);
//   }
// }

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

void setup() {
    Serial.begin(115200);
    
    // 부팅 대기
    unsigned long start_time = millis();
    while (millis() - start_time < 3000) {
        yield();
    }
    
    Serial.println("🚀 ESP32-S3 micro-ROS2 시작");
    
    // WiFi 연결
    Serial.printf("📡 WiFi 연결: %s\n", ssid);
    WiFi.begin(ssid, password);
    
    unsigned long wifi_start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - wifi_start < 10000)) {
        static unsigned long dot_time = 0;
        if (millis() - dot_time >= 500) {
            Serial.print(".");
            dot_time = millis();
        }
        yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n✅ WiFi 연결: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n❌ WiFi 연결 실패");
        ESP.restart();
    }
        // WiFi 재연결 로직 추가
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    

    // micro-ROS 설정
    set_microros_wifi_transports(ssid, password, MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT);
    Serial.printf("🤖 Agent: %s:%d\n", MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT);
    // // micro-ROS agent 연결 확인
    // unsigned long agent_timeout = millis();
    // while (!rmw_uros_ping_agent(1000, 1) && (millis() - agent_timeout < 10000)) {
    //     Serial.print(".");
    //     delay(500);
    // }
    
    // if (rmw_uros_ping_agent(1000, 1)) {
    //     Serial.println("\n✅ Agent 연결 완료");
    // } else {
    //     Serial.println("\n❌ Agent 연결 실패");
    //     return;
    // }

    // 카메라 초기화
    if (!init_camera()) {
        Serial.println("❌ 카메라 초기화 실패");
        return;
    }
    delay(2000);

    // micro-ROS 초기화
    Serial.println("🔗 micro-ROS 초기화...");

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    Serial.println("🔗 micro-ROS 초기화...1");
    
    String node_name = String(NODE_PREFIX) + "_node";
    RCCHECK(rclc_node_init_default(&node, node_name.c_str(), "", &support));
    Serial.println("🔗 micro-ROS 초기화...2");
    
    // Publishers
    String img_topic = create_topic_name("/camera/image/compressed");
    
    RCCHECK(rclc_publisher_init_default(&camera_image_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage), "image/compressed"/*img_topic.c_str()*/));

	static micro_ros_utilities_memory_conf_t conf = {};

	// OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences

	conf.max_string_capacity = 50;
	conf.max_ros2_type_sequence_capacity = 5;
	conf.max_basic_type_sequence_capacity = 5;
	// OPTIONALLY this struct can store rules for specific members
	// !! Using the API with rules will use dynamic memory allocations for handling strings !!

	micro_ros_utilities_memory_rule_t rules[] = {
		{"header.frame_id", 30},
		{"format",10},
		{"data", 15000}
	};
	conf.rules = rules;
	conf.n_rules = sizeof(rules) / sizeof(rules[0]);
	
	micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
		&camera_msg,
		conf
	);
    // RCCHECK(rclc_publisher_init_best_effort(&camera_image_pub, &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), img_topic.c_str()));

    Serial.println("✅ 시스템 준비 완료!");
    Serial.printf("📋 노드: %s\n", node_name.c_str());
    Serial.printf("📷 이미지: %s\n", img_topic.c_str());
    
}

void loop() {
    unsigned long current_time = millis();
    
        // WiFi 연결 상태 확인
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi 연결 끊어짐");
        WiFi.reconnect();
        return;
    }

    // 자동 촬영
    if (auto_camera_mode) {
      // Serial.printf("🔄 자동 촬영 in the loop: %s\n", auto_camera_mode ? "ON" : "OFF");
      if (current_time - last_shot_time >= (shot_period_seconds * 1000UL)) {
          Serial.printf("period : %d \n", shot_period_seconds);
          capture_and_publish_image();
          last_shot_time = current_time;
      }
    }

}
