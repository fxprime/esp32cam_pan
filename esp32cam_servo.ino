/**
 * www.arduinoNa.com
 * จากบทความ [โปรเจค] ใช้ esp32-cam เป็นกล้องวงจรปิดแบบพกพา
 */

#include "esp_camera.h"
#include <esp_wifi.h> //NEED THIS TO COMPILE
#include <esp_wifi_internal.h>
#include <WiFi.h>

//ต่อ WiFi ไหน รหัสอะไรใส่ตรงนี้
const char* ssid = "ArduinoNa_AP";
const char* password = "3213213213";
IPAddress local_IP(192, 168, 67, 195);
IPAddress gateway(192, 168, 67, 1);
IPAddress subnet(255, 255, 254, 0);
    
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

//Pin ต่างๆที่ใช้กับกล้อง
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// GPIO สำหรับมอเตอร์ 
extern float svPanPWM = 1.5;
extern int gpLed =  4; // Light
extern String WiFiAddr ="";

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  

  pinMode(gpLed, OUTPUT); //Light
  ledcAttachPin(15, 4);  //Pin 15 ใช้ servo timer 4
  ledcSetup(4, 400, 10); //400hz 10bit resolution duty of cycle

  //initialize
  digitalWrite(gpLed, LOW);

  
  
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
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);

  //ต่อ WiFi 
  wifi_country_t contry = {
          .cc = "TH",
          .schan = 1,
          .nchan = 11,
          .max_tx_power = 28,
          .policy = WIFI_COUNTRY_POLICY_MANUAL
  };
  esp_wifi_set_country(&contry);
  esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT20);

    
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.config(local_IP, gateway, subnet);


  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  //เปิดเซิฟเวอร์สำหรับฟีดภาพจากกล้อง
  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  WiFiAddr = WiFi.localIP().toString();
  Serial.println("' to connect");


  for(int i=0; i<17; i++) {
    digitalWrite(gpLed, i%2);
    delay(100);
  }


}

int pwmToDutycycleSmooth(float pwm) {
  static float pwm_smooth = pwm;
  const float speed = 0.5; // 2 ms = 180 deg/s, 0.5ms = 45deg/s in 1 sec

  if( fabs(pwm - pwm_smooth) < 0.01 ) {
   //Do nothing 
  }else if(pwm < pwm_smooth){
    pwm_smooth -= (speed*0.01);
  }else if(pwm > pwm_smooth){
    pwm_smooth += (speed*0.01);
  }

  pwm_smooth = constrain(pwm_smooth, 0.5, 2.5);
  int duty = pwm_smooth*(400.0/1000.0)*1000.0;
  return duty;
}
void loop() {
  ledcWrite(4, pwmToDutycycleSmooth(svPanPWM));
//  Serial.println((temprature_sens_read() - 32) / 1.8);
  delay(10);
}
