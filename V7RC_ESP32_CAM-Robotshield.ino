#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include <ESP32Servo.h>

#define SERVO_DEFAULT_VALUE 1500
#define SERVO_DEFAULT_MAX_VALUE 2000
#define SERVO_DEFAULT_MIN_VALUE 1000
#define PACKET_LENGTH 20
#define numOfServo 4

// Servo Pin,  目前先用1組(2)，其餘-1不處理;
const int SERVO_Pin[] = { -1, -1, 2, -1 };
Servo robotServo[numOfServo];

int count = 0;  //計數用

int servValue[numOfServo];          // 目前的伺服馬達數值
int oldServValue[numOfServo];       // 舊的的伺服馬達數值
int failSafeServValue[numOfServo];  // 各個伺服馬達FailSafe設定
int servoMAXValue[numOfServo];      // 每個Servo最大的數值
int servoMINValue[numOfServo];      // 每個Servo最小的數值
int bytes[PACKET_LENGTH];
int receiveServoValue[numOfServo];

int dcMotorPinA[] = { 12, 13 };  // DC motor A
int dcMotorPinB[] = { 14, 15 };  // DC motor B


long startProcessTime = 0;
long endProcessTime = 0;

// 測試Servo相關數據
boolean ifTestMode = false;   // 是否進入測試模式
boolean ifAddValue = false;   // 是否增加資料
int servoMoveStepValue = 80;  // 測試用的速度

#define LOST_SIGNAL_MAX_TIME 500  // 最大失去信號時間;

int currentLostSignalTime = 0;

int accelPWMValue = SERVO_DEFAULT_VALUE;  // 需要控制油門的的PWM;
int accelPWMChannel = 1;                  // Channel0, 1, 2, 3, 4, 5, 6....
bool isControlAccelerator = false;        // 是否需要限制油門;

// Set these to your desired credentials.
// const char *ssid = "WiFi_ESP32";  //設定一組網路名稱(ssid)
char ssid[23];                      //設定一組網路名稱(ssid)
const char *password = "12345678";  //設定一組網路密碼(pasword)

WiFiUDP Udp;

char packetBuffer[255];

unsigned int localPort = 6188;

IPAddress local_IP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

#define PART_BOUNDARY "123456789000000000000987654321"

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

/* define CAMERA_MODEL_AI_THINKER */
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

bool displayFrameTime = false;  // 控制是否顯示幀處理時間
bool calculateFPS = false;      // 控制是否計算FPS

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  static int64_t last_frame = 0;                        // 用於計算單幀處理時間
  static int frame_count = 0;                           // 幀計數器，用於計算FPS
  static int64_t fps_last_time = esp_timer_get_time();  // 用於FPS計算的時間戳記，初始化為當前時間


  if (displayFrameTime && !last_frame) {
    last_frame = esp_timer_get_time();
  }


  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.printf("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.printf("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }

    // 計算單幀處理時間
    if (displayFrameTime) {
      int64_t fr_end = esp_timer_get_time();
      int64_t frame_time = fr_end - last_frame;
      last_frame = fr_end;
      frame_time /= 1000;  // 轉換為毫秒
      Serial.printf("Frame time: %lld ms\n", frame_time);

      // 重置單幀處理時間
      last_frame = 0;
    }

    // 計算FPS
    if (calculateFPS) {
      frame_count++;                                      // 增加幀計數
      int64_t now = esp_timer_get_time();                 // 獲取當前時間
      int64_t elapsed = now - fps_last_time;              // 計算自上次更新FPS以來經過的時間
      if (elapsed >= 1000000) {                           // 如果超過1秒
        float fps = frame_count / (elapsed / 1000000.0);  // 計算FPS
        Serial.printf("FPS: %.2f\n", fps);

        // 重置計數器和時間戳記
        frame_count = 0;
        fps_last_time = now;
      }
    }
  }

  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}



const char* versionInfo = "Version: Test_5"; // 編譯版本
char burnDate[] = __DATE__; // 編譯日期
char burnTime[] = __TIME__; // 編譯時間

void setup() {
  servoControlInit();

  Serial.begin(115200);
  
  Serial.println("========");
  Serial.println(versionInfo);
  Serial.print("burnDate:");
  Serial.println(burnDate);
  Serial.print("burnTime:");
  Serial.println(burnTime);
  Serial.println("========");


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
  // if (psramFound()) {
  //   config.frame_size = FRAMESIZE_UXGA;
  //   config.jpeg_quality = 10;
  //   config.fb_count = 2;
  // } else {
  //   config.frame_size = FRAMESIZE_SVGA;
  //   config.jpeg_quality = 12;
  //   config.fb_count = 1;
  // }

  config.frame_size = FRAMESIZE_HVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  // sensor_t *s = esp_camera_sensor_get();
  // s->set_framesize(s, FRAMESIZE_HVGA);

  WiFi.mode(WIFI_AP);
  Serial.println();
  Serial.println("Configuring soft-AP...");

  WiFi.softAPConfig(local_IP, gateway, subnet);


  // 獲取ESP32的Chip ID
  uint64_t chipid = ESP.getEfuseMac();                                               // 獲取EFUSE MAC地址，用於Chip ID
  snprintf(ssid, 23, "ESP32_%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);  // 將Chip ID格式化並存儲於ssid陣列
  WiFi.softAP(ssid, password);

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("softAP macAddress: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println("Server started.....");

  Udp.begin(localPort);

  startCameraServer();


  // setup DC motor;
  pinMode(dcMotorPinA[0], OUTPUT);
  pinMode(dcMotorPinA[1], OUTPUT);
  pinMode(dcMotorPinB[0], OUTPUT);
  pinMode(dcMotorPinB[1], OUTPUT);

  processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinA);
  processDCMotor(SERVO_DEFAULT_VALUE, dcMotorPinB);
}

bool isDebug = false;
IPAddress replyPlace;  //對方裝置的位置

void loop() {

  int packetSize = Udp.parsePacket();

  if (packetSize) {

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }

    if (isDebug) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      replyPlace = Udp.remoteIP();
      Serial.print(replyPlace);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());

      Serial.println("Contents:");
      Serial.println(packetBuffer);
    }

    String receiveCommand = String(packetBuffer);
    processRCString(receiveCommand);
  }
}


//設定servo初始值
void servoControlInit() {

  // 設定Servo, 並且設定成預設1500;

  for (int i = 0; i < (sizeof(SERVO_Pin) / sizeof(SERVO_Pin[0])); i++) {

    // servoMAXValue[i] = SERVO_DEFAULT_MAX_VALUE;
    // servoMINValue[i] = SERVO_DEFAULT_MIN_VALUE;
    // oldServValue[i] = SERVO_DEFAULT_VALUE;

    if (SERVO_Pin[i] == -1) continue;

    robotServo[i].setPeriodHertz(50);
    //    robotServo[i].attach(SERVO_Pin[i], SERVO_DEFAULT_MIN_VALUE, SERVO_DEFAULT_MAX_VALUE); // 設定
    robotServo[i].attach(SERVO_Pin[i]);  // 設定
    robotServo[i].writeMicroseconds(SERVO_DEFAULT_VALUE);
    // int thisAngle = map(1500, 544, 2400, 0, 180);

    // Serial.println(thisAngle);
    // robotServo[i].write(thisAngle);
  }

  //  servo_channel1.attach(SERVO_Pin1);
  //  servo_channel2.attach(SERVO_Pin2);
  //  servo_channel3.attach(SERVO_Pin3);
  //  servo_channel4.attach(SERVO_Pin4);
  //  servo_channel5.attach(SERVO_Pin5);
  //  servo_channel6.attach(SERVO_Pin6);
}

// 目前只要大於等於16Bytes, 並且在最後有個井字好結尾，那就是合理的command;
void processRCString(String command) {

  int commandLength = command.length();

  if (commandLength > 15) {

    if (command.charAt(commandLength - 1) != '#') {  // 表示結尾不是預設的結果;

      return;
    }

  } else {

    return;
  }

  //  Serial.println(command);


  if (command.indexOf("SRV") > -1 || command.indexOf("SS4") > -1) {  // 表示伺服馬達操作;

    // Serial.println("接收到伺服馬達命令;");

    int i = 3;
    int servoIndex = 0;

    while (i < commandLength - 1) {  // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();


          if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

            if (receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
            if (receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
            oldServValue[servoIndex] = receiveServoValue[servoIndex];

            //               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
            //               robotServo[servoIndex].write(thisAngle);
            robotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
          }

          servoIndex++;
        }
      }

      i = i + 4;
    }

    processDCMotor(receiveServoValue[1], dcMotorPinA);
    processDCMotor(receiveServoValue[0], dcMotorPinB);

    // processServoCommand(receiveServoValue);     // 處理伺服馬達;

  } else if (command.indexOf("SRT") > -1) {  // 表示伺服馬達操作;

    // Serial.println("接收到伺服馬達命令;");

    int i = 3;
    int servoIndex = 0;

    while (i < commandLength - 1) {  // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();


          if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

            if (receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
            if (receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
            oldServValue[servoIndex] = receiveServoValue[servoIndex];

            //               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
            //               robotServo[servoIndex].write(thisAngle);
            //crobotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
          }

          servoIndex++;
        }
      }

      i = i + 4;
    }

    // 這裡要處理坦克的訊號
    int tankServo1 = 1500;
    int tankServo0 = 1500;

    if (receiveServoValue[1] >= 1500) {

      int duration = receiveServoValue[1] - 1500;

      tankServo1 = tankServo1 + duration;
      tankServo0 = tankServo0 - duration;

    } else {

      int duration = 1500 - receiveServoValue[1];

      tankServo1 = tankServo1 - duration;
      tankServo0 = tankServo0 + duration;
    }

    if (receiveServoValue[0] >= 1500) {

      int duration = receiveServoValue[0] - 1500;

      tankServo1 = tankServo1 + duration;
      tankServo0 = tankServo0 + duration;
    } else {

      int duration = 1500 - receiveServoValue[0];

      tankServo1 = tankServo1 - duration;
      tankServo0 = tankServo0 - duration;
    }

    //    Serial.print("tankServo0:");
    //    Serial.println(tankServo0);
    //
    //    Serial.print("tankServo1:");
    //    Serial.println(tankServo1);

    if (tankServo0 < 1000) {
      tankServo0 = 1000;
    }

    if (tankServo0 > 2000) {
      tankServo0 = 2000;
    }

    if (tankServo1 < 1000) {
      tankServo1 = 1000;
    }

    if (tankServo1 > 2000) {
      tankServo1 = 2000;
    }

    receiveServoValue[0] = tankServo0;
    receiveServoValue[1] = tankServo1;

    //    Serial.print("servo 0:");
    //    Serial.println(receiveServoValue[0]);

    processDCMotor(receiveServoValue[0], dcMotorPinA);

    processDCMotor(receiveServoValue[1], dcMotorPinB);

    // 傳給ProcessServoCommand發送訊號;
    processServoCommand(receiveServoValue);  // 處理伺服馬達;

  } else if (command.indexOf("SR2") > -1) {
    // Serial.println("接收到第二組伺服馬達命令;");

    int i = 3;
    int servoIndex = 4;

    while (i < commandLength - 1) {  // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 4);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();


          if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

            if (receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
            if (receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
            oldServValue[servoIndex] = receiveServoValue[servoIndex];

            //               int thisAngle = map(receiveServoValue[servoIndex], 544, 2400, 0, 180);
            //               robotServo[servoIndex].write(thisAngle);
            robotServo[servoIndex].writeMicroseconds(receiveServoValue[servoIndex]);
          }

          servoIndex++;
        }
      }

      i = i + 4;
    }
  } else if (command.indexOf("SS8") > -1) {
    // Serial.println("接收到第二組伺服馬達命令;");

    int i = 3;
    int servoIndex = 4;

    while (i < commandLength - 1) {  // 解碼;

      if (i + 3 < commandLength) {

        String singleCommand = command.substring(i, i + 2);

        // Serial.println(singleCommand);

        if (servoIndex < numOfServo) {

          receiveServoValue[servoIndex] = singleCommand.toInt();


          if (receiveServoValue[servoIndex] != oldServValue[servoIndex]) {

            if (receiveServoValue[servoIndex] > SERVO_DEFAULT_MAX_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MAX_VALUE;
            if (receiveServoValue[servoIndex] < SERVO_DEFAULT_MIN_VALUE) receiveServoValue[servoIndex] = SERVO_DEFAULT_MIN_VALUE;
            oldServValue[servoIndex] = receiveServoValue[servoIndex];

            int pwmValue = map(receiveServoValue[servoIndex], 0, 99, 544, 2400);
            // robotServo[servoIndex].write(thisAngle);
            robotServo[servoIndex].writeMicroseconds(pwmValue);
          }

          servoIndex++;
        }
      }

      i = i + 2;
    }
  }
}


//將命令發送至servo
void V7RCCommand(int servoValue1, int servoValue2, int servoValue3, int servoValue4, int servoValue5, int servoValue6, int servoValue7, int servoValue8) {


  // Serial.println( servoValue1 );
  //       Serial.println("servoAngle[0]: "+String(servoAngle[0])+"servoAngle[1]: " +String(servoAngle[1]));

  if (servoValue1 != oldServValue[0]) {

    if (servoValue1 > servoMAXValue[0]) {
      servoValue1 = servoMAXValue[0];
    } else if (servoValue1 < servoMINValue[0]) {
      servoValue1 = servoMINValue[0];
    }

    oldServValue[0] = servoValue1;
    robotServo[0].write(servoValue1);
    //
    //delay(5);
  }

  if (servoValue2 != oldServValue[1]) {


    if (servoValue2 > servoMAXValue[1]) {
      servoValue2 = servoMAXValue[1];
    } else if (servoValue2 < servoMINValue[1]) {
      servoValue2 = servoMINValue[1];
    }

    oldServValue[1] = servoValue2;
    robotServo[1].write(servoValue2);
    //servo_channel2.write(servoValue2);
    //delay(5);
  }

  if (servoValue3 != oldServValue[2]) {

    if (servoValue3 > servoMAXValue[2]) {
      servoValue3 = servoMAXValue[2];
    } else if (servoValue3 < servoMINValue[2]) {
      servoValue3 = servoMINValue[2];
    }

    oldServValue[2] = servoValue3;
    robotServo[2].write(servoValue2);
    //servo_channel3.write(servoValue3);
    //delay(5);
  }

  if (servoValue4 != oldServValue[3]) {

    if (servoValue4 > servoMAXValue[3]) {
      servoValue4 = servoMAXValue[3];
    } else if (servoValue4 < servoMINValue[3]) {
      servoValue4 = servoMINValue[3];
    }

    oldServValue[3] = servoValue4;
    robotServo[3].write(servoValue4);
    //delay(5);
  }

  if (servoValue5 != oldServValue[4]) {

    if (servoValue5 > servoMAXValue[4]) {
      servoValue5 = servoMAXValue[4];
    } else if (servoValue5 < servoMINValue[4]) {
      servoValue5 = servoMINValue[4];
    }

    oldServValue[4] = servoValue5;
    robotServo[4].write(servoValue5);
    //delay(5);
  }

  if (servoValue6 != oldServValue[5]) {

    if (servoValue6 > servoMAXValue[5]) {
      servoValue6 = servoMAXValue[5];
    } else if (servoValue6 < servoMINValue[5]) {
      servoValue6 = servoMINValue[5];
    }

    oldServValue[5] = servoValue6;
    robotServo[5].write(servoValue6);
    //delay(5);
  }

  if (servoValue7 != oldServValue[6]) {

    if (servoValue7 > servoMAXValue[6]) {
      servoValue7 = servoMAXValue[6];
    } else if (servoValue7 < servoMINValue[6]) {
      servoValue7 = servoMINValue[6];
    }

    oldServValue[6] = servoValue7;
    robotServo[6].write(servoValue7);
    //delay(5);
  }

  if (servoValue8 != oldServValue[7]) {

    if (servoValue8 > servoMAXValue[7]) {
      servoValue8 = servoMAXValue[7];
    } else if (servoValue8 < servoMINValue[7]) {
      servoValue8 = servoMINValue[7];
    }

    oldServValue[7] = servoValue8;
    robotServo[7].write(servoValue8);
    //delay(5);
  }

  //  servo_channel1.write(servoValue2);
  //  servo_channel2.write(servoValue2);
  //  servo_channel3.write(servoValue3);
  //  servo_channel4.write(servoValue4);
  //  servo_channel5.write(servoValue5);
  //  servo_channel6.write(servoValue6);
  // servoPosition(SERVO_Pin1, servoAngle[0]);
}

void processServoCommand(int servoValue[]) {

  int thisIndex = 0;
  while (thisIndex < numOfServo && thisIndex < sizeof(servoValue)) {
    if (SERVO_Pin[thisIndex] != -1) {
      robotServo[thisIndex].writeMicroseconds(servoValue[thisIndex]);
    }
    thisIndex++;
  }
}

void processDCMotor(int pwmValue, int dcMotor[]) {
  if (pwmValue == 1500) {
    digitalWrite(dcMotor[0], LOW);
    analogWrite(dcMotor[1], 0);

  } else if (pwmValue > 1500) {
    int power = map(pwmValue, 1500, 2000, 0, 255);
    digitalWrite(dcMotor[0], LOW);
    analogWrite(dcMotor[1], power);
  } else {
    int power = map(pwmValue, 1500, 1000, 255, 0);
    digitalWrite(dcMotor[0], HIGH);
    analogWrite(dcMotor[1], power);
  }
}
