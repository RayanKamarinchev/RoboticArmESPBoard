#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"          // Disable brownout problems
#include "soc/rtc_cntl_reg.h" // Disable brownout problems
#include "driver/rtc_io.h"
#include <HTTPClient.h>
#include <FS.h>
#include <SPIFFS.h>

// Replace with your network credentials
const char *ssid = "M-Tel_DF97";
const char *password = "4857544371DF9731";
WiFiClientSecure client;

// Camera module pins (CAMERA_MODEL_AI_THINKER)
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
#define FLASH_GPIO_NUM 4

void flashOnForNSeconds(int seconds)
{
  digitalWrite(FLASH_GPIO_NUM, 0.3);
  delay(seconds * 1000);
  digitalWrite(FLASH_GPIO_NUM, LOW);
}

void initFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else
  {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
}

void initCamera()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

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

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  config.jpeg_quality = 30; // Lower quality for faster transfer

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
}

void initAndConnectWifi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void sendPhotoToServer()
{
  //camera_fb_t *fb = esp_camera_fb_get();
  // if (!fb)
  // {
  //   Serial.println("Camera capture failed");
  //   return;
  // }
  // HTTPClient http;
  // http.begin("https://infiniscript.pythonanywhere.com/get_position");
  // http.addHeader("Content-Type", "multipart/form-data; boundary=123456789");

  // String bodyStart = "--123456789\r\nContent-Disposition: form-data; name=\"file\"; filename=\"photo.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
  // String bodyEnd = "\r\n--123456789--\r\n";

  // int contentLength = bodyStart.length() + fb->len + bodyEnd.length();

  // // Use POST with custom body length
  // WiFiClient *stream = http.getStreamPtr();
  // http.setTimeout(10000);
  // Serial.println(1);
  // int httpCode = http.POST(fb->buf, fb->len);

  // if (httpCode > 0)
  // {
  //   Serial.println(2);
  //   delay(5000);
  //   String response = http.getString();
  //   Serial.println("Server response: " + response);
  // }
  // else
  // {
  //   Serial.printf("Error sending photo: %s\n", http.errorToString(httpCode).c_str());
  // }
  // esp_camera_fb_return(fb);
  // http.end();
  
  String serverName = "infiniscript.pythonanywhere.com";
  String serverPath = "/get_position";
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);
  
  client.setInsecure(); //skip certificate validation
  if (client.connect(serverName.c_str(), 443)) {
    Serial.println("Connection successful!");    
    String head = "--RandomNerdTutorials\r\n"
              "Content-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\n"
              "Content-Type: image/jpeg\r\n\r\n";

String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + serverPath + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    client.println();
    client.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(tail);
    
    esp_camera_fb_return(fb);
    
    int timoutTimer = 10000;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + timoutTimer) > millis()) {
      Serial.print(".");
      delay(100);      
      while (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    client.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
  // return getBody;
}

void setup()
{
  Serial.begin(115200);
  pinMode(FLASH_GPIO_NUM, OUTPUT);

  initAndConnectWifi();
  flashOnForNSeconds(1);

  initFS();
  initCamera();
  flashOnForNSeconds(1);
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    sendPhotoToServer();
  }
  else
  {
    Serial.println("WiFi not connected!");
  }

  delay(10000); // Send a photo every 10 seconds
}