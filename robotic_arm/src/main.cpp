#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>

//servos
Servo servoGrip;
Servo servoHeadJoint;
Servo servoHeadRotate;
Servo servoJoint2;
Servo servoJoint1;
Servo servoBase;

Servo servos[6];
double initialAngles[] = {30, 90+10, 90 + 15, 150, 0+6, 170};
double servoAngles[6];

const char *ssid = "M-Tel_DF97";
const char *password = "4857544371DF9731";
// String sessionId = String(esp_random(), HEX);
// const String serverURL = "http://infiniScript.pythonanywhere.com/get_movements?session_id=" + sessionId;
const char* serverURL = "http://infiniScript.pythonanywhere.com/get_movements";
uint8_t receiverMAC[] = {0x3C, 0x8A, 0x1F, 0xD5, 0x23, 0xFC};

enum Command {
  NONE = 0,
  TAKE_PHOTO = 1,
  PHOTO_TAKEN = 2
};

typedef struct{
  uint8_t command;
} struct_message;

struct_message dataToSend;
struct_message receivedData;

bool success = false;
double angles[6];
double newAngles[6];

void initAndConnectWifi()
{
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
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

void sendMessage(Command command){
  dataToSend.command = command;

  esp_now_send(receiverMAC, (uint8_t *) &dataToSend, sizeof(dataToSend));
}

void onReceive(const uint8_t *macAddr, const uint8_t *data, int len) {
  memcpy(&receivedData, data, sizeof(receivedData));
  Serial.print("Received: ");
  Serial.println(receivedData.command);
}

void initESPNow(){
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(onReceive);
}

void attachServos(){
   // servo.setPeriodHertz(50);    // standard 50 hz servo
  // 1 33 110-180
  // 2 27
  // 3 13
  // 4 26 inv 0-176
  // 5 14
  // 6 25

  servoGrip.attach(33);
  servoHeadJoint.attach(27);
  servoHeadRotate.attach(12);
  servoJoint2.attach(26);
  servoJoint1.attach(14);
  servoBase.attach(25);
  servos[0] = servoBase;
  servos[1] = servoJoint1;
  servos[2] = servoJoint2;
  servos[3] = servoHeadRotate;
  servos[4] = servoHeadJoint;
  servos[5] = servoGrip;
  Serial.begin(9600);
}

void initPosition(){
  for (int i = 0; i < 6; i++)
  {
    servoAngles[i] = initialAngles[i];
  }
  
  for (int i = 0; i <= 5; i++)
  {
    servos[i].write(servoAngles[i]);
    // delay(100);
  }
  delay(1000);
}





bool receiveInstructions(std::vector<String> &commands, std::vector<std::vector<double>> &instructions)
{
  HTTPClient http;
  http.begin(serverURL);

  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200) {
    String payload = http.getString();

    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, payload);
    Serial.println("Received instructions:");
    Serial.println(payload);
    // check if doc is empty
    if (!error) {
      JsonArray rows = doc.as<JsonArray>();
      for (JsonArray row : rows) {
        String command = row[0].as<String>();
        commands.push_back(command);

        std::vector<double> values;
        for (int i = 1; i < row.size(); i++)
        {
          values.push_back(row[i].as<double>());
        }
        instructions.push_back(values);
      }

      if (rows.size()==0)
      {
        Serial.println("No instructions received");
        return false;
      }
      

    } else {
      Serial.println("JSON parse error");
    }
  } else {
    Serial.printf("HTTP error: %d\n", httpResponseCode);
    return false;
  }
  http.end();
  return true;
}

void getServoAngles(double angles[], double *newAngles)
{
  newAngles[0] = 30-angles[0];
  newAngles[1] = angles[1]+10; //7 10
  newAngles[2] = 214-angles[2];//215 214
  newAngles[3] = 150-angles[3];
  newAngles[4] = angles[4]-73;//74 73
}

void rotateServos(double newAngles[], int moveTime = 1000, int stepTime = 20)
{
  const int NUM_SERVOS = 5;

  double startAngles[NUM_SERVOS];
  double delta[NUM_SERVOS];

  for (int i = 0; i < NUM_SERVOS; i++) {
      startAngles[i] = servoAngles[i];
      delta[i] = newAngles[i] - startAngles[i];

      printf(
          "Servo %d: from %f to %f, delta %f\n",
          i, startAngles[i], newAngles[i], delta[i]
      );
  }

  unsigned long startTime = millis();

  while (true) {
      unsigned long elapsed = millis() - startTime;
      if (elapsed >= moveTime) break;

      double t = (double)elapsed / (double)moveTime;

      double s = 0.5 * (1.0 - cos(PI * t));

      for (int i = 0; i < NUM_SERVOS; i++) {
          double angle = startAngles[i] + s * delta[i];

          servos[i].write(angle);
      }

      delay(15);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
      servoAngles[i] = newAngles[i];
      servos[i].write(newAngles[i]);
  }
}

void pickUp(){
  for (int i = 170; i >= 110; i-=10)
  {
    servos[5].write(i);
    delay(100);
  }
}

void release(){
  servos[5].write(170);
}

void setup()
{
  attachServos();

  initAndConnectWifi();

  initESPNow();

  initPosition();
}

void loop()
{
  if (receivedData.command == PHOTO_TAKEN) {
    std::vector<String> commands;
    std::vector<std::vector<double>> instructions;
    success = receiveInstructions(commands, instructions);
    if (success)
    {
      for (int i = 0; i < commands.size(); i++)
      {
        Serial.printf("Command: %s\n", commands[i].c_str());
        Serial.printf("Values: ");
        for (int j = 0; j < instructions[i].size(); j++)
        {
          Serial.printf("%f ", instructions[i][j]);
        }
        Serial.println();

        if (commands[i] == "move")
        {
          angles[0] = instructions[i][0];
          angles[1] = instructions[i][1];
          angles[2] = instructions[i][2];
          angles[3] = instructions[i][3];
          angles[4] = instructions[i][4];
          getServoAngles(angles, newAngles);
          // if (isPositionUnsafe(angles))
          // {
          //   Serial.println("Position is unsafe, skipping move.");
          //   continue;
          // }

          rotateServos(newAngles, 2000);
        }
        else if (commands[i] == "grip")
        {
          if (instructions[i][0] == 1)
          {
            pickUp();
          }
          else if (instructions[i][0] == 0)
          {
            release();
          }
        }
        else if (commands[i] == "wait")
        {
          int waitTime = (int)instructions[i][0]*1000;
          Serial.printf("Waiting for %d ms\n", waitTime);
          delay(waitTime);
        }
        else if (commands[i] == "initial")
        {
          rotateServos(initialAngles, 2000);
        }
      }
    }
    receivedData.command = NONE;
  }
  else
  {
    sendMessage(TAKE_PHOTO);
  }
}