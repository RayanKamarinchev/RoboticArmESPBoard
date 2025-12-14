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
double angles[3];
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

void sendMessage(Command command){
  dataToSend.command = command;

  esp_now_send(receiverMAC, (uint8_t *) &dataToSend, sizeof(dataToSend));
}

void onReceive(const uint8_t *macAddr, const uint8_t *data, int len) {
  memcpy(&receivedData, data, sizeof(receivedData));
  Serial.print("Received: ");
  Serial.println(receivedData.command);
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
    delay(100);
  }
}





bool receiveInstructions(std::vector<String> &commands, std::vector<std::vector<double>> &instructions)
{
  HTTPClient http;
  http.begin(serverURL);

  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200) {
    String payload = http.getString();

    StaticJsonDocument<200> doc;
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
  newAngles[1] = angles[0]+7;
  newAngles[2] = 215-angles[1];
  newAngles[4] = angles[2]-74;

  newAngles[0] = servoAngles[0];
  newAngles[3] = servoAngles[3];
  newAngles[5] = servoAngles[5];
}

bool isPositionUnsafe(double angles[])
{
  double alpha = angles[0];
  double beta = angles[1];
  double gamma = angles[2];
  Serial.printf("Angles: %f %f %f\n", alpha, beta, gamma);

  double a = 0.117;
  double b = 0.122;
  double c = 0.127;

  double x = a * cos(alpha) + b * cos(alpha + beta) + c * cos(alpha + beta + gamma);
  double z = a * sin(alpha) + b * sin(alpha + beta) + c * sin(alpha + beta + gamma);
  Serial.printf("Position: x=%f, z=%f\n", x, z);

  return z < 0 || x < 0.05;
}

void rotateServos(double newAngles[], int moveTime = 1000, int stepTime = 20)
{
  int steps = moveTime / stepTime;
  if (steps < 1) steps = 1;

  double delta[6];
  double increment[6];

  for (int i = 0; i < 6; i++) {
      delta[i] = newAngles[i] - servoAngles[i];
      increment[i] = delta[i] / steps;
      printf("Servo %d: from %f to %f, delta %f, increment %f\n", i, servoAngles[i], newAngles[i], delta[i], increment[i]);
  }

  for (int s = 0; s <= steps; s++) {
      for (int i = 0; i < 6; i++) {
          double targetAngle = servoAngles[i] + increment[i] * s;

          // if (targetAngle < 0) targetAngle = 0;
          // if (targetAngle > 180) targetAngle = 180;

          servos[i].write(targetAngle);
      }
      delay(stepTime);
  }

  for (int i = 0; i < 6; i++) {
      servoAngles[i] = newAngles[i];
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
  
  sendMessage(TAKE_PHOTO);
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
          angles[0] = instructions[i][0] * 180 / M_PI;
          angles[1] = instructions[i][1] * 180 / M_PI;
          angles[2] = instructions[i][2] * 180 / M_PI;
          getServoAngles(angles, newAngles);
          
          if (isPositionUnsafe(angles))
          {
            Serial.println("Position is unsafe, skipping move.");
            continue;
          }

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

  // for (int i = 270; i >= 90; i-=10)
  // {
  //   servoJoint1.write(i);
  //   delay(500);
  // }
  // for (int i = 10; i < ; i++)
  // {
  //   /* code */
  // }

  // servoHead1.write(90);
  // servoJoint1.write(200);
}