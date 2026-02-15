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

Servo servos[6];//155
double initialAngles[] = {30, 100, 100, 155, 6, 160};
double depthOffsetAngles[] = {-5, 1, 1, 0, 0, 0};
float servoAngles[6];

const char *ssid = "M-Tel_DF97";
const char *password = "4857544371DF9731";
// String sessionId = String(esp_random(), HEX);
// const String serverURL = "http://infiniScript.pythonanywhere.com/get_movements?session_id=" + sessionId;
String serverName = "http://192.168.100.13";
String serverIp = "192.168.100.13";
const String serverExtension = ":5000/get_movements";
uint8_t receiverMAC[] = {0x3C, 0x8A, 0x1F, 0xD5, 0x23, 0xFC};

bool isFirstImage = true;
volatile bool photoTakenFlag = false;
enum Command {
  NONE = 0,
  TAKE_PHOTO = 1,
  PHOTO_TAKEN = 2
};

enum State {
  IDLE,
  REQUESTING_PHOTO
};
State state = REQUESTING_PHOTO;

typedef struct{
  uint8_t command;
  float angles[6];
  char serverIp[32];
} struct_message;

struct_message dataToSend;
struct_message receivedData;

bool success = false;
double angles[6];
double receivedAngles[5];
double gripAngle = 70;
bool gripping = false;

unsigned long waitStart = 0;
unsigned long waitDuration = 0;
bool isWaiting = false;
bool isInControlMode = false;
bool isTakingPhoto = false;

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

  HTTPClient http;

  http.begin(serverName);
  http.setTimeout(500); 

  int httpResponseCode = http.GET();

  Serial.println("Connecting to server: " + serverName);

  if (httpResponseCode != -1) {
    Serial.println("Connection to " + serverName + " successful.");
    return;
  }
  Serial.println("Connection to " + serverName + " failed.");
}

void sendMessage(Command command, float sentAngles[], String serverIp){
  dataToSend.command = command;

  for (size_t i = 0; i < 6; i++) {
    dataToSend.angles[i] = sentAngles[i];
  }

  strncpy(dataToSend.serverIp,
        serverIp.c_str(),
        sizeof(dataToSend.serverIp) - 1);

  dataToSend.serverIp[sizeof(dataToSend.serverIp) - 1] = '\0';

  esp_now_send(receiverMAC, (uint8_t*)&dataToSend, sizeof(dataToSend));
}

void onReceive(const uint8_t *macAddr, const uint8_t *data, int len) {
  memcpy(&receivedData, data, sizeof(receivedData));

  if (receivedData.command == PHOTO_TAKEN) {
    photoTakenFlag = true;
  }
  
  Serial.print("Received command: ");
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

  servoGrip.attach(12);
  servoHeadJoint.attach(27);
  servoHeadRotate.attach(33);
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
  
  for (int i = 0; i < 6; i++)
  {
    servos[i].write(servoAngles[i]);
    // delay(100);
  }
}





bool receiveInstructions(std::vector<String> &commands, std::vector<std::vector<double>> &instructions)
{
  HTTPClient http;
  http.begin(serverName + serverExtension);

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

void checkForConnectedMode(){
  String data = Serial.readStringUntil('\n');
  if (data == "activate"){
    isInControlMode = true;
  }
}

void pickUp(){
  servos[5].write(gripAngle);
  bool gripping = true;
}

void release(){
  servos[5].write(160);
  bool gripping = false;
}

void setup()
{
  attachServos();

  initAndConnectWifi();

  initESPNow();

  initPosition();
  checkForConnectedMode();
  delay(1000);
  checkForConnectedMode();
}

void loop()
{
  if (!isInControlMode){
    if (gripping) {
      servos[5].write(gripAngle);
    }

    if (receivedData.command == PHOTO_TAKEN)
    {
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
            for (size_t j = 0; j < 5; j++)
            {
              angles[j] = instructions[i][j];
            }
            // if (isPositionUnsafe(angles))
            // {
            //   Serial.println("Position is unsafe, skipping move.");
            //   continue;
            // }

            rotateServos(angles);
          }
          else if (commands[i] == "grip")
          {
            if (instructions[i][0] == 1)
            {
              Serial.println("grip it");
              pickUp();
            }
            else if (instructions[i][0] == 0)
            {
              Serial.println("ungrip it");
              release();
            }
          }
          else if (commands[i] == "wait")
          {
            waitDuration = (unsigned long)instructions[i][0];
            waitStart = millis();
            isWaiting = true;

            Serial.printf("Waiting for %lu ms\n", waitDuration);

            while (millis() - waitStart < waitDuration) {
              if (gripping) {
                servos[5].write(gripAngle);
              }

              delay(5);
            }

            isWaiting = false;
          }
          else if (commands[i] == "initial")
          {
            rotateServos(initialAngles);
          }
        }
      }
      receivedData.command = NONE;
    }
    else
    {
      checkForConnectedMode();
      if(!isInControlMode)
      {
        sendMessage(TAKE_PHOTO, servoAngles, serverIp);
      }
    }
  }
  else
  {
    servos[5].write(servoAngles[5]);
    String command = Serial.readStringUntil('\n');
    command.trim();
    if(command == "deactivate"){
      isInControlMode = false;
      return;
    }
    if(command.startsWith("take_photo")){
      int separatorIndex = command.indexOf(':');
      serverIp = command.substring(separatorIndex + 1);

      isTakingPhoto = true;
    }
    if (receivedData.command == PHOTO_TAKEN)
    {
      receivedData.command = NONE;
      isTakingPhoto = false;
    }
    
    if (isTakingPhoto)
    {
      sendMessage(TAKE_PHOTO, servoAngles, serverName);
    }
    
    
    // format: S<id>:<angle>
    if (command.charAt(0) == 'S') {
      int colon = command.indexOf(':');
      int idx = command.substring(1, colon).toInt();
      int angle = command.substring(colon + 1).toInt();

      if (idx >= 0 && idx <= 5 && angle >= 0 && angle <= 180) {
        servoAngles[idx] = angle;
        servos[idx].write(angle);
        // delay(100);
        // servos[idx].write(angle);
        
        Serial.print("Servo ");
        Serial.print(idx);
        Serial.print(" set to ");
        Serial.print(angle);
        Serial.println(" degrees");
      } else {
        Serial.println("Invalid servo ID or angle");
      }
    }
    else if(command.charAt(0) == 'P'){

      int start = 1;

      for (int i = 0; i < 5; i++) {
        int nextColon = command.indexOf(':', start);

        if (nextColon == -1) {  
          // Last value (no more colons)
          receivedAngles[i] = command.substring(start).toInt();
        } else {
          receivedAngles[i] = command.substring(start, nextColon).toInt();
          start = nextColon + 1;
        }
      }

      rotateServos(receivedAngles);
    }
  }
}