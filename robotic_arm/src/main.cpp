#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

Servo servoGrip;
Servo servoHead2;
Servo servoHead1;
Servo servoJoint2;
Servo servoJoint1;
Servo servoBase;
int angle = 0;
Servo servos[6];
double servoAngles[] = {90, 90, 35, 150, 0, 170};
const char *ssid = "M-Tel_DF97";
const char *password = "4857544371DF9731";
const char *serverURL = "http://infiniScript.pythonanywhere.com/get_movements";

void getServoAngles(double angles[], double *changes)
{
  changes[1] = angles[0]+7;
  changes[2] = 215-angles[1];
  changes[4] = angles[2]-74;
}

void rotateServos(double changes[])
{
  changes[1] -= servoAngles[1];
  changes[2] -= servoAngles[2];
  changes[4] -= servoAngles[4];

  Serial.println(changes[1]);
  Serial.println(changes[2]);
  Serial.println(changes[4]);
  delay(2000);

  int time = 20;
  double steps[6];
  int delayTime = 20;
  for (int i = 0; i <= 5; i++)
  {
    steps[i] = changes[i] * delayTime / time;
  }

  for (int i = 0; i <= time / delayTime; i++)
  {
    for (int j = 0; j <= 4; j++)
    {
      Serial.println(String(j) + "----" + String(servoAngles[j] + steps[j] * i));
      servos[j].write(servoAngles[j] + steps[j] * i);
    }
    delay(delayTime);
  }
  for (int i = 0; i <= 4; i++)
  {
    servoAngles[i] += changes[i];
  }
}

bool getAngleChanges(double x, double y, double z, double *outValues)
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected");
    return false;
  }

  String url = String(serverURL) +
               "?x=" + String(x) +
               "&y=" + String(y) +
               "&z=" + String(z);

  HTTPClient http;
  http.begin(url);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0)
  {
    String response = http.getString();
    Serial.println(response);
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, response);

    if (error)
    {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      http.end();
      return false;
    }

    for (int i = 0; i < 3; i++)
    {
      outValues[i] = doc[i];
      Serial.printf("Value %d: %f\n", i + 1, outValues[i]);
    }

    http.end();
    return true;
  }
  else
  {
    Serial.print("HTTP GET failed. Error code: ");
    Serial.println(httpResponseCode);
    http.end();
    return false;
  }
}

void pickUp(){
  for (int i = 170; i >= 110; i-=10)
  {
    servos[5].write(i);
    delay(100);
  }
}

void setup()
{
  // servo.setPeriodHertz(50);    // standard 50 hz servo
  // 1 33 110-180
  // 2 27
  // 3 13
  // 4 26 inv 0-176
  // 5 14
  // 6 25
  servoGrip.attach(33);
  servoHead2.attach(27);
  servoHead1.attach(12);
  servoJoint2.attach(26);
  servoJoint1.attach(14);
  servoBase.attach(25);
  servos[0] = servoBase;
  servos[1] = servoJoint1;
  servos[2] = servoJoint2;
  servos[3] = servoHead1;
  servos[4] = servoHead2;
  servos[5] = servoGrip;
  Serial.begin(9600);

  delay(1000);
  for (int i = 0; i <= 5; i++)
  {
    servos[i].write(servoAngles[i]);
    delay(100);
  }

  double angles[3];
  double changes[6];
  getAngleChanges(20, 0, 2, angles);
  getServoAngles(angles, changes);
  rotateServos(changes);

  // delay(2000);
  // pickUp();

  // getAngleChanges(10, 0, 10, angles);
  // getServoAngles(angles, changes);
  // rotateServos(changes);
  // delay(1000);
  // servos[5].write(170);




  // for (int i = 0; i <= 5; i++)
  // {
  //   rotateServo(i, servoAngles[i]);
  // }

  // servoJoint2.write(0);
  // rotateServo(2, 90);
}

void loop()
{

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