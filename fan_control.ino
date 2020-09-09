#include <ESP8266WiFi.h>

// WifiManager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

// Temp sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Display
#include <Wire.h>
#include "OLED.h"

// MQTT
#include <PubSubClient.h>

#define INTERVAL      5000
#define FAN_PIN       4
#define ONE_WIRE_BUS  2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

OLED display(2, 14);

WiFiClient espClient;
PubSubClient client(espClient);

const char* MQTT_BROKER = "BROKER_IP";
const int MQTT_BROKER_PORT = 1883;

int lastMillis = 0;
float set_temperature = 50.0;

void setup() {
  Serial.begin(115200);

  sensors.begin();

  WiFiManager wifiManager;

  wifiManager.setBreakAfterConfig(true);

  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  client.setServer(MQTT_BROKER, MQTT_BROKER_PORT);

  display.begin();
}

void loop() {

  if (abs(millis() - lastMillis) > INTERVAL) {
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    int pidValue = calcPID(temp);
    
    analogWrite(FAN_PIN, 1023 - pidValue);

    displayData(temp, pidValue);
    logData(temp, pidValue);
    publishData(temp, pidValue);


    lastMillis = millis();
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Reconnecting to MQTT broker...");
    if (!client.connect("ProjectorClient")) {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" retrying in 5 seconds");
        delay(5000);
    }
  }
}

void displayData(float temp, int speed) {
  char buf[25];
  sprintf(buf, "Current temp: %f", temp);
  display.clear();
  display.print(buf);
}

void logData(float temp, int speed) {
  Serial.print("Current Temperature: ");
  Serial.println(temp);

  Serial.print("Current Fanspeed: ");
  Serial.println(speed);
}

void publishData(float temp, int speed) {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  char tempMsg[3];
  sprintf(tempMsg, "%f", temp);
  client.publish("/projector/temp", tempMsg);

  char pidMsg[4];
  sprintf(pidMsg, "%d", speed);
  client.publish("/projector/fan/speed", pidMsg);
}

////////////////////////////////////////////////////////////////
// from https://www.electronoobs.com/eng_arduino_tut24_code3.php

//PID constants
///////////////////////////////////////////////////////////////
int kp = 90;
int ki = 30;
int kd = 80;
///////////////////////////////////////////////////////////////
float elapsedTime, Time, timePrev;
float temperature_read;
float previous_error = 0;
float PID_value = 0;
float PID_error = 0;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int calcPID(float currentTemp) {
  // First we read the real value of temperature
  temperature_read = currentTemp;
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read + 3;
  //Calculate the P value
  PID_p = 0.01 * kp * PID_error;
  //Calculate the I value in a range on +-3
  PID_i = 0.01 * PID_i + (ki * PID_error);


  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000;
  //Now we can calculate the D calue
  PID_d = 0.01 * kd * ((PID_error - previous_error) / elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = (PID_p + PID_i + PID_d) * 4;

  //We define PWM range between 0 and 255
  if(PID_value < 0)
  {
    PID_value = 0;
  }
  if(PID_value > 1023)
  {
    PID_value = 1023;
  }
  //Now we can write the PWM signal to the mosfet on digital pin D3
  //Since we activate the MOSFET with a 0 to the base of the BJT, we write 255-PID value (inverted)
  previous_error = PID_error;     //Remember to store the previous error for next loop.

  return PID_value;
}
