#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Wire.h>
#include <oled.h>
#include <PID_v1.h>

#include <UrlTokenBindings.h>
#include <RichHttpServer.h>

// WiFi
#define WIFI_SSID     ssid
#define WIFI_PWD      pwd

// fans
#define PWM_PIN       2  // D4
#define TACH_PIN      14 // D5

// DS18B20 temp sensor
#define ONE_WIRE_BUS  12 // D6
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
double Temp_Out;

// DHT22 temp sensor
#define DHT_SENSOR_PIN  13 // D7
#define DHT_TYPE        DHT22
double Temp_In;
DHT dht(DHT_SENSOR_PIN, DHT_TYPE);

// SSD1306 display
#define SDA_PIN       4
#define SCL_PIN       5
#define I2C_ADDRESS   0x3C
#define RES_X         128
#define RES_Y         64
#define IS_SSD1306    true
OLED display(SDA_PIN, SCL_PIN, NO_RESET_PIN, I2C_ADDRESS, RES_X ,RES_Y, IS_SSD1306);

// PID controller
#define PID_P   20
#define PID_I   5
#define PID_D   5
#define CALCULATION_PERIOD 1000 //Number of milliseconds over which to interrupts
double SetPoint, PIDValue;
PID myPID(&Temp_Out, &PIDValue, &SetPoint, PID_P, PID_I, PID_D, P_ON_M, DIRECT);

volatile int interruptCounter; //counter use to detect hall sensor in fan
unsigned long previousMillis;    

using RichHttpConfig = RichHttp::Generics::Configs::EspressifBuiltin;
using RequestContext = RichHttpConfig::RequestContextType;
SimpleAuthProvider authProvider;
RichHttpServer<RichHttpConfig> server(80, authProvider);


void ICACHE_RAM_ATTR handleInterrupt() { //This is the function called by the interrupt
  if(digitalRead(TACH_PIN)==0) interruptCounter++;
}

void setup() {
  // init console
  Serial.begin(115200);

  DeviceAddress devAddr;
  if (!sensors.getAddress(devAddr, 0)) {
    Serial.println("No DS18B20 device found");
  } else {
    sensors.setResolution(devAddr, 11);
  }
  sensors.begin();
  dht.begin();

  // init temp & PID
  Temp_In = analogRead(0);
  SetPoint = 31;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,1023);
  myPID.SetSampleTime(CALCULATION_PERIOD);
  myPID.SetControllerDirection(REVERSE);

  // init display
  display.begin();

  WiFi.begin(WIFI_SSID, WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Waiting to connect...");
  }
  Serial.println(WiFi.localIP());

  server.buildHandler("/temp/in").on(HTTP_GET, handleTempInGET);
  server.buildHandler("/temp/out").on(HTTP_GET, handleTempOutGET);
  server.buildHandler("/speed").on(HTTP_GET, handleSpeedGET);
  server.buildHandler("/target").on(HTTP_GET, handleTargetGET);
  server.buildHandler("/target/:value").on(HTTP_PUT, handleTargetPUT);
  server.clearBuilders();
  server.begin();

  // init fan speed and reading
  previousMillis = 0;
  interruptCounter = 0;
  pinMode(TACH_PIN, INPUT_PULLUP);
  analogWriteFreq(10000);  
  analogWrite(PWM_PIN, 0); // start with lowest speed
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), handleInterrupt, FALLING);

}

void loop() {

  if((millis() - previousMillis) > CALCULATION_PERIOD) {// Process counters once every second
    previousMillis = millis();
    int count = interruptCounter;
    interruptCounter = 0;

    sensors.requestTemperatures();
    Temp_Out = sensors.getTempCByIndex(0);
    if (Temp_Out < 0) {
      Temp_Out = 0;
    }

    Temp_In = dht.readTemperature();
    if (isnan(Temp_In)) {
      Temp_In = 0;
    }

    if (myPID.Compute()) {
      analogWrite(PWM_PIN, PIDValue);
    }
    
    displayData(count);
    logData(count, PIDValue);
  }

  server.handleClient();

}

void displayData(int speed) {
  display.clear();
  
  char bufTempIn[25];
  sprintf(bufTempIn, "I: %.1f°C", Temp_In);
  display.draw_string(0, 0, bufTempIn, OLED::DOUBLE_SIZE);
  
  char bufTempOut[25];
  sprintf(bufTempOut, "O: %.1f°C", Temp_Out);
  display.draw_string(0, 24, bufTempOut, OLED::DOUBLE_SIZE);
  
  char bufSpeed[25];
  int rpm = 2 * speed / 3;
  sprintf(bufSpeed, "%d RPM", rpm);
  display.draw_string(0, 48, bufSpeed, OLED::DOUBLE_SIZE);
  display.display();
}

void logData(int speed, double pidValue) {
  Serial.print("TIn: ");
  Serial.print(Temp_In);
  Serial.print("°C / TOut: ");
  Serial.print(Temp_Out);
  Serial.print("°C / Speed: ");
  int rpm = 2 * speed / 3;
  Serial.print((int)rpm);
  Serial.print("RPM / ");
  Serial.println(pidValue);
}

void handleTempInGET(RequestContext& requestContext) {
  handleGetTemp(Temp_In);
}

void handleTempOutGET(RequestContext& requestContext) {
  handleGetTemp(Temp_Out);
}

void handleGetTemp(double temp) {
  char bufTemp[25];
  sprintf(bufTemp, "%.1f", temp);
  server.send(200, "text/plain", bufTemp);
}

void handleSpeedGET(RequestContext& requestContext) {
  char bufSpeed[25];
  int rpm = 2 * PIDValue / 3;
  sprintf(bufSpeed, "%d", rpm);
  server.send(200, "text/plain", bufSpeed);
}

void handleTargetGET(RequestContext& requestContext) {
  char bufSetPoint[25];
  sprintf(bufSetPoint, "%.1f", SetPoint);
  server.send(200, "text/plain", bufSetPoint);
}

void handleTargetPUT(RequestContext& request) {
  double value = atof(request.pathVariables.get("value"));
  Serial.print("New SetPoint: ");
  Serial.println(value);

  SetPoint = value;

  request.response.json["success"] = true;
  request.response.setCode(200);
  
}
