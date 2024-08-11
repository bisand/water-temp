
#include "Arduino.h"
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>

#define POWER_TRIGGER_PIN 32   // Power trigger pin
#define DS_PIN 4               // DS18B20 data pin
#define BATTERY_VOLTAGE_PIN 34 // Battery voltage pin

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "vanntemp/klopp/temperature"

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (128)
char msg[MSG_BUFFER_SIZE];
int value = 0;

OneWire oneWire(DS_PIN);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

double rawHigh = 100.0;
double rawLow = 0.75;
double referenceHigh = 100.0;
double referenceLow = 0.0;
double rawRange = rawHigh - rawLow;
double referenceRange = referenceHigh - referenceLow;

int readings = 1;

String read_sec_from_file(const char *sec_file)
{
  SPIFFS.begin();

  // Calculate the required length for filePath
  size_t len = strlen("/sec/") + strlen(sec_file) + 1; // +1 for the null terminator

  // Allocate memory for the concatenated string
  char *filePath = (char *)malloc(len);
  if (filePath == nullptr)
  {
    Serial.println("Failed to allocate memory");
    return "";
  }

  // Copy and concatenate the strings
  strcpy(filePath, "/sec/");
  strcat(filePath, sec_file);

  // Open the file
  File file = SPIFFS.open(filePath, "r");
  free(filePath); // Free the allocated memory

  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return "";
  }

  String data = file.readString();
  file.close();
  SPIFFS.end();

  return data;
}

double read_temp()
{
  sensors.begin();
  sensors.setResolution(12);

  double accumulatedValue = 0.0;
  for (int i = 0; i < readings; i++)
  {
    // New temperature readings
    sensors.requestTemperatures();
    // Temperature in Celsius degrees
    accumulatedValue += sensors.getTempCByIndex(0);
  }
  double rawValue = accumulatedValue / readings;

  double correctedValue = (((rawValue - rawLow) * referenceRange) / rawRange) + referenceLow;
  Serial.printf("\nTemperature: %.1f °C (Raw: %.1f)\n", correctedValue, rawValue);

  return correctedValue;
}

void setup_wifi()
{
  String ssid = read_sec_from_file("wifi_ssid");
  String pwd = read_sec_from_file("wifi_pwd");

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void publish(float temp, float voltage, int rawVoltage, float batteryLevel)
{
  if (!client.connected())
  {
    String user = read_sec_from_file("mqtt_usr");
    String password = read_sec_from_file("mqtt_pwd");
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client", user.c_str(), password.c_str()))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      return;
    }
  }

  // Calculate battery level where 3.3v is 0% and 4.2v is 100%
  float batteryLevelReal = (float)(voltage - 3.4f) / (4.15f - 3.4f) * 100.0f;

  snprintf(msg, MSG_BUFFER_SIZE, "{\"t\":%.2f,\"v\":%.2f,\"bg\":%.2f,\"bn\":%.2f,\"a\":%i}", temp, voltage, batteryLevel, batteryLevelReal, rawVoltage);
  Serial.print("Publish message: ");
  Serial.println(msg);
  client.publish(MQTT_PUB_TEMP, msg);
  client.disconnect();
}

void setup()
{
  analogReadResolution(12);
  Serial.begin(115200);
  Serial.setTimeout(2000);

  // // initialize LED digital pin as an output.
  pinMode(POWER_TRIGGER_PIN, OUTPUT);
  // pinMode(DS_PIN, INPUT_PULLDOWN);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT_PULLDOWN);

  // Wait for serial to initialize.
  while (!Serial)
  {
  }

  digitalWrite(POWER_TRIGGER_PIN, HIGH);
  delay(100);

  float temp = read_temp();
  int rawVoltage = 0;
  for (int i = 0; i < 10; i++)
  {
    rawVoltage += analogRead(BATTERY_VOLTAGE_PIN);
    delay(10);
  }
  rawVoltage /= 10;

  // 4.15v = 2.615v = 3250 raw value from ADC using voltage divider 33k and 55k
  float voltage = (float)map(rawVoltage, 0, 3250.0f, 0, 415) / 100.0;
  float batteryLevel = (float)map(rawVoltage, 0, 3250.0f, 0, 10000) / 100.0;

  String host = read_sec_from_file("mqtt_host");
  int port = 1883;

  client.setServer(host.c_str(), port);

  digitalWrite(POWER_TRIGGER_PIN, LOW);

  Serial.printf("Battery level: %.2f\n", batteryLevel);
  Serial.printf("Raw voltage: %d\n", rawVoltage);
  Serial.printf("Voltage: %.2f\n", voltage);

  setup_wifi();
  publish(temp, voltage, rawVoltage, batteryLevel);
  Serial.flush();
  ESP.deepSleep(300e6);
}

void loop()
{
}
