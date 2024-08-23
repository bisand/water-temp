
#include "Arduino.h"
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <iostream>
#include <vector>

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

// Define a struct to represent the mapping table entries
struct MappingEntry
{
  int analogValue;
  float voltage;
};

// Analog to voltage mapping table
std::vector<MappingEntry> voltageMappingTable = {
    {0, 0.0f},
    {1711, 2.764f},
    {2246, 3.156f},
    {2397, 3.3f},
    {2505, 3.467f},
    {2544, 3.516f},
    {2642, 3.61f},
    {3060, 4.06f},
    {3250, 4.15f}};

// Function to map analog reading to voltage using linear interpolation
float mapAnalogToVoltage(int analogReading, const std::vector<MappingEntry> &table)
{
  for (size_t i = 0; i < table.size() - 1; ++i)
  {
    int x0 = table[i].analogValue;
    float y0 = table[i].voltage;
    int x1 = table[i + 1].analogValue;
    float y1 = table[i + 1].voltage;

    if (analogReading >= x0 && analogReading <= x1)
    {
      // Linear interpolation
      float voltage = y0 + (analogReading - x0) * (y1 - y0) / (x1 - x0);
      return voltage;
    }
  }
  // If the reading is out of range, return the closest boundary value
  if (analogReading < table[0].analogValue)
  {
    return table[0].voltage;
  }
  else
  {
    return table[table.size() - 1].voltage;
  }
}

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

float read_temp()
{
  float rawHigh = 100.0;
  float rawLow = 0.75;
  float referenceHigh = 100.0;
  float referenceLow = 0.0;
  float rawRange = rawHigh - rawLow;
  float referenceRange = referenceHigh - referenceLow;

  int readings = 1;

  sensors.begin();
  sensors.setResolution(12);

  float accumulatedValue = 0.0;
  for (int i = 0; i < readings; i++)
  {
    // New temperature readings
    sensors.requestTemperatures();
    // Temperature in Celsius degrees
    accumulatedValue += sensors.getTempCByIndex(0);
  }
  float rawValue = accumulatedValue / readings;

  float correctedValue = (((rawValue - rawLow) * referenceRange) / rawRange) + referenceLow;
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

  // Calculate battery level where 2.9v is 0% and 4.15v is 100%
  float netBatteryLevelReal = (float)((voltage - 2.9f) / (4.15f - 2.9f)) * 100.0f;

  snprintf(msg, MSG_BUFFER_SIZE, "{\"t\":%.2f,\"v\":%.2f,\"bg\":%.2f,\"bn\":%.2f,\"a\":%i}", temp, voltage, batteryLevel, netBatteryLevelReal, rawVoltage);
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
  delay(500);

  int samples = 20;
  float temp = read_temp();
  int analogReading = 0;
  for (int i = 0; i < samples; i++)
  {
    analogReading += analogRead(BATTERY_VOLTAGE_PIN);
    delay(10);
  }
  analogReading /= samples;

  // 4.15v = 2.615v = 3250 raw value from ADC using voltage divider 33k and 55k
  // float voltage = (float)map(analogReading, 0, 3060, 0, 406) / 100.0;
  float voltage = mapAnalogToVoltage(analogReading, voltageMappingTable);
  float grossBatteryLevel = (voltage - 0.0f) / (4.2f - 0.0f) * 100.0f;

  String host = read_sec_from_file("mqtt_host");
  int port = 1883;

  client.setServer(host.c_str(), port);

  digitalWrite(POWER_TRIGGER_PIN, LOW);

  Serial.printf("Battery level: %.2f\n", grossBatteryLevel);
  Serial.printf("Raw voltage: %d\n", analogReading);
  Serial.printf("Voltage: %.2f\n", voltage);

  setup_wifi();
  publish(temp, voltage, analogReading, grossBatteryLevel);
  Serial.flush();
  ESP.deepSleep(60e6);
}

void loop()
{
}
