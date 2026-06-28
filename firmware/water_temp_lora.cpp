/*
 * Solar / LoRa water-temperature node  -  ESP8266 (Wemos D1 mini) + SX1262 (LoRa1262-868)
 * ---------------------------------------------------------------------------------------
 * Power model: a TPL5110 nano-timer powers the WHOLE board for a few seconds once an
 * hour, then physically cuts power (~35 nA standby) the instant we raise DONE. So this
 * file is "do everything once, then die" - there is almost nothing in loop().
 *
 * Flow each wake-up:
 *   1. kill WiFi radio (ESP8266 defaults to STA on boot -> ~70 mA wasted)
 *   2. read DS18B20 + battery voltage
 *   3. TX one LoRa P2P frame to the base station
 *   4. open a short RX window -> base replies with epoch time + config (optional)
 *   5. raise TPL5110 DONE -> board powers off. deepSleep() is only a fallback.
 *
 * Build: see firmware/platformio_lora.ini  (RadioLib + DallasTemperature + OneWire)
 *
 * SAFETY: we ALWAYS reach goToSleep(); if we never did, TPL5110 keeps the board
 * powered and drains the battery. Keep every early-return routed through it.
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>   // for WiFi.mode(OFF)/forceSleepBegin() to kill the radio
#include <RadioLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---- Pin map (bare ESP-12 / ESP8266 <-> LoRa1262 + e-paper + DS18B20) -------
// Final assignment that fits all peripherals — see hardware/CORRECTIONS.md.
// SPI bus is shared by LoRa + e-paper: SCK=GPIO14, MOSI=GPIO13, MISO=GPIO12.
#define PIN_LORA_NSS    16   // GPIO16  (CS, output-only pin is fine here)
#define PIN_LORA_DIO1   4    // GPIO4   (interrupt-capable)
#define PIN_LORA_BUSY   5    // GPIO5
#define PIN_LORA_RST    RADIOLIB_NC  // NRESET tied to board RST net (POR each wake)
#define PIN_EPD_CS      0    // GPIO0   (pull-up R5, high at boot)
#define PIN_EPD_DC      2    // GPIO2   (pull-up R4, high at boot)
#define PIN_EPD_RST     -1   // RES tied to board RST net
#define PIN_EPD_BUSY    -1   // BUSY not wired -> firmware waits a fixed delay
#define PIN_DS_DATA     3    // GPIO3 / RXD  (4.7k pull-up to 3V3)
#define PIN_TPL_DONE    15   // GPIO15  (held low at boot by 10k pulldown R1)
#define PIN_BATT_ADC    A0   // VSW -> R8(220k)/R7(68k) divider -> A0 (0..1V)

// ---- LoRa P2P radio config (must match the base station) --------------------
#define LORA_FREQ       868.0   // MHz (EU868)
#define LORA_BW         125.0   // kHz
#define LORA_SF         9       // spreading factor (range vs airtime/energy)
#define LORA_CR         7       // coding rate 4/7
#define LORA_SYNC       0x12    // private sync word (0x34 = public/LoRaWAN)
#define LORA_PWR        17      // dBm TX power
#define NODE_ID         1       // this node's address
#define RX_WINDOW_MS    800     // how long to listen for the base reply

// Battery calibration: A0 = VBAT * 68k/(220k+68k) = VBAT * 0.236, ESP ADC 0..1V.
// So full-scale (A0=1.0V) ~= 4.24V. Trim to match a multimeter.
#define BATT_FULLSCALE_V 4.24f

SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
OneWire oneWire(PIN_DS_DATA);
DallasTemperature sensors(&oneWire);

uint32_t baseEpoch = 0;   // last time/date learned from the base station (0 = unknown)

void goToSleep() {
  Serial.flush();
  radio.sleep();                 // put SX1262 in cold sleep (~600 nA)
  digitalWrite(PIN_TPL_DONE, HIGH);   // <-- tells TPL5110 to cut all power NOW
  delay(50);
  digitalWrite(PIN_TPL_DONE, LOW);
  // If no TPL5110 is fitted yet, fall back to ESP deep sleep (needs GPIO16->RST).
  ESP.deepSleep(0);              // 0 = sleep until external reset
}

float readTemp() {
  sensors.begin();
  sensors.setResolution(12);
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);   // -127 = sensor missing
}

float readBattery() {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) { acc += analogRead(PIN_BATT_ADC); delay(2); }
  float raw = acc / 16.0f;             // 0..1023
  return (raw / 1023.0f) * BATT_FULLSCALE_V;
}

void setup() {
  pinMode(PIN_TPL_DONE, OUTPUT);
  digitalWrite(PIN_TPL_DONE, LOW);     // keep ourselves powered while we work

  // ESP8266 boots into WiFi STA and burns ~70 mA - shut it down hard.
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();

  Serial.begin(115200);
  delay(20);

  float tempC = readTemp();
  float vbat  = readBattery();
  Serial.printf("\nT=%.2fC  Vbat=%.2fV\n", tempC, vbat);

  // ---- bring up SX1262 ----
  int st = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, LORA_PWR);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("LoRa init failed: %d\n", st);
    goToSleep();
  }

  // ---- build a compact binary frame ----
  //  [0]=NODE_ID  [1..2]=temp*100 (int16, LE)  [3..4]=vbat*1000 (uint16, LE)
  int16_t  t = (int16_t)(tempC * 100.0f);
  uint16_t v = (uint16_t)(vbat * 1000.0f);
  uint8_t  pkt[5] = { NODE_ID, (uint8_t)(t & 0xFF), (uint8_t)(t >> 8),
                      (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };

  st = radio.transmit(pkt, sizeof(pkt));
  Serial.printf("TX %s\n", st == RADIOLIB_ERR_NONE ? "ok" : "fail");

  // ---- RX window: base station may reply with epoch time + config ----
  uint8_t rx[16];
  st = radio.receive(rx, sizeof(rx));     // blocks up to its internal timeout
  if (st == RADIOLIB_ERR_NONE && rx[0] == NODE_ID) {
    // reply layout: [0]=NODE_ID [1..4]=unix epoch (uint32 LE) [5]=config flags
    baseEpoch = (uint32_t)rx[1] | ((uint32_t)rx[2] << 8) |
                ((uint32_t)rx[3] << 16) | ((uint32_t)rx[4] << 24);
    Serial.printf("base epoch=%lu cfg=0x%02X\n", (unsigned long)baseEpoch, rx[5]);
    // The base station decides the "measurement window": it can answer with a
    // flag telling this node to stay quiet, or you just let it log hourly.
  } else {
    Serial.println("no base reply (offline / out of range)");
  }

  goToSleep();   // always - never let TPL5110 keep us alive
}

void loop() { /* never reached: goToSleep() powers the board off */ }
