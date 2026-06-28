# Solar / LoRa Water-Temperature Node — Hardware Design

ESP8266 (Wemos D1 mini) + **SX1262** (G-NiceRF **LoRa1262-868**) + DS18B20, run from a
1S 18650 charged by a solar panel. Measures once an hour, sends one LoRa frame
**point-to-point** to your base station, and gets time/date back in the reply.

---

## 1. Why the old build drained the battery

| Cause | Old design | Cost |
|---|---|---|
| Dev board housekeeping | ESP32 DevKit on-board USB-serial + AMS1117 LDO | **10–20 mA *continuous*, even in deep sleep** |
| Wrong radio | WiFi connect + MQTT every cycle | 3–10 s @ ~120–180 mA per send |
| Wrong sleep time | `ESP.deepSleep(60e6)` = **60 s**, not 1 h | 60× too many wake-ups |
| Bare deep sleep | best case ~10 µA, but never zero | always leaking something |

The single biggest fix is **stop trusting deep sleep and physically cut power** between
samples. A **TPL5110** nano-timer does exactly that: it powers the board for a few
seconds once an hour, and the moment firmware raises `DONE` it disconnects the load and
idles at **~35 nA**.

---

## 2. Architecture

```
 Solar 6V ─► U1 CN3065 ─► VBAT(18650) ─┬─► U2 TPL5110 ──(DRV)──► Q1 P-FET ─► VSW
 (charger, MPPT-ish)                    │     ▲  hourly timer        │
                                        │     └──(DONE)──────────────┼─ ESP8266 D8/GPIO15
                                        │                            │
                                        └─► R2 100k ─► A0 (Vbat ADC)  └─► U3 LDO 3V3 ─┬─ ESP8266 (3V3)
                                                                                      ├─ LoRa1262 (VCC)
                                                                                      └─ DS18B20 (VDD)
 ESP8266  ──SPI(SCK/MOSI/MISO)+NSS/BUSY/DIO1/NRESET──► LoRa1262 (SX1262) ─► antenna
 ESP8266  ──GPIO2(1-wire, 4.7k pull-up)─────────────► DS18B20 probe
```

**Operating sequence (once per hour):** TPL5110 fires → Q1 connects VBAT to the LDO →
ESP8266 boots → kills WiFi → reads temp + battery → SX1262 TX one frame → ~0.8 s RX
window for the base station's time/config reply → raises `DONE` → board powers off.

---

## 3. Power budget

| Phase | Current | Time | Charge / hour |
|---|---|---|---|
| Boot + read DS18B20 | ~25 mA | ~1.0 s | 0.007 mAh |
| LoRa TX (SF9, +17 dBm) | ~95 mA | ~0.15 s | 0.004 mAh |
| RX window | ~12 mA | ~0.8 s | 0.003 mAh |
| **Standby (TPL5110)** | **~0.035 µA** | rest of hour | ~0 |
| **Total** | | | **≈ 0.015 mAh/hr → ~0.35 mAh/day** |

A 3000 mAh 18650 alone would last **years**; the solar panel makes it effectively
perpetual. (Add margin for self-discharge and cold.) Compare to the old WiFi build,
which spent **~250 mAh/day** — that's the ~700× improvement.

> If you skip the TPL5110 and use only ESP8266 deep sleep, standby rises to ~20 µA and
> the max sleep is ~71 min (so hourly is borderline). **The TPL5110 is the recommended
> path**; deep sleep is the fallback that `goToSleep()` already calls.

---

## 4. Net list (pin → net)

`U4` = Wemos D1 mini, `U5` = LoRa1262, `U2` = TPL5110, `Q1` = AO3401 P-FET,
`U3` = ME6211C33 LDO, `U1` = CN3065 charger.

| Net | Connections |
|---|---|
| `VBAT` | BT1+, U1.BAT, U2.VDD, Q1.S, R4→PG, R2(→A0), C1 |
| `VSW` | Q1.D, U3.VIN, U3.EN, C2 |
| `3V3` | U3.VOUT, U4.3V3, U5.VCC, J2.VDD(DS18B20), R1(→DS_DATA), C3, C4 |
| `GND` | everything's ground / 18650−, panel− |
| `SOLAR_IN` | J1.IN, U1.VIN |
| `PG` (FET gate) | U2.DRV, Q1.G, R4(→VBAT) |
| `TPL_DONE` | U2.DONE, U4.D8/GPIO15 |
| `TDELAY` | U2.DELAY, R3(→GND)  — sets the 1 h period |
| `BATT_ADC` | U4.A0, R2(→VBAT) |
| `SCK` | U4.D5/GPIO14 → U5.SCK |
| `MISO` | U4.D6/GPIO12 → U5.MISO |
| `MOSI` | U4.D7/GPIO13 → U5.MOSI |
| `LORA_NSS` | U4.D3/GPIO0 → U5.NSS |
| `LORA_BUSY` | U4.D2/GPIO4 → U5.BUSY |
| `LORA_DIO1` | U4.D1/GPIO5 → U5.DIO1 |
| `LORA_RST` | U4.D0/GPIO16 → U5.NRESET |
| `LORA_ANT` | U5.ANT → J3 (u.FL / spring antenna) |
| `DS_DATA` | U4.D4/GPIO2 → J2.DATA, R1(→3V3) |

**Boot-strapping notes (ESP8266):** GPIO15 (`TPL_DONE`) must be **low at boot** — the
Wemos has a 10k pulldown, perfect. GPIO0 (`LORA_NSS`) and GPIO2 (`DS_DATA`) must be
**high at boot** — NSS idles high and the 1-wire pull-up holds DATA high, so both are
fine. Don't reassign these without re-checking the strap levels.

### TPL5110 period resistor (R3)
Set `R3` per the TPL5110 datasheet "Time Interval vs. R" curve. ~1 hour is near the part's
max (~7200 s) — use the top end of the resistor range and **verify on the bench** with a
shorter interval first. A trimpot in the R3 spot makes tuning easy.

---

## 5. BOM (search the LCSC number in EasyEDA → footprint comes automatically)

| Ref | Part | LCSC | Notes |
|---|---|---|---|
| U4 | Wemos D1 mini (ESP8266) | (module, 2×8 headers) | you already have it |
| U5 | LoRa1262-868 (SX1262) | (G-NiceRF module) | you already have it |
| U1 | CN3065 solar Li-ion charger | C347465 | ESOP-8; or CN3791 for true MPPT |
| U2 | TPL5110DDCR nano-timer | C92462 | SOT-23-6 |
| U3 | ME6211C33M5G LDO 3.3V | C82942 | SOT-23-5, low-Iq, 500 mA |
| Q1 | AO3401 P-MOSFET | C15127 | SOT-23 load switch |
| R1 | 4.7k (1-wire pull-up) | — | 0603 |
| R2 | 100k (VBAT→A0) | — | 0603 |
| R3 | ~170k / trimpot (TPL delay) | — | set for 1 h |
| R4 | 100k (FET gate pull-up) | — | 0603 |
| R5 | 1.5k (CN3065 ISET ≈ 500 mA) | — | 0603 |
| C1,C2,C4 | 10 µF | — | 0805 |
| C3 | 10 µF + 100 nF | — | LDO out / decoupling |
| BT1 | 18650 holder | — | keyed |
| J1 | Solar 6 V / 1–2 W + 2-pin | — | reverse-protect optional |
| J2 | DS18B20 probe, 3-pin | — | waterproof probe |
| J3 | u.FL or spring antenna | — | 868 MHz |

---

## 6. Base-station / time-sync protocol (P2P, not LoRaWAN)

Matches the firmware in `firmware/water_temp_lora.cpp`:

- **Uplink (node→base), 5 bytes:** `[NODE_ID][temp×100 int16 LE][vbat×1000 uint16 LE]`
- **Downlink (base→node), ≥6 bytes:** `[NODE_ID][unix_epoch uint32 LE][cfg flags]`

The node has no battery-backed clock (TPL5110 cuts everything), so it **re-learns the
time from the base on every wake**. The "measure only within a time window" rule lives on
the **base station**: it timestamps each hourly frame and keeps/ignores per your window —
or sets a `cfg` flag the node can read. (If you ever need the node to self-gate without
the radio, add a PCF8563 RTC on `VBAT` + I²C; noted but not required.)

---

## 7. EasyEDA file

`hardware/easyeda_schematic.json` is a **net-label documentation schematic** (generated by
`gen_easyeda.py`). Import it: EasyEDA Std → *File ▸ Import ▸ EasyEDA* (or open the JSON).
Every part is a labelled block; **pins with the same red net name are the same wire.**

It deliberately uses generic blocks (no fake footprints — those need EasyEDA library
UUIDs that can't be hand-authored reliably). **To route a PCB:** create a new EasyEDA
schematic, place the real parts from the BOM by their **LCSC number** (footprints attach
automatically), wire them to match the net list above using this JSON as the picture,
then *Convert to PCB*. Re-run `python3 gen_easyeda.py` if you tweak the layout.
