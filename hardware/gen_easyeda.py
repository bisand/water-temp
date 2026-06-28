#!/usr/bin/env python3
"""
Generate an EasyEDA *Standard* edition schematic source file (.json) — a NET-LABEL
REFERENCE DIAGRAM of the corrected solar/LoRa water-temp node.

This is NOT the PCB schematic. EasyEDA footprints come from library parts (LCSC UUIDs)
that can't be hand-faked, so every part here is a generic labelled block whose pins are
tagged with a NET NAME. Pins sharing a net name are the same wire. Import it to *see and
check* the corrected connectivity (incl. the CN3065 charger + cold-charge limiter); build
the real PCB in your own SCH_*.json by placing LCSC parts and wiring to match (CORRECTIONS.md).

Run:  python3 gen_easyeda.py  ->  writes easyeda_schematic.json
"""
import json

GID = 0
def gid(p="gge"):
    global GID; GID += 1; return f"{p}{GID}"

shapes = []
def rect(x, y, w, h, stroke="#000080"):
    shapes.append(f"R~{x}~{y}~0~0~{w}~{h}~{stroke}~1~0~none~{gid()}~0~~~~")
def text(x, y, s, size=9, color="#000000", anchor="start", bold=False):
    w = "bold" if bold else "normal"
    shapes.append(f"T~comment~{x}~{y}~0~{color}~~{w}~~~{size}pt~{anchor}~~~{s}~pass~{gid()}~0")
def line(x1, y1, x2, y2, color="#888888"):
    shapes.append(f"PL~{x1} {y1} {x2} {y2}~{color}~1~0~none~{gid()}~0")

ROW, PAD = 14, 16
def part(x, y, ref, value, pins_left, pins_right, wmin=170):
    n = max(len(pins_left), len(pins_right)); h = PAD * 2 + n * ROW; w = wmin
    rect(x, y, w, h)
    text(x + w/2, y - 6, ref, size=10, anchor="middle", bold=True)
    text(x + w/2, y + h + 14, value, size=8, anchor="middle", color="#555555")
    for i, (pin, net) in enumerate(pins_left):
        py = y + PAD + i*ROW + 9
        line(x - 18, py - 3, x, py - 3)
        text(x + 4, py, pin, size=7, anchor="start")
        text(x - 22, py, net, size=8, anchor="end", color="#aa0000", bold=True)
    for i, (pin, net) in enumerate(pins_right):
        py = y + PAD + i*ROW + 9
        line(x + w, py - 3, x + w + 18, py - 3)
        text(x + w - 4, py, pin, size=7, anchor="end")
        text(x + w + 22, py, net, size=8, anchor="start", color="#aa0000", bold=True)

# =============================================================================
text(40, 26, "Solar/LoRa(SX1262) Water-Temp Node — corrected net-label reference", size=13, bold=True)
text(40, 44, "Same red NET NAME = same wire. Generic blocks (no footprints): place LCSC parts in your real SCH to build the PCB.", size=8, color="#555555")

# ---- Column 1: power input + charger + cold-charge limiter -------------------
part(80, 90, "BT1", "18650 x2 protected (1S2P)", [], [("+","VBAT"),("-","GND")], 150)
part(80, 210, "J1", "Solar 6V 2-5W", [], [("IN","SOLAR_IN"),("GND","GND")], 150)
part(80, 330, "U1", "CN3065 solar charger (bare)",
     [("VIN","SOLAR_IN"),("GND","GND"),("BAT","VBAT")],
     [("ISET","ISET"),("CHRG","NC"),("DONE","NC")], 180)

text(80, 520, "Cold-charge limiter (VIN-powered, 0 C threshold)", size=9, bold=True, color="#006000")
part(80, 540, "U6", "TLV3691 nano-comparator",
     [("VDD","VCLAMP"),("GND","GND"),("IN+","TSENSE"),("IN-","TREF")],
     [("OUT","COLDn")], 180)
part(80, 700, "NT1", "NTC 100k B3950 @cell", [("a","VIN_R")], [("b","TSENSE")], 150)
part(80, 800, "Q2", "2N7002 NMOS (ISET sw)", [("G","COLDn"),("S","GND")], [("D","ISETB")], 160)
part(330, 540, "Rz", "1k +5V1 zener+100n", [("VIN","SOLAR_IN")], [("VDD","VCLAMP")], 150)
part(330, 640, "Ra", "100k", [("1","TSENSE")], [("2","GND")], 110)
part(330, 720, "Rb", "330k", [("1","SOLAR_IN")], [("2","TREF")], 110)
part(330, 800, "Rc", "100k", [("1","TREF")], [("2","GND")], 110)
part(330, 880, "Rtr", "12k trickle (~100mA)", [("1","ISET")], [("2","GND")], 150)
part(330, 960, "Rbo", "3k boost (~500mA warm)", [("1","ISET")], [("2","ISETB")], 150)

# ---- Column 2: TPL5110 power gate + LDO + battery sense ----------------------
part(620, 90, "U2", "TPL5110 hourly timer",
     [("VDD","VBAT"),("GND","GND"),("DELAY","TDELAY")],
     [("DRV","PG"),("DONE","TPL_DONE")], 180)
part(620, 270, "Rd", "Rdelay ~1h (trimpot)", [("1","TDELAY")], [("2","GND")], 150)
part(620, 360, "Q1", "AO3401 P-FET load sw", [("G","PG"),("S","VBAT")], [("D","VSW")], 160)
part(620, 470, "Rg", "100k gate pull-up", [("1","PG")], [("2","VBAT")], 140)
part(620, 560, "U3", "MCP1700 LDO 3V3",
     [("IN","VSW"),("EN","VSW"),("GND","GND")], [("OUT","VCC")], 160)
part(620, 700, "C1", "10uF VSW", [("1","VSW")], [("2","GND")], 120)
part(620, 780, "C3", "10uF+100nF VCC", [("1","VCC")], [("2","GND")], 130)
part(620, 870, "R8", "220k Vbat sense", [("1","VSW")], [("2","BATT_ADC")], 140)
part(620, 950, "R7", "68k", [("1","BATT_ADC")], [("2","GND")], 120)

# ---- Column 3: ESP8266 + straps ---------------------------------------------
part(940, 120, "U4", "ESP-12E (ESP8266)",
     [("VCC","VCC"),("GND","GND"),("EN","ESP_EN"),("ADC","BATT_ADC"),
      ("GPIO14","SPI_SCK"),("GPIO13","SPI_MOSI"),("GPIO12","SPI_MISO"),
      ("GPIO15","TPL_DONE")],
     [("GPIO16","LORA_NSS"),("GPIO4","LORA_DIO1"),("GPIO5","LORA_BUSY"),
      ("GPIO0","EPD_CS"),("GPIO2","EPD_DC"),("GPIO3/RX","DS_DATA"),
      ("GPIO1/TX","DBG_TX"),("RST","RST")], 180)
part(940, 470, "R6", "10k EN pull-up", [("1","ESP_EN")], [("2","VCC")], 140)
part(940, 545, "R2", "10k RST pull-up", [("1","RST")], [("2","VCC")], 140)
part(940, 620, "R4", "10k GPIO2 (EPD_DC)", [("1","EPD_DC")], [("2","VCC")], 150)
part(940, 695, "R5", "10k GPIO0 (EPD_CS)", [("1","EPD_CS")], [("2","VCC")], 150)
part(940, 770, "R1", "10k GPIO15->GND", [("1","TPL_DONE")], [("2","GND")], 150)
part(940, 845, "SW1", "RESET btn", [("1","RST")], [("2","GND")], 120)

# ---- Column 4: LoRa + antenna + e-paper + DS18B20 ---------------------------
part(1280, 90, "U5", "LoRa1262-868 (SX1262)",
     [("VCC","VCC"),("GND","GND"),("SCK","SPI_SCK"),
      ("MOSI","SPI_MOSI"),("MISO","SPI_MISO"),("NSS","LORA_NSS")],
     [("BUSY","LORA_BUSY"),("DIO1","LORA_DIO1"),("NRESET","RST"),
      ("ANT","LORA_ANT")], 180)
part(1280, 320, "J3", "u.FL / spring antenna", [("ANT","LORA_ANT")], [("GND","GND")], 150)
part(1280, 430, "U7", "WeAct 2.13 e-paper (min)",
     [("VCC","VCC"),("GND","GND"),("SCL","SPI_SCK"),("SDA","SPI_MOSI")],
     [("CS","EPD_CS"),("DC","EPD_DC"),("RES","RST"),("BUSY","NC")], 180)
part(1280, 660, "J2", "DS18B20 probe (3-pin)",
     [("VDD","VCC"),("DATA","DS_DATA"),("GND","GND")], [], 160)
part(1280, 770, "Rds", "4.7k 1-wire pull-up", [("1","DS_DATA")], [("2","VCC")], 150)

# =============================================================================
doc = {
    "head": {"docType": "1", "editorVersion": "6.5.42", "newgId": True,
             "c_para": {"Prefix Start": "1"}, "x": "0", "y": "0",
             "importFlag": 0, "transformList": ""},
    "canvas": "CA~2600~1200~#FFFFFF~yes~#CCCCCC~10~2600~1200~line~1~pixel~5~0~0.5~0~5~5~0~0",
    "shape": shapes,
    "BBox": {"x": 0, "y": 0, "width": 2600, "height": 1200},
    "colors": {}}
with open("easyeda_schematic.json", "w") as f:
    json.dump(doc, f, indent=1)
print(f"wrote easyeda_schematic.json  ({len(shapes)} shapes)")
