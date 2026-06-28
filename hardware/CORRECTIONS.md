# Schematic corrections — water-temp (ESP8266 + LoRa1262 + e-paper + DS18B20)

Target from your choices: **LoRa + DS18B20 + minimal e-paper**, plus **TPL5110 hourly
hard power-cut** and a **proper solar charger**. This is the corrected, conflict-free
wiring to apply to `SCH_water-temp_2026-06-27.json`.

## The one trick that makes it fit
The ESP8266 has only 9 usable GPIO (GPIO6–11 are the internal flash bus — never wire to
them). 10 signals are needed. The fix: **tie both LoRa `NRESET` and e-paper `RES` to the
board `RST` net** (they reset together with the MCU and at every TPL5110 power-up). That
frees a pin *and* lets `DIO1` sit on an interrupt-capable GPIO. In firmware pass
`reset = RADIOLIB_NC` for the SX1262 and `rst = -1` for the e-paper.

## Corrected ESP8266 pin map

| ESP-12 pin | GPIO | Net | Goes to | Boot strap (keep) |
|---|---|---|---|---|
| 14 SCLK→ **use IO14** | **GPIO14** | `SPI_SCK` | LoRa SCK + epaper SCL | — |
| 13 MOSI→ **use IO13** | **GPIO13** | `SPI_MOSI` | LoRa MOSI + epaper SDA | — |
| 10 MISO→ **use IO12** | **GPIO12** | `SPI_MISO` | LoRa MISO only | — (was No-Connect) |
| 4 | GPIO16 | `LORA_NSS` | LoRa NSS (CS) | — |
| 19 | GPIO4 | `LORA_DIO1` | LoRa DIO1 (IRQ) | (R3 pull-up ok/optional) |
| 20 | GPIO5 | `LORA_BUSY` | LoRa BUSY | — |
| 18 | GPIO0 | `EPD_CS` | epaper CS | R5 10k→VCC (high@boot) |
| 17 | GPIO2 | `EPD_DC` | epaper D/C | R4 10k→VCC (high@boot) |
| 21 RXD | GPIO3 | `DS_DATA` | DS18B20 data | 4.7k→VCC (high@boot) |
| 16 | GPIO15 | `TPL_DONE` | TPL5110 DONE | **R1 10k→GND (low@boot) ✓** |
| 22 TXD | GPIO1 | `DBG_TX` | optional serial debug out | — |
| 1 | RST | `RST` | RST btn, R2 pull-up, **LoRa NRESET, epaper RES** | R2 10k→VCC |
| 2 | ADC | `BATT_ADC` | R7/R8 divider midpoint | — |

> Why these land where they do: `GPIO15` must be low at boot → it's `TPL_DONE` (which
> must be low until firmware finishes — your R1 pulldown is exactly right). `GPIO0/2/3`
> must be high at boot → e-paper CS/DC idle high and the 1-wire pull-up holds DS data
> high. `GPIO16` is output-only-friendly → LoRa NSS. `GPIO4` keeps interrupts → DIO1.

## What to change on the existing sheet

**Delete these wrong nets / connections:**
- The nets `MISO`, `MOSI`, `SCLK`, `CS0` as currently drawn — they land on ESP pins
  9/10/13/14 = **flash GPIO6/7/8/11, unusable**. Remove those wires.
- The No-Connect (green X) on ESP **IO12, IO4, IO0** — those pins are now used.

**LoRa1262 (U2) — rewire to:**
| U2 pin | Net |
|---|---|
| 2 MISO | `SPI_MISO` (GPIO12) |
| 3 MOSI | `SPI_MOSI` (GPIO13) |
| 4 SCK | `SPI_SCK` (GPIO14) |
| 5 NSS | `LORA_NSS` (GPIO16) |
| 6 NRESET | `RST` net |
| 15 DIO1 | `LORA_DIO1` (GPIO4) |
| 16 BUSY | `LORA_BUSY` (GPIO5) |
| 13 VCC | `VCC` (3V3) · 1,8,10 GND · 9 ANT · 7,11,12,14 NC |

**E-paper (SCREEN) — rewire to (minimal 2-wire):**
| SCREEN pin | Net |
|---|---|
| 4 CS | `EPD_CS` (GPIO0) |
| 3 D/C | `EPD_DC` (GPIO2) |
| 5 SCL | `SPI_SCK` (GPIO14) |
| 6 SDA | `SPI_MOSI` (GPIO13) |
| 2 RES | `RST` net |
| 1 BUSY | **No-Connect** (firmware uses a fixed worst-case delay) |
| 8 VCC → `VCC` · 7 GND → `GND` | |

**DS18B20 — ADD (it's missing entirely):**
- 3-pin connector: VDD→`VCC`, DATA→`DS_DATA` (GPIO3), GND→`GND`.
- One **4.7k** pull-up from `DS_DATA` to `VCC`.

## Power section corrections (TPL5110 + solar)

Insert the TPL5110 + P-FET so the **whole 3V3 rail is switched** (off between samples):

```
 Solar ─► CN3065 (bare IC) ─► VBAT(18650+) ─┬─► TPL5110.VDD
                                           ├─► Q1.S (P-FET)
                                           └─► R8 220k (divider, see note)
 TPL5110.DRV ─► Q1.G   (R 100k gate pull-up to VBAT)
 TPL5110.DELAY ─► Rdelay ─► GND   (set ~1 h; trimpot recommended)
 TPL5110.DONE ◄─ ESP GPIO15
 Q1.D ─► VSW ─► MCP1700.IN ;  MCP1700.OUT ─► VCC (3V3)   <- now gated
```

- **Replace the TP4056 module (M1)** with a **bare CN3065** (no LEDs, no protection-IC
  quiescent — that LED was a 24–120 mAh/day drain). See the charger sub-circuit below.
  Its `BAT` pin = `VBAT`, battery+ = `VBAT`; the system load taps `VBAT` through Q1.
- **Move MCP1700 input** from the always-on charger output to **`VSW`** (Q1 drain). This
  is what gives you the ~35 nA standby — the LDO, ESP, LoRa, e-paper and DS18B20 all lose
  power between hourly wakes.
- **Move the R8/R7 battery divider top to `VSW`** (not raw VBAT) so it doesn't leak
  ~14 µA continuously. VSW = VBAT while powered, so the reading is unchanged.
- TPL5110 runs from **raw VBAT** (must stay alive to time the hour).

## Bare-IC solar charger — CN3065 (replaces the TP4056 module)
Linear single-cell Li-ion solar charger: internal pass transistor **and internal reverse
blocking** (no Schottky, no night-drain into the panel), auto input-current adaptation.
8-DFN, [LCSC C45284](https://www.lcsc.com/product-detail/C45284.html). Connect **by pin
name** and cross-check numbers against the datasheet *Typical Application* — the LCSC
symbol is pre-pinned. ([CN3065 datasheet](https://raw.githubusercontent.com/SeeedDocument/Lipo_Rider_Pro/master/res/DSE-CN3065.pdf))

| Pin | Connect to |
|---|---|
| VIN | Solar + · 10 µF X5R → GND at the pin |
| GND / VSS / thermal pad | Solar − , Battery − , system GND (star); flood pad with copper |
| BAT | `VBAT` (battery +) · 10 µF → GND (the 18650 is the bulk) |
| ISET | charge-current set; now driven by the **cold-charge limiter** below (full when warm, trickle when sub-zero). `I_chg[mA] ≈ 1.2e6 / R_ISET[Ω]` *(approx, verify)* |
| CHRG / DONE | **leave open — no LEDs** (this is the efficiency win) |
| FB | per datasheet single-cell 4.2 V config |

- Dark-period battery drain ≈ a few µA (vs the module's mA-level LED).
- **No cell protection on a bare IC** → use **protected 18650 cells** (or add DW01A+FS8205).
  Over-discharge is already bounded by the MCP1700 dropout (~3.4 V).
- 8-DFN needs hot air / JLC assembly; hand-solder alternative = **CN3083 (MSOP-10)**.
- Max-harvest alternative (more parts): **CN3791** MPPT — P-FET + inductor + Schottky +
  sense-R + COM net; only worth it if weak-light harvest is the binding constraint.

## Cold-charge current limiter (full charge ≥0 °C, trickle only sub-zero)
Switches the CN3065 `ISET` resistor at a 0 °C threshold. **Powered from VIN (panel)** →
zero battery drain, only active while charging; **ratiometric** → panel-voltage independent.

| Part | Connection |
|---|---|
| U6 TLV3691 (nano comparator, SC70-5) | VDD ← VIN via 1k, with 5.1 V zener + 100 nF to GND (clamps panel Voc) |
| NTC 100k @25 °C B≈3950 (bonded to the **cell**) | VIN → NTC → `TSENSE` (= comp IN+) |
| R_a 100k | `TSENSE` → GND |
| R_b 330k / R_c 100k | VIN → `TREF` → GND (comp IN−; sets 0 °C trip) |
| R_hyst ~3.3M | OUT → IN+ (a couple °C hysteresis) |
| Q2 2N7002 (NMOS) | G ← comp OUT, S → GND, D → R_boost |
| R_trickle 12k | `ISET` → GND (≈100 mA, always present) |
| R_boost 3k | `ISET` → Q2.D (adds → ≈500 mA when warm) |

- T ≥ 0 °C → OUT high → Q2 on → `12k∥3k ≈ 2.4k` → **~500 mA full charge**.
- T < 0 °C → OUT low → Q2 off → `12k` → **~100 mA trickle** (or leave R_trickle off to fully pause).
- Verify the 0 °C point against your NTC's R–T table; tune R_b/R_c. NTC senses the
  **battery**, not the water probe.

## Strap resistors — keep, with new meanings
R1(GPIO15↓)=now the TPL_DONE pulldown ✓ · R2(RST↑) ✓ · R4(GPIO2↑)=EPD_DC ✓ ·
R5(GPIO0↑)=EPD_CS ✓ · R6(EN↑) ✓ · R3(GPIO4↑)=on DIO1, harmless (may remove).

## Parts to add / change (Norway-sourceable)

| Ref | Part | Source | Note |
|---|---|---|---|
| U (timer) | **TPL5110DDCR** SOT-23-6 | LCSC **C2651358**, Digi-Key, Mouser, Elfa Distrelec NO | hourly cut-off |
| Q1 | **AO3401A** / DMG3415U P-MOSFET SOT-23 | LCSC C15127 | load switch |
| Rg / Rdelay | 100k + ~170k (or 200k trimpot) | any | gate pull-up / 1 h |
| — | **DS18B20** waterproof probe + 3-pin conn + 4.7k | electrokit.se, Digi-Key, AliExpress | the temp sensor |
| Charger | **CN3065** bare IC, 8-DFN | LCSC **C45284** | replaces TP4056 module; see sub-circuit above |
| R_ISET | ~10–12k (sets ~100–120 mA) | any | gentle trickle for cold cell + small panel |
| C_IN, C_BAT | 2× 10 µF X5R | any | charger in/out |
| U6 | **TLV3691** nano comparator SC70-5 | LCSC / Digi-Key / Mouser | cold-limiter |
| Q2 | 2N7002 NMOS SOT-23 | LCSC C8545 | ISET switch |
| NTC | 100k @25 °C B3950 | any | bonded to the cell |
| limiter R/C | 1k, 100k, 330k, 100k, 3.3M, 12k, 3k, 5.1V zener, 100nF | any | see sub-circuit |
| BT1 | **protected** 18650 ×2 (1S2P, ~6000 mAh) | — | parallel; protection replaces module's |

(Everything else you already have — ESP-12E, LoRa1262, MCP1700, u.FL, e-paper, R/C — stays.)
