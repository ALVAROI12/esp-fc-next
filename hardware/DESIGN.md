# ESP-FC Next flight controller: design inputs (Phase 3, gate G4)

Status: schematic as code (`boards.py`, SKiDL), checked by `make hw`. Nothing ordered. Facts below are from Espressif
documents fetched 2026-10-28; re-check against the latest revisions before layout.

## Decisions
| Topic | Decision | Why |
| --- | --- | --- |
| Form factor | Two-board stack: 28 mm FC (20x20 mm M2 holes) + 28 mm radio board with ESP32-C5-MINI-1 | C5-MINI-1 is 15.4 x 21.3 mm; P4 + power + sensors + C5 do not fit one whoop board. Radio swappable. |
| Battery | 1S only (1S HV, 3.0-4.35 V) | Plan section 4.1. 2S would need a wider-input regulator and 2S ESC. |
| 3.3 V rail | Buck-boost TPS63802 (2 A) | P4 needs 3.0-3.6 V; an LDO drops out at 3.0 V cell voltage. Budget about 0.9 A peak (below). |
| Core rail | TLV62569 buck from 3.3 V, driven by P4 EN_DCDC / FB_DCDC | Espressif-verified part and reference circuit (below). |
| MCU | ESP32-P4NRW16X (rev v3, 16 MB in-package PSRAM) | Rev v1 parts are EOL. ESP-IDF 5.5 defaults to rev >= v3.1. |
| Flash | W25Q128JV 16 MB, 3.3 V, on the dedicated flash pins 27-33 | P4 has no internal flash. |
| Radio link | P4 SPI master <-> C5 SPI slave, 4 wires; P4 also drives C5 EN, C5 boot strap and C5 UART0 | In-system update of the C5 from the P4, no second USB port. |
| Battery current | INA226 reads pack voltage and FC-side current only; motor current comes from the ESC CUR pin | 1S whoop motor current (10-20 A) cannot pass through the FC connector. |

## ESP32-P4 facts used
Sources: ESP32-P4 datasheet v0.7 (rev v3.x); P4 Hardware Design Guidelines, schematic checklist
(https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/schematic-checklist-esp32p4.html);
P4 Function-EV board schematic v1.5.2 (https://dl.espressif.com/dl/schematics/esp32-p4-function-ev-board-schematics_v1.52.pdf).

- Supplies: VDD_LDO, VDD_DCDCC, VDD_ANA, VDD_LP 3.0-3.6 V; VDD_BAT 2.5-3.6 V (never floating); VDD_IO_x and
  VDD_FLASHIO 1.65-3.6 V; VDD_HP_0..3 0.99-1.3 V (>= 0.5 A); VDD_MIPI_DPHY and VDD_USBPHY may float when unused.
  Minimum supply current 380 mA (incl. flash/PSRAM).
- Core buck (EV board v1.5.2, U7): TLV62569 VIN = 3.3 V (4.7 uF), EN/PG = EN_DCDC (pin 79), FB = FB_DCDC (pin 78),
  L = 2.2 uH, output 22 uF. Feedback: 499 k VOUT->FB, 499 k FB->GND, 22 pF across the top resistor. NC on rev v1,
  **must be populated on rev v3** (HDG). EN_DCDC is 0 in download mode or without firmware.
- Decoupling: 10 uF per power entrance; 0.1 uF per VDD_IO/VDD_LP/VDD_HP pin; VDD_BAT 0.1 + 10 uF; 1 uF on VDDO_FLASH,
  VDDO_PSRAM, VDDO_3, VDDO_4; VDD_FLASHIO and VDD_PSRAM_x 0.1 + 1 uF.
- Clock: 40 MHz crystal only (+/-10 ppm); load caps per CL = C1*C2/(C1+C2) + Cstray.
- Reset: CHIP_PU 10 k / 1 uF RC, never floating; tRST >= 1 ms.
- Straps: GPIO34-38. GPIO35 weak pull-up (1 = SPI boot), reserve external pull-up, no large capacitor.
  GPIO35 = 0 and GPIO36 = 1 -> joint download. GPIO34 (JTAG source) has no internal pull. GPIO37/38 = UART0.
- USB-Serial-JTAG on GPIO24 (D-) / GPIO25 (D+), 22-33 ohm series resistors on FS lines.
- SPI2 IO MUX: CS0 7, MOSI 8, SCLK 9, MISO 10 (fastest path, used for the IMU). SPI3 any GPIO.
- RMT: 4 TX channels (exactly 4x DShot). SDMMC slot 0 fixed to GPIO39-44 (D0-D3 39-42, CLK 43, CMD 44).

## ESP32-C5-MINI-1 facts used
Sources: ESP32-C5-MINI-1 datasheet v1.4; ESP32-C5 datasheet v1.5; Espressif certificates page.
- 15.4 x 21.3 x 2.4 mm, PCB antenna, FCC 2.4G/5G + CE certified. Supply 3.0-3.6 V. Peak TX 426 mA (5 GHz 11a).
- Straps: GPIO27/28 pull-up; GPIO28 = 1 SPI boot; GPIO28 = 0, GPIO27 = 1 -> joint download (USB-Serial-JTAG/UART).
- SPI2 slave via GPIO matrix to any pin; up to 40 MHz. IO MUX MISO (GPIO2) and MOSI (GPIO7) are strapping pins:
  avoid them for the link.

## Other parts (pins and application values used in `boards.py`)
- **ICM-42688-P**: TDK DS-000347 **Rev 1.2** (Digi-Key mirror; TDK's site served a bot check). LGA-14 2.5 x 3 mm.
  4-wire SPI: CS 12, SCLK 13, SDI 14, SDO 1, INT1 4. Pin 7 RESV must be GND. VDD 0.1 + 2.2 uF, VDDIO 10 nF.
  Rev 1.8 (07/2023) could not be fetched (TDK pages answer 403/bot check). A second, non-TDK document (LCSC-hosted)
  names pins 2/3 aux I2C (to sensors or VDDIO, not GND) and pin 10 an aux SPI enable, where Rev 1.2 says "NC or GND":
  pins 2/3/10/11 are left unconnected, which both allow. Pin 9 (FSYNC input / INT2 output) goes to P4 GPIO18 with a
  10 k pull-down instead of GND. KiCad's `LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y` matches Fig. 4 and section 10.2
  (pin order, lead centres +-1.1625 / +-0.9125); a test checks it.
- **BMP581**: Bosch BST-BMP581-DS004-13 rev 1.13. LGA-10 2 x 2 mm. I2C: CSB to VDDIO, SDO low = 0x46,
  INT left unconnected (Bosch allows DNC), SDO to GND through a 0 ohm strap (move to 3V3 for 0x47),
  100 nF on VDD and VDDIO. Supplies ramping faster than 10 us may need 10 ohm in series (Fig. 28).
- **TPS63802**: TI SLVSEU9D. DLA0010A VSON-HR 3 x 2 mm. Adjustable only, VFB 0.5 V: 511 k / 91 k = 3.31 V
  (R2 <= 100 k). L 0.47 uH (XFL4015-471ME), CIN 10 uF, COUT 22 uF+. 2 A guaranteed for VIN >= 2.3 V;
  UVLO 1.25 V falling. MODE low = power save. Peak switch limit 4 A min.
- **TLV62569DBV**, **W25Q128JVS**, **INA226**, USB-C and connectors come from the KiCad 10 library.

## P4 pin map (`boards.P4`)
| Function | GPIO | Note |
| --- | --- | --- |
| IMU SPI2 CS/MOSI/SCLK/MISO, INT1 | 7/8/9/10, 11 | IO MUX pins |
| C5 link SPI3 MOSI/MISO/SCLK/CS | 2/3/4/5 | matches `ESPFC_C5_SPI_*` Kconfig defaults; JTAG pins, unused (USB-JTAG) |
| Flow module CS (PMW3901 shares SPI3) | 6 | VL53L1X on I2C |
| C5 EN, C5 BOOT (C5 GPIO28), UART to C5 TX/RX | 12, 13, 14/15 | reflash the C5 from the P4 |
| ESC current, ESC telemetry | 16 (ADC1_CH0), 17 | |
| IMU INT2/FSYNC | 18 | 10 k pull-down |
| I2C SDA/SCL (BMP581 0x46, INA226 0x40, flow ToF) | 28/29 | 4.7 k pull-ups |
| DShot M1-M4 | 45-48 | 4 RMT TX channels |
| CRSF receiver TX/RX | 50/51 | |
| USB D-/D+ (USB-Serial-JTAG) | 24/25 | 22 ohm series |
| Straps 34/35/36, UART0 37/38 | | 34 pulled low, 35/36 pulled up, BOOT button on 35; UART0 on J5 |

C5 side (`boards.C5`): SPI MOSI 8, MISO 9, SCLK 6, CS 10 (`ESPFC_RADIO_SPI_*` defaults, moved off strapping pins
2/3), UART0 11/12, EN with 10 k / 1 uF, GPIO28 10 k pull-up. Stack connector order: `boards.STACK`.
microSD (SDMMC slot 0) left out of the first board: not needed for flight; GPIO39-44 stay free for it.

## Power path and 3.3 V budget
Battery -> 10 mOhm INA226 shunt -> P-FET (off when USB VBUS is present) -> VSYS; USB VBUS -> Schottky -> VSYS.
P-FET: **Diodes DMP1022UFDF-7** (LCSC C155358), U-DFN2020-6 Type F; DS36624 rev 6-2: Rds(on) max 19 mOhm at
VGS -2.5 V and 26 mOhm at -1.8 V (ID -4 A), VGS(th) -0.35 to -0.8 V, -12 V / +-8 V. No SOT-23 part checked met
<= 30 mOhm max at -2.5 V (Si2333DDS 40, SSM3J328R 39.7, DMP1045U 45, AO3415A 53). Alternative: Vishay SiA447DJ
(19.4 mOhm max at -2.5 V, but 34.4 at -1.8 V). The simulation budgets 30 mOhm for the FET (hot-Rds margin).
VSYS (10 uF + 100 uF) -> TPS63802 -> 3V3 -> TLV62569 -> VDD_HP.

Budget (peak, simultaneous): P4 380 mA (HDG minimum incl. flash/PSRAM) + C5 426 mA + IMU/baro/flash/INA226 ~20 mA +
flow module (PMW3901 + VL53L1X) ~40 mA = **~0.87 A**. Regulator rating 2 A.

`power_sim.py` (ngspice; pack OCV + internal resistance, 16 A motor step, 1 A on 3.3 V at 85 % efficiency):
| Case | VBAT min | VSYS min |
| --- | --- | --- |
| 4.2 V, 50 mOhm pack | 3.17 V | 3.12 V |
| 3.5 V (end of flight), 50 mOhm pack | 2.44 V | **2.38 V** (>= 2.3 V full-load limit) |
| 3.5 V, 100 mOhm pack | 1.41 V | 1.29 V (brownout) |
| First draft: 20 mOhm shunt + AO3401A (85 mOhm) | 2.44 V | 2.26 V (fails) |

The FC battery path must stay <= 50 mOhm in total: max pack resistance at 3.5 V and 16 A is then 54 mOhm. A worn or
small pack sags below this regardless of the FC: the 3.3 V/cell low-battery failsafe has to land before it gets there.

## Verification (`make hw`)
`boards.py` writes `out/fc_board.net` and `out/radio_board.net` (stable tags, so KiCad keeps placement across
regenerations) and draws `out/<board>/<board>.kicad_sch`. The sheets are auto-placed with net labels and no wires:
schematizer's routed mode shorted GND into eight other nets on the flight board, which the netlist comparison below
caught. PWR_FLAGs are added only to the sheets. `test_boards.py`: SKiDL ERC clean on both boards; P4 rails and per-pin decoupling; core-buck
feedback populated (499 k / 499 k / 22 pF, 1.2 V) and wired to FB_DCDC/EN_DCDC; 3.3 V divider; straps, CHIP_PU RC,
FLASH_CS pull-up; IMU on SPI2 IO MUX; firmware Kconfig pin defaults equal the board nets on both chips; stack
connector identical on both boards with the UART crossed; C5 link off strapping pins; power path vs ngspice; every
symbol pin has a pad in an existing footprint; ICM footprint orientation; ESPFC library regenerated; each drawn
sheet loads in KiCad, passes KiCad's ERC and exports a netlist identical in connectivity to SKiDL's. Mutations
(FB resistor, GPIO36 pull-up, shunt value, C5 pin, FB_DCDC wire, wrong footprint, missing footprint, routed sheets,
unquoted symbol text, a missing PWR_FLAG) each fail a test.

## PCB placement and routing (`make hw-pcb`)
`pcb.py` (run by KiCad's Python) turns each netlist into `out/<board>.kicad_pcb`: 28 mm square, 4 layers, M2 holes
on a 20 mm square, every footprint with its nets, greedy courtyard placement on a 0.1 mm grid. Fixed: the stack
header on the left edge between the holes (FC: header on top; radio: socket on the bottom, same pad XY and nets),
USB-C on the bottom edge. Order: P4 with its pin rows facing the crystal, crystal and load caps right beside pins
99/100 (a few mm of F.Cu; greedy placement put the crystal 13 mm away), connectors (edge strips), IMU (top only), big
parts (over 8 mm2), 100 nF decoupling caps aimed at the rail pin farthest from the caps already there, then by size.
The buck-boost inductor gets its TPS63802 right beside it. No connector under the P4 (it blocked the fanout of a motor
pin), and no small part on the bottom side over the P4's two pin rows, where the fanout vias go (`frame`; a test
checks it; 13 parts sat there while the frame was computed round the inductor by mistake, and routing left 5-8 nets). A 0.25 mm fanout ring round the P4 courtyard keeps other parts
(not the edge or the hole keep-outs) away: with the 0201 caps at the normal 0.15 mm, two P4 pins (`M2`, `FLASH_CK`)
had no room for their escape vias; rings of 0.3-0.35 mm moved the choke point to other pins and 0.4 mm and up no
longer fit every part. pF/nF caps are 0201: with 0402 the 25.5 mm flight board did not fit (16 caps left over at
~85 % courtyard fill). The boards are 28 mm, not 25.5: at 25.5 mm the same FC placement routed clean in one run and
left one or two P4 fanout nets in three others (a 0.35 mm pitch LGA with 0.3/0.15 mm vias and 0.25 mm hole-to-hole);
27 mm still left one net every run, 6 layers at 25.5 mm routed but left a GND stray, 28 mm routed at the first
attempt with room to spare (top 508 / 629 mm2 at 27 mm). 28 mm is a common whoop FC size on the same 20 x 20 mm
mounts. GND: one solid plane on In1 (the reference for every signal, 0.1 mm clearance: at 0.2 the voids
round the via field under the P4 merged and split the plane); In2 for signals. Netclasses: `Power` (battery,
converter switch nodes and rails, listed in `pcb.py`) 0.3 mm, everything else 0.1 mm.

`route.py` autoroutes with Freerouting 2.5.0 (Java 25+, `brew install openjdk`; jar from
github.com/freerouting/freerouting/releases/tag/v2.5.0 into `hardware/tools/`, git-ignored, sha256
f6f51bb02245e8e717f9359bd260cc9c5c0b1bc0acc8b7cb2cd5b8ffeb5de3c7) to `out/<board>-routed.kicad_pcb`:
- The In1 plane goes to the router with 0.2 mm clearance (at 0.1 it fanned GND pads out into voids it could not
  then use) and is refilled at 0.1 mm afterwards. In1 stays a signal layer for the router: declared a power layer
  it left 73 FC nets unrouted, so signal tracks do cross the plane.
- Freerouting routes at 0.1 mm spacing against the 0.09 mm rule: it models the P4's oval pad ends as octagons, up to
  7.6 um short, and KiCad's exact check then fails at 0.1/0.1.
- Freerouting takes the DSN plane for solid copper, but the via field and the In1 tracks cut KiCad's fill into
  about ten pieces, so GND stubs it counted as "on the plane" sit on nothing. GND pours on F.Cu and B.Cu are added
  after routing (in the DSN they made the router skip GND pads sitting on them) and join every stub to GND copper
  nearby; islands that reach nothing are removed. What the pours cannot fix is a GND via landing on a cut-off piece
  of the In1 plane (an 8 x 12 mm piece on the FC board, holding one BMP581 GND pad): `stitch()` walks the GND copper
  as clusters (pads, tracks and vias by KiCad's connectivity, each pour island as a node) and puts one GND via into
  every stray island where it overlaps a connected island on another layer, clear of all other copper.
- Freerouting's geometry is octagonal, so against KiCad's exact check a track can sit a few um too close to a hole
  (up to 5 um short on 0.1 mm tracks, 11 um on 0.3 mm ones). `finish()` runs KiCad DRC and narrows the offending
  track by the shortfall (floor 0.09 mm; the router's 0.075 mm fanout necks are first widened to 0.09 and may go back
  to 0.076, JLCPCB's "3 mil in BGA fan-outs", inside the P4 courtyard). Through-hole padstacks grow by 0.16 mm in the
  DSN only, so the router keeps JLCPCB's 0.28 mm from plated holes to copper (the 1.0 mm stack header pads have a
  0.175 mm ring). An attempt counts as done only when the finished board passes the copper DRC with nothing
  unconnected; otherwise the next element order is tried.
- Freerouting is deterministic for a given DSN (six runs, six identical results), but KiCad's DSN exporter orders
  components, images, padstacks and nets by their random UUIDs, so the same board routed clean in one run and left
  a net in the next. `route.py` sorts the DSN by name before routing (`canonical()`); other runs use a different
  repeatable order (nets reversed, components reversed, both, then seeded shuffles).
- The critical nets (`XTAL_P/N`, `BB_L1/L2`, `HP_SW`) are routed alone first and locked, so the autorouter works round
  them; KiCad exports them as `fix` wires. Then `ROUTE_RUNS` orders (default 10) run in parallel, `ROUTE_JOBS` at a
  time, each stopped at `ROUTE_LIMIT` seconds: a pass is single-threaded, so this scales with the core count. Lowest
  clean order wins. Freerouting's unrouted count is not the gate: up to 2 leftovers still go to KiCad's check, because
  it can count a GND connection open that the plane and pours close (the pinned FC route: 1 "unrouted", 0 in KiCad).
- `finish()` deletes a GND stub that is still too close at the floor width (the pours carry GND); DRC reports an open
  if it was needed.
- **The routed copper is a source file: `routes/<board>.json`**, every track and via before the pours. Freerouting is
  deterministic for a DSN on one machine but not across them (the same order gave 1 unrouted on Linux/Java 25 and 5 on
  macOS/Java 27), so `route.py` applies the pinned copper to the fresh placement and runs the same finish and DRC gate.
  A netlist or placement change fails the gate and routes afresh, replacing the file only when the new route is clean;
  delete the file to force a re-route. The FC route took 30 orders on a 32 vCPU RunPod pod:
  `ROUTE_REMOTE="root@<ip> <port>" ROUTE_JOBS=30 ROUTE_RUNS=30` runs Freerouting there (Java 25+ on PATH, jar in the
  home directory, SSH key `~/.runpod/ssh/runpodctl-ssh-key`). KiCad itself still runs locally.

Rules are JLCPCB's 4-layer, 1 oz limits (jlcpcb.com/capabilities/pcb-capabilities, read 2026-10-31), written by
`pcb.py` to `out/<board>.kicad_pro` and `.kicad_dru`: 0.09 mm track and space (routed at 0.1), vias 0.15 mm hole with
a 0.35 mm pad (JLCPCB wants 0.2 mm from a via hole to copper, met exactly at 0.1 mm clearance; 0.37 and 0.38 mm pads,
a 0.115 mm Power-class clearance and typed DSN clearances such as `wire_via` all left P4 fanout nets unrouted, so the
few-um shortfalls are repaired after routing instead), via hole-to-hole 0.2, copper-to-edge 0.2, solder mask dam 0.1
with no mask expansion (JLCPCB opens 1:1), and conditional rules for through-hole pads (0.28 hole-to-copper, 0.15
ring, 0.45 hole-to-hole), a 0.15 mm gap between SMD pads, and 0.19 mm for a footprint's own NPTH pegs (the KiCad USB-C
footprint puts them 0.194 mm from its GND pads). Not expressible in KiCad: JLCPCB's 0.25 mm same-net track spacing.
The test's copper set is everything JLCPCB's check would reject: clearance, shorts, hole clearance, hole-to-hole, edge
clearance, annular ring, track width, via size, drill size, mask bridges, starved thermals, isolated copper, plus zero
unconnected items.

Result: both boards routed and clean against those rules (FC 1565 tracks, 182 vias; radio 123 tracks, 32 vias).
Critical lengths on the FC board: XTAL_P 4.3 mm, XTAL_N 6.1 mm, BB_L1 7.3 mm, BB_L2 5.2 mm, HP_SW 13.5 mm.
Remaining DRC, not fab-related: the FC stack header courtyard overlaps two M2 hole courtyards (header body clears
an M2 head by ~0.5 mm), the FC crystal courtyard overlaps one, one dangling FC via, and the radio board's C5 module
courtyard overlaps two hole courtyards. The C5 antenna now hangs over the top edge, so no hole sits in its keep-out.
Tests: both boards fit, the stack pads line up with the same nets (a wrong bottom-side rotation fails it), and each
routed board passes the copper DRC with zero unconnected items (skipped without Java or the jar; about 2 min for the
whole file with both routes pinned).
`make hw-pcb` also renders `out/render/*.png` and plots the power-path transient to `out/power_transient.png`.
`make hw-gerbers` writes `out/gerbers/<board>.zip` for the JLCPCB order form: Gerber X1 with Protel extensions
(copper, paste, silk, mask, Edge.Cuts), Excellon PTH and NPTH drills plus a drill map, all on the absolute origin.
JLCPCB's own DFM check on upload is the final word on the footprints.
The same target runs `assembly.py`, which writes `out/assembly/<board>-bom.csv` (Comment, Designator, Footprint,
LCSC Part #) and `<board>-cpl.csv` (Designator, Mid X, Mid Y, Layer, Rotation) in the column layout JLCPCB's
assembly form expects; mounting holes and test points are excluded. The LCSC numbers live in a table in
`assembly.py` (looked up 2026-11-01): ICs, sensors, connectors, switch, crystal, diode and the 2.2 uH inductor have
one; passives are blank so JLCPCB matches them by value and footprint. Without a listing: the ESP32-C5-MINI-1
module (consign it or buy from Espressif's distributors), the 0.47 uH XFL4020 inductor (series not stocked;
Coilcraft's drop-in is the XGL4020-471MEC) and the 1.27 mm stack header (hand-solder). The ESP32-P4NRW16X maps to
JLCPCB's "ESP32-P4NRW16" listing, which needs confirming on upload. Rotations are KiCad's; check polarised parts
(D1, C2, the ICs' pin 1) in JLCPCB's placement preview before paying.

## Libraries
`lib/espressif/`: Espressif official KiCad symbols (ESP32-P4X, ESP32-C5-MINI-1) and footprints, CC-BY-SA 4.0 with
the KiCad design exception (`LICENSE.md`, upstream commit in `UPSTREAM`). Other parts come from KiCad's own library;
parts missing there (ICM-42688-P, BMP581, TPS63802, DMP1022UFDF) are KiCad symbols in `lib/ESPFC.kicad_sym`, and
`library.py` writes them and `lib/ESPFC.pretty` (BMP581: Bosch Fig. 30 + section 8.2, pads +25 um, mask opening over
the body; TPS63802: TI DLA0010A example board layout); `fp-lib-table` and `sym-lib-table` map
the local libraries for a KiCad project in `hardware/`. Tests check pad count vs symbol, >= 0.2 mm pad gaps and pin 1
position. Local change to the vendored Espressif symbols: P4 VDD_ANA and VDD_BAT retyped from bidirectional to power
input (recorded in `UPSTREAM`). Inductors: Coilcraft XFL4020-471ME (TI lists XFL4015-471ME; KiCad has the 4020
land pattern) and Murata DFE201610E-2R2M.

## Open items
- `ESPFC` footprints follow the datasheet drawings; they pass KiCad DRC on the routed boards with JLCPCB's rules,
  not yet checked against a physical part or JLCPCB's own DFM check.
- ICM-42688-P against TDK Rev 1.8 once the document can be downloaded.
- GPIO34 level: only matters once the JTAG-select eFuse is burned; confirm with the P4 TRM.
- VDD_FLASH_IO fed from 3V3 directly (allowed 1.65-3.6 V); the EV board's choice not confirmed.
- Reset supervisor vs RC only (HDG suggests a ~3.0 V reset IC for battery supplies).
- Crystal load caps 12 pF assume CL 10 pF and ~4 pF stray; set from the chosen crystal.
- Stack connector part and pitch; flow/ToF module connector; ESC connector pinout (varies by ESC vendor).
- Sheets are machine-placed and dense (one A3 page per board); connectivity is verified, readability is not.
- Stack height: the FC top carries a 4.25 mm tall JST-SH vertical and the USB-C; the stack connector must clear them.
- Radio board: M2 screws next to the antenna; nylon screws there, or a different hole pattern on that board.
- Autorouted, not reviewed by a human: return paths, the 40 MHz crystal and USB D+/D- (not length-matched or
  impedance-controlled), current capacity of the 3V3/VSYS/VBAT tracks (all 0.1 mm now; the battery path needs real
  copper width or pours), via-in-pad, assembly (0201 with solid GND connections).
- 3D models for the P4 and C5 (pads only in the renders).
- Not simulated: converter loop stability, C5 TX burst transients on 3V3, layout parasitics.
