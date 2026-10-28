"""ESP-FC Next hardware as code (SKiDL): the 25.5 mm P4 flight controller and the stacked C5 radio board.

`python boards.py` writes KiCad netlists to out/. Sources and decisions: DESIGN.md. Checks: test_boards.py.
"""
import builtins
import os
from pathlib import Path

from skidl import KICAD9, POWER, SKIDL, TEMPLATE, Circuit, Net, Part, Pin, lib_search_paths, set_default_tool

HERE = Path(__file__).resolve().parent
set_default_tool(KICAD9)
lib_search_paths[KICAD9] += [os.environ.get("KICAD9_SYMBOL_DIR", "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"),
                             str(HERE / "lib/espressif")]
T = Pin.types

# P4 <-> C5 board-to-board signals, connector pin 1..12 (both boards use this order).
STACK = ["3V3", "3V3", "GND", "GND", "C5_MOSI", "C5_MISO", "C5_SCLK", "C5_CS", "C5_EN", "C5_BOOT", "P4_TX", "P4_RX"]

# P4 GPIO assignments (firmware/flight/main/Kconfig.projbuild for the C5 link).
P4 = dict(IMU_CS=7, IMU_MOSI=8, IMU_SCLK=9, IMU_MISO=10, IMU_INT=11,  # SPI2 IO MUX pins
          C5_MOSI=2, C5_MISO=3, C5_SCLK=4, C5_CS=5, FLOW_CS=6,  # SPI3 (GPIO2-5 are JTAG; we debug over USB)
          C5_EN=12, C5_BOOT=13, P4_TX=14, P4_RX=15,
          ESC_CUR=16, ESC_TLM=17, SDA=28, SCL=29,
          M1=45, M2=46, M3=47, M4=48, CRSF_TX=50, CRSF_RX=51)
# C5 side (firmware/radio/main/Kconfig.projbuild); avoids the C5 strapping pins 2, 3, 7, 25-28.
C5 = dict(C5_MOSI=8, C5_MISO=9, C5_SCLK=6, C5_CS=10, P4_TX=12, P4_RX=11)  # UART0: RX=12 gets P4_TX

R0402, C0402, C0603 = "Resistor_SMD:R_0402_1005Metric", "Capacitor_SMD:C_0402_1005Metric", "Capacitor_SMD:C_0603_1608Metric"


def custom(name, footprint, pins, ref="U"):
    """Part missing from the KiCad library: pins are (number, name, type) from the datasheet in DESIGN.md."""
    return Part(name=name, tool=SKIDL, dest=TEMPLATE, ref_prefix=ref, footprint=footprint,
                pins=[Pin(num=n, name=nm, func=f) for n, nm, f in pins])


def ICM42688P():
    return custom("ICM-42688-P", "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y", [
        (1, "SDO", T.OUTPUT), (2, "RESV2", T.PASSIVE), (3, "RESV3", T.PASSIVE), (4, "INT1", T.OUTPUT),
        (5, "VDDIO", T.PWRIN), (6, "GND", T.PWRIN), (7, "RESV7", T.PASSIVE), (8, "VDD", T.PWRIN),
        (9, "FSYNC", T.INPUT), (10, "RESV10", T.PASSIVE), (11, "RESV11", T.PASSIVE), (12, "CS", T.INPUT),
        (13, "SCLK", T.INPUT), (14, "SDI", T.INPUT)])()


# ponytail: KiCad has no footprint for these two; draw ESPFC.pretty from the datasheets at layout.
def BMP581():
    return custom("BMP581", "ESPFC:Bosch_LGA-10_2x2mm_P0.5mm", [
        (1, "VDDIO", T.PWRIN), (2, "SCK", T.INPUT), (3, "VSS3", T.PWRIN), (4, "SDI", T.BIDIR), (5, "SDO", T.BIDIR),
        (6, "CSB", T.INPUT), (7, "INT", T.OUTPUT), (8, "VSS8", T.PWRIN), (9, "VSS9", T.PWRIN), (10, "VDD", T.PWRIN)])()


def TPS63802():
    return custom("TPS63802", "ESPFC:Texas_DLA0010A_VSON-HR-10_3x2mm_P0.5mm", [
        (1, "EN", T.INPUT), (2, "MODE", T.INPUT), (3, "AGND", T.PWRIN), (4, "FB", T.INPUT), (5, "PG", T.OPENCOLL),
        (6, "VOUT", T.PWROUT), (7, "L2", T.PASSIVE), (8, "GND", T.PWRIN), (9, "L1", T.PASSIVE), (10, "VIN", T.PWRIN)])()


def r(a, b, value, fp=R0402):
    p = Part("Device", "R", value=value, footprint=fp)
    a += p[1]
    b += p[2]
    return p


def c(a, b, value, fp=C0402):
    p = Part("Device", "C", value=value, footprint=fp)
    a += p[1]
    b += p[2]
    return p


def gpio(part, n):
    """Pin of GPIOn, whatever alternate names the symbol adds ("GPIO16/ADC1_CHANNEL0", "U0TXD/GPIO11")."""
    (pin,) = [p for p in part.pins if f"GPIO{n}" in p.name.split("/")]
    return pin


def net(name, *pins):
    n = Net(name)
    for p in pins:
        n += p
    return n


def power(*names):
    nets = [Net(n) for n in names]
    for n in nets:
        n.drive = POWER
    return nets


def stack_connector(nets):
    j = Part("Connector_Generic", "Conn_01x12", ref="J_STACK",
             footprint="Connector_PinHeader_1.27mm:PinHeader_1x12_P1.27mm_Vertical")
    for i, name in enumerate(STACK, 1):
        nets[name] += j[i]
    return j


def fc_board():
    vbat, vsys, vbus, v33, vhp, gnd = power("VBAT", "VSYS", "VBUS", "3V3", "VDD_HP", "GND")
    sig = {k: Net(k) for k in P4}
    p4 = Part("Espressif", "ESP32-P4X", ref="U1", value="ESP32-P4NRW16X")
    for pin in p4["VDD_ANA", "VDD_BAT"]:
        pin.func = T.PWRIN  # typed bidirectional in the Espressif symbol

    # Power path: battery through the INA226 shunt and a P-FET that turns off when USB is present
    # (USB-or-battery load sharing), USB through a Schottky.
    vshunt = Net("VBAT_SHUNT")
    r(vbat, vshunt, "10m", "Resistor_SMD:R_1206_3216Metric")
    # Battery path must stay <= 50 mOhm (power_sim.py); AO3401A (85 mOhm at -2.5 V) failed. Part not chosen yet.
    q = Part("Device", "Q_PMOS", ref="Q1", value="PMOS Rds(on)<=30m@Vgs-2.5V",
             footprint="Package_TO_SOT_SMD:SOT-23")
    q["D"] += vshunt
    q["S"] += vsys
    q["G"] += vbus
    r(vbus, gnd, "100k")
    d = Part("Device", "D_Schottky", ref="D1", value="B5819W", footprint="Diode_SMD:D_SOD-123")
    d["A"] += vbus
    d["K"] += vsys

    # 3.3 V buck-boost, TPS63802 datasheet Fig. 10-1: VOUT = 0.5 V * (1 + 511k/91k) = 3.31 V.
    bb = TPS63802()
    bb["VIN", "EN"] += vsys
    bb["MODE", "AGND", "GND"] += gnd
    bb["PG"] += builtins.NC
    bb["VOUT"] += v33
    lx = Part("Device", "L", value="0.47u", footprint="Inductor_SMD:L_Coilcraft_XFL4015-xxx")
    lx[1] += bb["L1"]
    lx[2] += bb["L2"]
    fb = Net("BB_FB")
    bb["FB"] += fb
    r(v33, fb, "511k")
    r(fb, gnd, "91k")
    c(vsys, gnd, "10u", C0603)
    c(vsys, gnd, "100u", "Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B")  # rides out ESC current spikes
    for _ in range(2):
        c(v33, gnd, "22u", C0603)

    # Core rail: TLV62569 per the P4 Function-EV board v1.5.2 (U7), populated as rev v3 requires.
    buck = Part("Regulator_Switching", "TLV62569DBV")
    buck["VIN"] += v33
    buck["GND"] += gnd
    buck["EN"] += p4["EN_DCDC"]
    c(v33, gnd, "4.7u", C0603)
    l2 = Part("Device", "L", value="2.2u", footprint="Inductor_SMD:L_0806_2016Metric")
    l2[1] += buck["SW"]
    l2[2] += vhp
    hp_fb = Net("HP_FB")
    buck["FB"] += hp_fb
    hp_fb += p4["FB_DCDC"]
    r(vhp, hp_fb, "499k")
    c(vhp, hp_fb, "22p")
    r(hp_fb, gnd, "499k")
    c(vhp, gnd, "22u", C0603)

    # P4 supplies (HDG schematic checklist).
    for pin in ("VDD_LDO", "VDD_DCDCC", "VDD_ANA", "VDD_LP", "VDD_BAT", "VDD_FLASH_IO",
                "VDD_IO_0", "VDD_IO_4", "VDD_IO_5", "VDD_IO_6"):
        v33 += p4[pin]
        c(v33, gnd, "100n")
    c(v33, gnd, "10u", C0603)
    for pin in ("VDD_HP_0", "VDD_HP_1", "VDD_HP_2", "VDD_HP_3"):
        vhp += p4[pin]
        c(vhp, gnd, "100n")
    vpsram = Net("VDD_PSRAM")
    vpsram.drive = POWER
    vpsram += p4["VDDO_PSRAM", "VDD_PSRAM_0", "VDD_PSRAM_1"]
    c(vpsram, gnd, "1u")
    c(vpsram, gnd, "100n")
    for pin in ("VDDO_FLASH", "VDDO_3", "VDDO_4"):  # internal LDO outputs, unused: cap only
        c(net(pin, p4[pin]), gnd, "1u")
    p4["GND"] += gnd
    for pin in p4.pins:  # MIPI DSI/CSI, HS USB and their PHY supplies are unused (may float per HDG)
        if pin.name.startswith(("DSI_", "CSI_", "USB-", "VDD_MIPI", "VDD_USBPHY")):
            pin += builtins.NC

    # Clock, reset, straps.
    x = Part("Device", "Crystal_GND24", ref="Y1", value="40MHz 10ppm CL10pF",
             footprint="Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm")
    x[2, 4] += gnd
    c(net("XTAL_P", x[1], p4["XTAL_P"]), gnd, "12p")  # C = 2 * (CL - ~4 pF stray)
    c(net("XTAL_N", x[3], p4["XTAL_N"]), gnd, "12p")
    pu = net("CHIP_PU", p4["CHIP_PU"])
    r(v33, pu, "10k")
    c(pu, gnd, "1u")
    for n in (35, 36):  # 35 = 1: SPI boot; 35 = 0 with 36 = 1: joint download
        r(v33, net(f"STRAP{n}", gpio(p4, n)), "10k")
    boot = Part("Switch", "SW_Push", ref="SW1", value="BOOT", footprint="Button_Switch_SMD:SW_SPST_PTS810")
    boot[1] += gpio(p4, 35).net
    boot[2] += gnd
    # ponytail: GPIO34 (JTAG source) only matters once the JTAG-select eFuse is burned; pulled low, re-check
    # with the P4 TRM before layout.
    r(net("STRAP34", gpio(p4, 34)), gnd, "10k")

    # Flash, 3.3 V, dedicated pins.
    fl = Part("Memory_Flash", "W25Q128JVS")
    fl["VCC"] += v33
    fl["GND"] += gnd
    c(v33, gnd, "100n")
    for p4pin, flpin in (("FLASH_CS", 1), ("FLASH_Q", 2), ("FLASH_WP", 3), ("FLASH_D", 5), ("FLASH_CK", 6),
                         ("FLASH_HOLD", 7)):
        net(p4pin, p4[p4pin], fl[flpin])
    r(v33, p4["FLASH_CS"].net, "10k")

    # USB-C device (USB-Serial-JTAG on GPIO24/25).
    usb = Part("Connector", "USB_C_Receptacle_USB2.0_14P", ref="J1",
               footprint="Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal")
    usb["VBUS"] += vbus
    usb["GND", "SHIELD"] += gnd
    for cc in ("CC1", "CC2"):
        r(net(cc, usb[cc]), gnd, "5.1k")
    for name, pin, n in (("USB_DN", "D-", 24), ("USB_DP", "D+", 25)):
        r(net(name, *usb[pin]), net(name + "_P4", gpio(p4, n)), "22")
    c(vbus, gnd, "10u", C0603)

    # GPIO signal nets.
    for k, n in P4.items():
        sig[k] += gpio(p4, n)

    # IMU on SPI2 (4-wire), DS-000347 section 4.3 caps.
    imu = ICM42688P()
    imu["VDD", "VDDIO"] += v33
    imu["GND", "RESV2", "RESV3", "RESV7", "RESV10", "RESV11", "FSYNC"] += gnd
    c(v33, gnd, "100n")
    c(v33, gnd, "2.2u")
    c(v33, gnd, "10n")
    for pin, s in (("CS", "IMU_CS"), ("SCLK", "IMU_SCLK"), ("SDI", "IMU_MOSI"), ("SDO", "IMU_MISO"),
                   ("INT1", "IMU_INT")):
        sig[s] += imu[pin]

    # I2C: barometer (0x46) and battery monitor (0x40).
    r(v33, sig["SDA"], "4.7k")
    r(v33, sig["SCL"], "4.7k")
    baro = BMP581()
    baro["VDD", "VDDIO", "CSB"] += v33  # CSB high: I2C
    baro["VSS3", "VSS8", "VSS9", "SDO", "INT"] += gnd  # SDO low: 0x46; INT unused
    baro["SCK"] += sig["SCL"]
    baro["SDI"] += sig["SDA"]
    c(v33, gnd, "100n")
    c(v33, gnd, "100n")
    ina = Part("Sensor_Energy", "INA226")
    ina["VS"] += v33
    ina["GND", "A0", "A1"] += gnd
    ina["~{Alert}"] += builtins.NC
    ina["Vin+", "Vbus"] += vbat
    ina["Vin-"] += vshunt
    ina["SDA"] += sig["SDA"]
    ina["SCL"] += sig["SCL"]
    c(v33, gnd, "100n")

    # Connectors: ESC (8-pin JST-SH, check the ESC's pinout), CRSF receiver, flow/ToF module, C5 radio stack.
    def jst(n, ref, nets):
        j = Part("Connector_Generic", f"Conn_01x{n:02d}", ref=ref,
                 footprint=f"Connector_JST:JST_SH_SM{n:02d}B-SRSS-TB_1x{n:02d}-1MP_P1.00mm_Horizontal")
        for i, net in enumerate(nets, 1):
            net += j[i]
        return j

    jst(8, "J2", [vbat, gnd, sig["ESC_CUR"], sig["ESC_TLM"], sig["M1"], sig["M2"], sig["M3"], sig["M4"]])
    jst(4, "J3", [v33, gnd, sig["CRSF_TX"], sig["CRSF_RX"]])
    jst(8, "J4", [v33, gnd, sig["C5_SCLK"], sig["C5_MOSI"], sig["C5_MISO"], sig["FLOW_CS"], sig["SDA"], sig["SCL"]])
    jst(3, "J5", [gnd, net("U0TXD", gpio(p4, 37)), net("U0RXD", gpio(p4, 38))])  # debug UART0
    stack_connector({"3V3": v33, "GND": gnd, **sig})

    for pin in p4.pins:  # spare GPIOs
        if not pin.is_connected():
            pin += builtins.NC


def radio_board():
    v33, gnd = power("3V3", "GND")
    sig = {k: Net(k) for k in C5}
    c5 = Part("Espressif", "ESP32-C5-MINI-1", ref="U1")
    c5["3V3"] += v33
    for pin in c5.pins:
        if pin.name == "GND":
            pin += gnd
    c(v33, gnd, "22u", C0603)  # 426 mA TX bursts
    c(v33, gnd, "10u", C0603)
    c(v33, gnd, "100n")
    en, boot = net("C5_EN", c5["EN/CHIP_PU"]), net("C5_BOOT", gpio(c5, 28))  # P4 pulls BOOT low to download
    r(v33, en, "10k")
    c(en, gnd, "1u")
    r(v33, boot, "10k")
    for k, n in C5.items():
        sig[k] += gpio(c5, n)
    stack_connector({"3V3": v33, "GND": gnd, "C5_EN": en, "C5_BOOT": boot, **sig})
    for pin in c5.pins:
        if not pin.is_connected():
            pin += builtins.NC


BOARDS = {"fc_board": fc_board, "radio_board": radio_board}


def build(name):
    """Build one board into its own Circuit and return it."""
    circuit = Circuit(name=name)
    with circuit:
        BOARDS[name]()
    for part in circuit.parts:
        part.tag = part.ref  # stable netlist timestamps, so KiCad keeps footprint placement across regenerations
    return circuit


if __name__ == "__main__":
    (HERE / "out").mkdir(exist_ok=True)
    for name in BOARDS:
        ckt = build(name)
        ckt.ERC()
        ckt.generate_netlist(file_=str(HERE / "out" / f"{name}.net"))
