# hardware

KiCad design for the ESP32-P4 + ESP32-C5 flight controller (Phase 3, gate G4), written as SKiDL code in
`boards.py`; `make hw` builds the netlists and runs the checks. `make hw-pcb` places and autoroutes both boards
against JLCPCB's 4-layer rules; `make hw-gerbers` writes the JLCPCB upload set (Gerber zips in `out/gerbers/`,
assembly BOM and pick-and-place CSVs in `out/assembly/`). Decisions and sources: `DESIGN.md`; order status and
open decisions: the "Hardware status" section of the root `README.md`.
Upstream ESP-Drone reference hardware remains in `firmware/flight/hardware/`.
