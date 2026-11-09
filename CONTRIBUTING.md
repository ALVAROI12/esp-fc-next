# Contributing

This project is simulation-first and has never flown. The most useful help right now is running the simulator,
reviewing the board designs and flight code, and reporting what breaks.

## Getting started

Follow the prerequisites in the [README](README.md), then:

```sh
make core-test   # flight algorithms on the PC, with ASan/UBSan
make sim-test    # model and software-in-the-loop tests (about 9 minutes)
make flight      # P4 firmware, in Docker
```

`make hw` checks the boards and needs KiCad 10 and ngspice. CI runs on every pull request.

## Rules

The full list is in [AGENTS.md](AGENTS.md). The ones that matter most:

- Gates G1-G6 pass in order before any hardware purchase or flight.
- Every bug fix in flight code needs a unit test that failed before the fix.
- Flight-loop code uses no FreeRTOS calls and no dynamic allocation, and builds on host and target.
- `firmware/flight/` is a `git subtree` of espressif/esp-drone. Keep patches minimal, mark them `ESP-FC Next:` in a
  comment and list them in `docs/fc_core.md`.
- Review flight-code changes against [docs/review_checklist.md](docs/review_checklist.md).
- Keep Crazyflie CRTP compatibility.
- Never commit `sdkconfig` or anything from `secrets/`.

Contributions are licensed GPL-3.0. Keep upstream copyright headers.

## Safety

Fly only hardware that has passed the gates, with props guarded, in a netted area, with an independent kill switch.
