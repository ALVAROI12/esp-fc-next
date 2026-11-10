# ESP-FC Next developer entry points. Firmware builds run in the pinned ESP-IDF container.

FLIGHT_TARGET ?= esp32p4
RADIO_TARGET  ?= esp32c5
IDF           := tools/idf.sh

idf_build = $(IDF) $(1) -B build-$(2) -D SDKCONFIG=build-$(2)/sdkconfig -D IDF_TARGET=$(2) build

ifeq ($(shell uname),Darwin)
GCOV ?= xcrun llvm-cov gcov
else
GCOV ?= gcov
endif
# Linked only for controller.c's table; fc_step always selects the PID controller.
COVERAGE_UNREACHABLE := --exclude '.*/controller_indi\.c' --exclude '.*/position_controller_indi\.c' \
	--exclude '.*/controller_mellinger\.c'
COVERAGE_FILTERS := --include-internal-functions --filter 'firmware/flight/components/core/' \
	--filter 'firmware/fc_core/src/' --filter 'firmware/fc_core/host/' $(COVERAGE_UNREACHABLE)

.PHONY: hw-setup hw all flight flight-qemu flight-fcstep flight-c5link radio core-test core-coverage sim-setup sim-test g1 vibration failsafe flow-braking swarm video link-fuzz core-lint core-tidy mbedtls-src viewer pdf clean

all: flight radio core-test

# mbedtls for host tests of the AES-CCM link envelope: the exact copy bundled with the pinned ESP-IDF image,
# so host tests and firmware use the same library version. No network fetch beyond the Docker image.
IDF_IMAGE ?= espressif/idf:release-v5.5
MBEDTLS_DIR := build/mbedtls
$(MBEDTLS_DIR)/CMakeLists.txt:
	mkdir -p build
	docker run --rm -v "$(CURDIR)/build":/out $(IDF_IMAGE) \
		bash -c 'rm -rf /out/mbedtls && cp -r "$$IDF_PATH/components/mbedtls/mbedtls" /out/mbedtls && chown -R $(shell id -u):$(shell id -g) /out/mbedtls'
mbedtls-src: $(MBEDTLS_DIR)/CMakeLists.txt

# Host build of the flight algorithms with ASan/UBSan, then run the tests.
core-test: $(MBEDTLS_DIR)/CMakeLists.txt
	cmake -S firmware/fc_core -B build/fc_core -G Ninja -DFC_SANITIZE=ON -DFC_MBEDTLS_DIR=$(CURDIR)/$(MBEDTLS_DIR)
	cmake --build build/fc_core
	ctest --test-dir build/fc_core --output-on-failure

# Line/branch coverage of fc_core; HTML report in build/coverage/index.html.
core-coverage:
	cmake -S firmware/fc_core -B build/fc_core-cov -G Ninja -DFC_COVERAGE=ON
	cmake --build build/fc_core-cov
	find build/fc_core-cov -name '*.gcda' -delete
	ctest --test-dir build/fc_core-cov --output-on-failure
	mkdir -p build/coverage
	sim/.venv/bin/gcovr --gcov-executable "$(GCOV)" -r . $(COVERAGE_FILTERS) \
		--html-details build/coverage/index.html --print-summary build/fc_core-cov

# libFuzzer run of the P4<->C5 link codec (Task 1.7 gate: 1 h without crashes). FUZZ_SECONDS=60 for a smoke run.
FUZZ_SECONDS ?= 3600
FUZZ_CC ?= $(shell [ -x /opt/homebrew/opt/llvm/bin/clang ] && echo /opt/homebrew/opt/llvm/bin/clang || echo clang)
FUZZ_TARGET ?= fuzz_link
link-fuzz: $(MBEDTLS_DIR)/CMakeLists.txt
	cmake -S firmware/fc_core -B build/fuzz -G Ninja -DFC_FUZZ=ON -DCMAKE_C_COMPILER=$(FUZZ_CC) \
		-DFC_MBEDTLS_DIR=$(CURDIR)/$(MBEDTLS_DIR)
	cmake --build build/fuzz --target $(FUZZ_TARGET)
	mkdir -p build/fuzz/corpus-$(FUZZ_TARGET)
	build/fuzz/$(FUZZ_TARGET) -max_total_time=$(FUZZ_SECONDS) -print_final_stats=1 -artifact_prefix=build/fuzz/ build/fuzz/corpus-$(FUZZ_TARGET)

# G3 static analysis of ESP-FC Next sources (non-zero exit on any finding).
CF := firmware/flight/components
core-lint:
	cppcheck --quiet --error-exitcode=1 --enable=warning,portability,performance --inline-suppr \
		--suppress=missingInclude --suppress=missingIncludeSystem \
		-DUNIT_TEST_MODE -DFC_HOST_BUILD -DRATE_MAIN_LOOP=1000 \
		-I firmware/fc_core/include -I firmware/fc_core/src -I firmware/fc_core/host/include \
		-I $(CF)/core/crazyflie/modules/interface -I $(CF)/core/crazyflie/utils/interface \
		-I $(CF)/core/crazyflie/hal/interface -I $(CF)/config/include \
		-I firmware/fc_core/tests/shim firmware/fc_core/src firmware/fc_core/host/src firmware/fc_core/target

# G3 clang-tidy of ESP-FC Next sources: path-sensitive analyzer + bug-prone patterns, warnings are errors.
# Excluded as noise for this code base: C11 Annex K *_s suggestions (absent in newlib), struct padding, and
# style checks that fire on Crazyflie macros/headers (signed bitwise in LOG/PARAM, macro parens, reserved
# include guards, int->float tick narrowing, small-offset widening).
CLANG_TIDY ?= $(shell [ -x /opt/homebrew/opt/llvm/bin/clang-tidy ] && echo /opt/homebrew/opt/llvm/bin/clang-tidy || echo clang-tidy)
TIDY_CHECKS := -*,clang-analyzer-*,-clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling,\
-clang-analyzer-optin.performance.Padding,bugprone-*,-bugprone-easily-swappable-parameters,\
-bugprone-signed-bitwise,-bugprone-macro-parentheses,-bugprone-narrowing-conversions,\
-bugprone-reserved-identifier,-bugprone-implicit-widening-of-multiplication-result
core-tidy: core-test
	$(CLANG_TIDY) -p build/fc_core --quiet --checks='$(TIDY_CHECKS)' --warnings-as-errors='*' \
		--header-filter='firmware/fc_core/(include|src|target)/.*' \
		firmware/fc_core/src/*.c firmware/fc_core/host/src/*.c firmware/fc_core/target/*.c

flight:
	$(call idf_build,firmware/flight,$(FLIGHT_TARGET))

# ESP32-S3 image booted in Espressif's QEMU inside the IDF container: fails on a panic or if app_main is not reached.
# QEMU has no P4 or C5 machine, no radio and no sensors, so this checks start-up only (log: qemu_uart.log).
flight-qemu:
	$(IDF) firmware/flight -B build-esp32s3-qemu -D SDKCONFIG=build-esp32s3-qemu/sdkconfig -D IDF_TARGET=esp32s3 \
		-D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32s3;../fc_core/target/sdkconfig.qemu" build
	tools/qemu_boot.sh build-esp32s3-qemu

# Same image with fc_step running in the stabilizer task (CONFIG_ESPFC_FC_STEP); compile check, not flight-ready.
flight-fcstep:
	$(IDF) firmware/flight -B build-$(FLIGHT_TARGET)-fcstep -D SDKCONFIG=build-$(FLIGHT_TARGET)-fcstep/sdkconfig \
		-D IDF_TARGET=$(FLIGHT_TARGET) -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;../fc_core/target/sdkconfig.fc_step" build

# P4 image with CRTP over the C5 SPI link (CONFIG_TRANSPORT_C5_SPI); compile check, not run on a board.
flight-c5link:
	$(IDF) firmware/flight -B build-esp32p4-c5link -D SDKCONFIG=build-esp32p4-c5link/sdkconfig \
		-D IDF_TARGET=esp32p4 -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;../fc_core/target/sdkconfig.c5_link" build

radio:
	$(call idf_build,firmware/radio,$(RADIO_TARGET))

sim-setup:
	uv venv --python 3.12 sim/.venv
	# pybullet has no macOS arm64 wheel; its bundled zlib breaks on recent SDKs unless stdio.h is pre-included.
	CFLAGS="-include stdio.h" uv pip install --python sim/.venv/bin/python -r sim/requirements.txt

sim-test:
	sim/.venv/bin/python -m pytest sim

# Phase 3 boards as code (needs KiCad for its symbol library, ngspice for the power check): netlists in
# hardware/out/, then the netlist and power-path checks (gate G4).
hw-setup:
	uv venv --python 3.12 hardware/.venv
	uv pip install --python hardware/.venv/bin/python -r hardware/requirements.txt

hw:
	cd hardware && .venv/bin/python library.py && .venv/bin/python boards.py && .venv/bin/python -m pytest -q -p no:cacheprovider test_boards.py

KICAD_PY = /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
KICAD_CLI = /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
# PCB placement + Freerouting autoroute, 3D renders and the power-path plot in hardware/out/ (after `make hw`;
# routing needs Java 25+ and hardware/tools/freerouting-2.5.0.jar, see hardware/DESIGN.md).
hw-pcb:
	cd hardware && mkdir -p out/render && for b in fc_board radio_board; do \
	  $(KICAD_PY) pcb.py out/$$b.net out/$$b.kicad_pcb 28 20 || exit 1; $(KICAD_PY) route.py out/$$b.kicad_pcb 80 || exit 1; \
	  for s in top bottom; do \
	  $(KICAD_CLI) pcb render --side $$s --width 1400 --height 1400 --quality high --zoom 1 --background opaque \
	  -o out/render/$$b-$$s.png out/$$b-routed.kicad_pcb || exit 1; done; done
	cd hardware && $(KICAD_PY) parasitics.py
	cd hardware && uv run -q --with matplotlib --with-requirements requirements.txt python plot_power.py

# JLCPCB upload: hardware/out/gerbers/<board>.zip (Gerber X1 with Protel extensions, Excellon drills, one origin)
# plus the assembly BOM and pick-and-place CSVs in hardware/out/assembly/.
hw-gerbers:
	cd hardware && for b in fc_board radio_board; do rm -rf out/gerbers/$$b && mkdir -p out/gerbers/$$b && \
	  $(KICAD_CLI) pcb export gerbers -l F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts \
	    --no-x2 --no-netlist --subtract-soldermask -o out/gerbers/$$b/ out/$$b-routed.kicad_pcb || exit 1; \
	  $(KICAD_CLI) pcb export drill --excellon-separate-th --generate-map --map-format gerberx2 \
	    -o out/gerbers/$$b/ out/$$b-routed.kicad_pcb || exit 1; \
	  (cd out/gerbers && rm -f $$b.zip && zip -q -r $$b.zip $$b) || exit 1; \
	  $(KICAD_PY) assembly.py out/$$b-routed.kicad_pcb out/assembly/ || exit 1; done

# Gate G1 math-model report -> sim/reports/g1/README.md (non-zero exit if the gate fails).
g1:
	cd sim && .venv/bin/python g1_report.py

# Task 1.6b vibration/notch report -> sim/reports/vibration/README.md (non-zero exit if a check fails).
vibration:
	cd sim && .venv/bin/python vibration_report.py

# Opt-in command/tracking-loss landing evidence -> sim/reports/failsafe/README.md.
failsafe:
	cd sim && .venv/bin/python failsafe_report.py

flow-braking:
	cd sim && .venv/bin/python flow_braking_report.py

# Task 1.8 four-drone swarm missions with injected faults -> sim/reports/swarm/README.md (non-zero exit on failure).
swarm:
	cd sim && .venv/bin/python swarm_report.py

# Four-drone SITL mission with a link loss mid-swap -> docs/img/swarm-sitl.gif and build/swarm-sitl.mp4 (needs ffmpeg).
video:
	cd sim && .venv/bin/python swarm_video.py

# Interactive 3D replay of the G1 scenarios -> sim/reports/viewer/index.html (opens in the browser).
viewer:
	cd sim && .venv/bin/python visualize.py --open

pdf:
	uv run --no-project --with reportlab python docs/build_plan_pdf.py

clean:
	rm -rf firmware/flight/build-* firmware/radio/build-* build
