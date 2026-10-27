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

.PHONY: all flight radio core-test core-coverage sim-setup sim-test g1 vibration failsafe flow-braking link-fuzz core-lint viewer pdf clean

all: flight radio core-test

# Host build of the flight algorithms with ASan/UBSan, then run the tests.
core-test:
	cmake -S firmware/fc_core -B build/fc_core -G Ninja -DFC_SANITIZE=ON
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
link-fuzz:
	cmake -S firmware/fc_core -B build/fuzz -G Ninja -DFC_FUZZ=ON -DCMAKE_C_COMPILER=$(FUZZ_CC)
	cmake --build build/fuzz --target fuzz_link
	mkdir -p build/fuzz/corpus
	build/fuzz/fuzz_link -max_total_time=$(FUZZ_SECONDS) -print_final_stats=1 build/fuzz/corpus

# G3 static analysis of ESP-FC Next sources (non-zero exit on any finding).
CF := firmware/flight/components
core-lint:
	cppcheck --quiet --error-exitcode=1 --enable=warning,portability,performance --inline-suppr \
		--suppress=missingInclude --suppress=missingIncludeSystem \
		-DUNIT_TEST_MODE -DFC_HOST_BUILD -DRATE_MAIN_LOOP=1000 \
		-I firmware/fc_core/include -I firmware/fc_core/src -I firmware/fc_core/host/include \
		-I $(CF)/core/crazyflie/modules/interface -I $(CF)/core/crazyflie/utils/interface \
		-I $(CF)/core/crazyflie/hal/interface -I $(CF)/config/include \
		firmware/fc_core/src firmware/fc_core/host/src

flight:
	$(call idf_build,firmware/flight,$(FLIGHT_TARGET))

radio:
	$(call idf_build,firmware/radio,$(RADIO_TARGET))

sim-setup:
	uv venv --python 3.12 sim/.venv
	# pybullet has no macOS arm64 wheel; its bundled zlib breaks on recent SDKs unless stdio.h is pre-included.
	CFLAGS="-include stdio.h" uv pip install --python sim/.venv/bin/python -r sim/requirements.txt

sim-test:
	sim/.venv/bin/python -m pytest sim

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

# Interactive 3D replay of the G1 scenarios -> sim/reports/viewer/index.html (opens in the browser).
viewer:
	cd sim && .venv/bin/python visualize.py --open

pdf:
	uv run --no-project --with reportlab python docs/build_plan_pdf.py

clean:
	rm -rf firmware/flight/build-* firmware/radio/build-* build
