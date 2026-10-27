# fc_core host build: the portable Crazyflie flight algorithms plus the synchronous fc_step loop,
# compiled for the development machine. Produces:
#   fc_core  - static library for unit tests (optionally with ASan/UBSan or coverage)
#   fc_sitl  - shared library loaded by the Python SITL (sim/sitl), never sanitized
# On the ESP32 target the algorithm sources are built by the crazyflie ESP-IDF component in firmware/flight.
cmake_minimum_required(VERSION 3.20)
project(fc_core C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(COMPONENTS ${CMAKE_CURRENT_LIST_DIR}/../flight/components)
set(CF ${COMPONENTS}/core/crazyflie)

# Portable algorithm sources, used in place from the ESP-Drone subtree.
set(CF_SOURCES
    ${CF}/modules/src/pid.c
    ${CF}/modules/src/attitude_pid_controller.c
    ${CF}/modules/src/controller.c
    ${CF}/modules/src/controller_pid.c
    ${CF}/modules/src/controller_indi.c
    ${CF}/modules/src/controller_mellinger.c
    ${CF}/modules/src/position_controller_pid.c
    ${CF}/modules/src/position_controller_indi.c
    ${CF}/modules/src/power_distribution_stock.c
    ${CF}/modules/src/kalman_core.c
    ${CF}/modules/src/kalman_supervisor.c
    ${CF}/modules/src/outlierFilter.c
    ${CF}/modules/src/sensfusion6.c
    ${CF}/modules/src/pptraj.c
    ${CF}/modules/src/pptraj_compressed.c
    ${CF}/modules/src/planner.c
    ${CF}/utils/src/filter.c
    ${CF}/utils/src/num.c
)

# kalman_core needs matrix ops plus dsp_sin/cos (FastMath + their lookup tables).
file(GLOB DSP_SOURCES
    ${COMPONENTS}/lib/dsp_lib/MatrixFunctions/*.c
    ${COMPONENTS}/lib/dsp_lib/FastMathFunctions/*.c
    ${COMPONENTS}/lib/dsp_lib/CommonTables/*.c
)

# ESP-FC Next portable loop (fc_step + synchronous Kalman) and host HAL.
set(FC_SOURCES src/fc_step.c src/fc_kalman.c src/fc_profile.c src/fc_rpm_notch.c src/fc_link.c)
set(HOST_SOURCES host/src/fc_host.c host/src/fc_vars.c)

function(fc_configure target)
    # Host shims come first so they shadow the ESP-IDF/FreeRTOS headers and param.h/log.h.
    target_include_directories(${target} PUBLIC
        include
        src
        host/include
        ${CF}/modules/interface
        ${CF}/utils/interface
        ${CF}/hal/interface
        ${COMPONENTS}/config/include
        ${COMPONENTS}/platform
        ${COMPONENTS}/drivers/general/motors/include
        ${COMPONENTS}/lib/dsp_lib/include
    )
    # UNIT_TEST_MODE: Crazyflie macros skip the ELF linker sections; host/include re-adds the tables.
    target_compile_definitions(${target} PUBLIC UNIT_TEST_MODE FC_HOST_BUILD)
    target_compile_options(${target} PRIVATE -fno-strict-aliasing -Wno-format)
    target_link_libraries(${target} PUBLIC m)
endfunction()

# Planned ESP-FC Next timing on the P4 (G1 report): 4 kHz main + attitude loop, 150 Hz gyro LPF,
# 500 Hz Kalman prediction. Without these definitions the stock 1 kHz ESP-Drone timing is built.
set(FC_4K_DEFINITIONS RATE_MAIN_LOOP=4000 ATTITUDE_RATE=4000 FC_GYRO_LPF_HZ=150.0f FC_KALMAN_PREDICT_HZ=500)

set(ALL_SOURCES ${CF_SOURCES} ${DSP_SOURCES} ${FC_SOURCES} ${HOST_SOURCES})

add_library(fc_core STATIC ${ALL_SOURCES})
fc_configure(fc_core)
add_library(fc_core_4k STATIC ${ALL_SOURCES})
fc_configure(fc_core_4k)
target_compile_definitions(fc_core_4k PUBLIC ${FC_4K_DEFINITIONS})

add_library(fc_sitl SHARED ${ALL_SOURCES})
fc_configure(fc_sitl)
target_compile_options(fc_sitl PRIVATE -O2)
add_library(fc_sitl_4k SHARED ${ALL_SOURCES})
fc_configure(fc_sitl_4k)
target_compile_definitions(fc_sitl_4k PUBLIC ${FC_4K_DEFINITIONS})
target_compile_options(fc_sitl_4k PRIVATE -O2)

option(FC_SANITIZE "Build fc_core and tests with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(FC_SANITIZE)
    foreach(lib fc_core fc_core_4k)
        target_compile_options(${lib} PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${lib} PUBLIC -fsanitize=address,undefined)
    endforeach()
endif()

option(FC_COVERAGE "Instrument fc_core for gcov-style coverage (use with gcovr)" OFF)
if(FC_COVERAGE)
    target_compile_options(fc_core PUBLIC --coverage -O0 -g)
    target_link_options(fc_core PUBLIC --coverage)
endif()

# libFuzzer target for the link codec (needs LLVM clang, e.g. Homebrew llvm on macOS).
option(FC_FUZZ "Build the fc_link libFuzzer target" OFF)
if(FC_FUZZ)
    add_executable(fuzz_link tests/fuzz_link.c src/fc_link.c)
    target_include_directories(fuzz_link PRIVATE include)
    target_compile_options(fuzz_link PRIVATE -g -O1 -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all)
    target_link_options(fuzz_link PRIVATE -fsanitize=fuzzer,address,undefined)
    target_link_libraries(fuzz_link PRIVATE m)
endif()

# The SITL turns this off so its configure step needs no network (Unity is fetched from GitHub).
option(FC_BUILD_TESTS "Build the Unity test suites" ON)
if(FC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
