# fc_core host build: the portable Crazyflie flight algorithms compiled for the
# development machine (unit tests, SITL). On the ESP32 target the same sources
# are built by the crazyflie ESP-IDF component in firmware/flight.
cmake_minimum_required(VERSION 3.20)
project(fc_core C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(COMPONENTS ${CMAKE_CURRENT_LIST_DIR}/../flight/components)
set(CF ${COMPONENTS}/core/crazyflie)

# Portable algorithm sources, used in place from the ESP-Drone subtree.
set(FC_CORE_SOURCES
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

add_library(fc_core STATIC ${FC_CORE_SOURCES} ${DSP_SOURCES} host/src/fc_host.c)

# Host shims come first so they shadow the ESP-IDF/FreeRTOS headers.
target_include_directories(fc_core PUBLIC
    host/include
    ${CF}/modules/interface
    ${CF}/utils/interface
    ${CF}/hal/interface
    ${COMPONENTS}/config/include
    ${COMPONENTS}/platform
    ${COMPONENTS}/drivers/general/motors/include
    ${COMPONENTS}/lib/dsp_lib/include
)

# UNIT_TEST_MODE turns the Crazyflie LOG/PARAM registration macros into no-ops.
target_compile_definitions(fc_core PUBLIC UNIT_TEST_MODE FC_HOST_BUILD)
target_compile_options(fc_core PRIVATE -fno-strict-aliasing -Wno-format)
target_link_libraries(fc_core PUBLIC m)

option(FC_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
if(FC_SANITIZE)
    target_compile_options(fc_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(fc_core PUBLIC -fsanitize=address,undefined)
endif()

option(FC_COVERAGE "Instrument fc_core for gcov-style coverage (use with gcovr)" OFF)
if(FC_COVERAGE)
    target_compile_options(fc_core PUBLIC --coverage -O0 -g)
    target_link_options(fc_core PUBLIC --coverage)
endif()

enable_testing()
add_subdirectory(tests)
