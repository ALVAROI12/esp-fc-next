include(FetchContent)

FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG b6763fbd9cedfacaa89e2ad9fd00d615a234e355 # v2.7.0
)
FetchContent_MakeAvailable(unity)

# One executable per test file; each is registered with CTest.
function(fc_core_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE fc_core unity)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

fc_core_test(test_num)
fc_core_test(test_filter)
fc_core_test(test_pid)
fc_core_test(test_power_distribution)
fc_core_test(test_controller_pid)
fc_core_test(test_sensfusion6)
fc_core_test(test_kalman_core)
fc_core_test(test_fc_step)
