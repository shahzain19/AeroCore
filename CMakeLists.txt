cmake_minimum_required(VERSION 3.22)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(AeroCore
    VERSION 2.0.0
    DESCRIPTION "Open-source flight controller: sim + embedded HAL"
    LANGUAGES CXX
)

# sim = desktop simulator (default); stm32/linux-sbc/esp32 = firmware (scaffold)
set(AEROCORE_TARGET "sim" CACHE STRING "Build target: sim, stm32, linux-sbc, esp32")

# ============================================================
#  C++ standard
# ============================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
#  Build type default
# ============================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ============================================================
#  Dependencies
# ============================================================
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(SFML COMPONENTS Graphics Window System REQUIRED)

# ============================================================
#  Sources
# ============================================================
file(GLOB_RECURSE SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# ============================================================
#  Executable
# ============================================================
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Eigen3::Eigen
    sfml-graphics
    sfml-window
    sfml-system
)

# ============================================================
#  Compiler flags
# ============================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -Wextra -Wpedantic
        $<$<CONFIG:Release>:-O3 -DNDEBUG -march=native>
        $<$<CONFIG:Debug>:-g -O0 -fsanitize=address,undefined>
    )
    target_link_options(${PROJECT_NAME} PRIVATE
        $<$<CONFIG:Debug>:-fsanitize=address,undefined>
    )
elseif(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W4
        $<$<CONFIG:Release>:/O2>
    )
endif()

# ============================================================
#  Copy config to build directory
# ============================================================
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/config"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/config"
    COMMENT "Copying config files to build directory"
)

# ============================================================
#  Install
# ============================================================
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/aerocore/config)

# ============================================================
#  Tests
# ============================================================
option(AEROCORE_BUILD_TESTS "Build AeroCore unit tests" ON)
if(AEROCORE_BUILD_TESTS)
    enable_testing()

    function(aerocore_add_test TEST_NAME)
        set(options "")
        set(oneValueArgs "")
        set(multiValueArgs EXTRA_SOURCES)
        cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

        set(SOURCES
            tests/${TEST_NAME}.cpp
            src/Config.cpp
            src/TelemetryManager.cpp
        )
        if(ARG_EXTRA_SOURCES)
            list(APPEND SOURCES ${ARG_EXTRA_SOURCES})
        endif()

        add_executable(${TEST_NAME} ${SOURCES})
        target_include_directories(${TEST_NAME} PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${PROJECT_SOURCE_DIR}/tests
        )
        target_link_libraries(${TEST_NAME} PRIVATE
            Eigen3::Eigen
        )
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endfunction()

    aerocore_add_test(test_pid_controller EXTRA_SOURCES src/PIDController.cpp)
    aerocore_add_test(test_config)
    aerocore_add_test(test_telemetry_manager)
    aerocore_add_test(test_cli_args EXTRA_SOURCES src/CliArgs.cpp)
    aerocore_add_test(test_flight_mode)
    aerocore_add_test(test_physics_engine EXTRA_SOURCES src/PhysicsEngine.cpp)
    aerocore_add_test(test_simulation_engine EXTRA_SOURCES
        src/SimulationEngine.cpp
        src/PhysicsEngine.cpp
        src/Drone.cpp
        src/Motor.cpp
        src/FlightController.cpp
        src/PIDController.cpp
        src/IMU.cpp
        src/Accelerometer.cpp
        src/Gyroscope.cpp
        src/Sensor.cpp
        src/Altimeter.cpp
        src/BatterySensor.cpp
        src/Logger.cpp
        src/Core/ComplementaryEstimator.cpp
        src/platforms/sim/SimIMU.cpp
        src/platforms/sim/SimBarometer.cpp
        src/HAL/Types.cpp
    )
    aerocore_add_test(test_estimator EXTRA_SOURCES
        src/Core/ComplementaryEstimator.cpp
        src/HAL/Types.cpp
    )
endif()
