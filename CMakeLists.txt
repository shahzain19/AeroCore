cmake_minimum_required(VERSION 3.22)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(AeroCore
    VERSION 2.0.0
    DESCRIPTION "Open-source flight controller: sim + embedded HAL"
    LANGUAGES CXX
)

# sim = desktop simulator (default); stm32/linux-sbc/esp32 = firmware scaffolds
set(AEROCORE_TARGET "sim" CACHE STRING "Build target: sim, stm32, linux-sbc, esp32")
set(AEROCORE_VALID_TARGETS sim stm32 linux-sbc esp32)
list(FIND AEROCORE_VALID_TARGETS "${AEROCORE_TARGET}" _aerocore_target_index)
if(_aerocore_target_index EQUAL -1)
    message(FATAL_ERROR "Unsupported AEROCORE_TARGET '${AEROCORE_TARGET}'. Supported values: sim, stm32, linux-sbc, esp32")
endif()

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
#  Build configuration
# ============================================================
set(AEROCORE_BUILD_SIM OFF)
if(AEROCORE_TARGET STREQUAL "sim")
    set(AEROCORE_BUILD_SIM ON)
endif()

# ============================================================
#  Dependencies
# ============================================================
include(GNUInstallDirs)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
if(AEROCORE_BUILD_SIM)
    find_package(SFML COMPONENTS Graphics Window System REQUIRED)
endif()

# ============================================================
#  Source layout
# ============================================================
file(GLOB_RECURSE ALL_SRC_CPP
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

set(SIM_ONLY_SOURCES
    src/HeadedSimulation.cpp
    src/HeadlessSimulation.cpp
    src/Renderer.cpp
    src/SimulationEngine.cpp
    src/platforms/sim/SimIMU.cpp
    src/platforms/sim/SimBarometer.cpp
    src/platforms/sim/SimRCInput.cpp
)

set(CORE_SOURCES ${ALL_SRC_CPP})
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/main\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/HeadedSimulation\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/HeadlessSimulation\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/Renderer\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/SimulationEngine\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/platforms/sim/SimIMU\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/platforms/sim/SimBarometer\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*/platforms/sim/SimRCInput\\.cpp$")

# ============================================================
#  Library targets
# ============================================================
add_library(AeroCoreCore STATIC ${CORE_SOURCES})
target_include_directories(AeroCoreCore PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(AeroCoreCore PUBLIC Eigen3::Eigen)
target_compile_definitions(AeroCoreCore PRIVATE AEROCORE_BUILD_CORE)

install(TARGETS AeroCoreCore
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(AEROCORE_BUILD_SIM)
    add_library(AeroCoreSim STATIC ${SIM_ONLY_SOURCES})
    target_include_directories(AeroCoreSim PUBLIC
        ${PROJECT_SOURCE_DIR}/include
    )
    target_link_libraries(AeroCoreSim PUBLIC AeroCoreCore
        sfml-graphics
        sfml-window
        sfml-system
    )

    add_executable(${PROJECT_NAME} src/main.cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE AeroCoreSim)

    # Copy config to build directory
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${PROJECT_SOURCE_DIR}/config"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/config"
        COMMENT "Copying config files to build directory"
    )

    install(TARGETS AeroCoreSim ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/aerocore/config)
endif()

# ============================================================
#  Compiler flags
# ============================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_link_options($<$<CONFIG:Debug>:-fsanitize=address,undefined>)
elseif(MSVC)
    add_compile_options(/W4)
endif()

# ============================================================
#  Tests
# ============================================================
option(AEROCORE_BUILD_TESTS "Build AeroCore unit tests" ON)
if(AEROCORE_BUILD_TESTS)
    enable_testing()

    function(aerocore_add_test TEST_NAME)
        set(options "")
        set(oneValueArgs "")
        set(multiValueArgs EXTRA_LIBS)
        cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

        add_executable(${TEST_NAME} tests/${TEST_NAME}.cpp)
        target_include_directories(${TEST_NAME} PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${PROJECT_SOURCE_DIR}/tests
        )
        target_link_libraries(${TEST_NAME} PRIVATE Eigen3::Eigen AeroCoreCore)
        if(ARG_EXTRA_LIBS)
            target_link_libraries(${TEST_NAME} PRIVATE ${ARG_EXTRA_LIBS})
        endif()
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endfunction()

    aerocore_add_test(test_pid_controller)
    aerocore_add_test(test_config)
    aerocore_add_test(test_telemetry_manager)
    aerocore_add_test(test_cli_args)
    aerocore_add_test(test_flight_mode)
    aerocore_add_test(test_physics_engine)
    aerocore_add_test(test_estimator)

    if(AEROCORE_BUILD_SIM)
        aerocore_add_test(test_simulation_engine EXTRA_LIBS "AeroCoreSim")
        aerocore_add_test(test_pre_arm EXTRA_LIBS "AeroCoreSim")
        aerocore_add_test(test_imu_fusion EXTRA_LIBS "AeroCoreSim")
    endif()
endif()
