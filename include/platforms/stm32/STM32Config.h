/**
 * @file STM32Config.h
 * @brief STM32 platform-specific configuration for AeroCore.
 *
 * This file contains board-specific pin mappings, timer assignments,
 * and peripheral configurations for STM32-based flight controllers.
 *
 * Currently supports a generic STM32F4 configuration that can be
 * adapted to specific boards (e.g., OMNIBUSF4, JHEF7, etc.).
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "platforms/stm32/STM32MotorOutput.h"
#include "platforms/stm32/STM32RCInput.h"
#include <array>

namespace AeroCore {
namespace Platform {
namespace STM32 {

/**
 * @brief Board type enumeration
 */
enum class BoardType {
    GENERIC_F4 = 0,      ///< Generic STM32F4 board
    OMNIBUSF4,          ///< OmnibusF4SD
    JHEF7,              ///< JHEF7
    HAKRARCF7,          ///< HAKRARCF7
    CL_RACINGF4,        ///< CL_RACINGF4
};

/**
 * @brief STM32 platform configuration
 *
 * Contains all board-specific pin mappings and peripheral assignments.
 * Use getBoardConfig() to get the configuration for a specific board.
 */
struct PlatformConfig {
    BoardType board_type;
    const char* board_name;

    // Motor output configurations (up to 8 motors)
    std::array<STM32MotorOutput::MotorConfig, 8> motor_configs;
    size_t motor_count;

    // RC input configuration
    STM32RCInput::RCConfig rc_config;

    // IMU configuration (SPI/I2C)
    uint32_t imu_spi_instance;
    uint16_t imu_cs_pin;
    uint16_t imu_sck_pin;
    uint16_t imu_miso_pin;
    uint16_t imu_mosi_pin;

    // Barometer configuration (SPI/I2C)
    uint32_t baro_spi_instance;
    uint16_t baro_cs_pin;

    // LED configuration
    uint16_t led_pin;
    uint16_t buzzer_pin;

    // System clock configuration
    uint32_t system_clock_hz;
};

/**
 * @brief Get platform configuration for a specific board
 * @param board Board type
 * @return Platform configuration structure
 */
const PlatformConfig& getBoardConfig(BoardType board);

/**
 * @brief Get default board configuration
 * @return Default platform configuration (GENERIC_F4)
 */
inline const PlatformConfig& getDefaultBoardConfig() {
    return getBoardConfig(BoardType::GENERIC_F4);
}

// ============================================================
//  Board-specific configurations
// ============================================================

/**
 * @brief Generic STM32F4 configuration
 *
 * This is a template configuration that can be adapted to specific
 * STM32F4-based flight controller boards. Modify the pin assignments
 * to match your hardware.
 */
inline PlatformConfig createGenericF4Config() {
    PlatformConfig config;
    config.board_type = BoardType::GENERIC_F4;
    config.board_name = "Generic STM32F4";
    config.system_clock_hz = 168000000;  // 168 MHz

    // Motor outputs (4 motors for quadrotor)
    config.motor_count = 4;

    // Motor 1: TIM1 CH1 (PA8)
    config.motor_configs[0] = {
        .timer_instance = 1,          // TIM1
        .timer_channel = 1,            // Channel 1
        .pin = 8,                     // PA8
        .pin_af = 6,                  // AF6 for TIM1
        .protocol = STM32MotorOutput::MotorProtocol::DSHOT600,
        .pwm_frequency = 400,
        .dshot_rate = 600
    };

    // Motor 2: TIM1 CH2 (PA9)
    config.motor_configs[1] = {
        .timer_instance = 1,          // TIM1
        .timer_channel = 2,            // Channel 2
        .pin = 9,                     // PA9
        .pin_af = 6,                  // AF6 for TIM1
        .protocol = STM32MotorOutput::MotorProtocol::DSHOT600,
        .pwm_frequency = 400,
        .dshot_rate = 600
    };

    // Motor 3: TIM1 CH3 (PA10)
    config.motor_configs[2] = {
        .timer_instance = 1,          // TIM1
        .timer_channel = 3,            // Channel 3
        .pin = 10,                    // PA10
        .pin_af = 6,                  // AF6 for TIM1
        .protocol = STM32MotorOutput::MotorProtocol::DSHOT600,
        .pwm_frequency = 400,
        .dshot_rate = 600
    };

    // Motor 4: TIM1 CH4 (PA11)
    config.motor_configs[3] = {
        .timer_instance = 1,          // TIM1
        .timer_channel = 4,            // Channel 4
        .pin = 11,                    // PA11
        .pin_af = 6,                  // AF6 for TIM1
        .protocol = STM32MotorOutput::MotorProtocol::DSHOT600,
        .pwm_frequency = 400,
        .dshot_rate = 600
    };

    // RC input (USART2 for SBUS)
    config.rc_config = {
        .uart_instance = 2,           // USART2
        .baud_rate = 100000,          // 100kbps for SBUS
        .rx_pin = 3,                  // PA3 (USART2 RX)
        .pin_af = 7,                  // AF7 for USART2
        .protocol = STM32RCInput::RCProtocol::SBUS,
        .inverted = true              // SBUS is inverted
    };

    // IMU (SPI1 for ICM-42688-P)
    config.imu_spi_instance = 1;      // SPI1
    config.imu_cs_pin = 4;            // PA4 (SPI1 NSS)
    config.imu_sck_pin = 5;           // PA5 (SPI1 SCK)
    config.imu_miso_pin = 6;          // PA6 (SPI1 MISO)
    config.imu_mosi_pin = 7;          // PA7 (SPI1 MOSI)

    // Barometer (SPI2 for BMP280)
    config.baro_spi_instance = 2;     // SPI2
    config.baro_cs_pin = 12;          // PB12 (SPI2 NSS)

    // LED and buzzer
    config.led_pin = 13;              // PC13 (common LED pin)
    config.buzzer_pin = 15;           // PB15 (buzzer)

    return config;
}

/**
 * @brief Get board configuration for specific board type
 */
inline const PlatformConfig& getBoardConfig(BoardType board) {
    static PlatformConfig generic_f4 = createGenericF4Config();

    switch (board) {
        case BoardType::GENERIC_F4:
        default:
            return generic_f4;

        // Add more board configurations here as needed
        // case BoardType::OMNIBUSF4:
        //     return createOmnibusF4Config();
        // case BoardType::JHEF7:
        //     return createJHEF7Config();
    }
}

} // namespace STM32
} // namespace Platform
} // namespace AeroCore
