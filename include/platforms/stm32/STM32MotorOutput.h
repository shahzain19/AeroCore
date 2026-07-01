/**
 * @file STM32MotorOutput.h
 * @brief STM32 motor output implementation using PWM/DShot protocols.
 *
 * This implementation provides motor output for STM32-based flight controllers
 * using standard PWM (ESC PWM) or DShot protocols. It uses STM32 HAL timers
 * for PWM generation and DMA for DShot telemetry if supported.
 *
 * Supported protocols:
 * - PWM: Standard 50Hz-400Hz PWM output for conventional ESCs
 * - DShot: DShot150/300/600/1200 digital protocol for modern ESCs
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "HAL/IMotorOutput.h"
#include <array>
#include <cstdint>

namespace AeroCore {
namespace Platform {
namespace STM32 {

/**
 * @brief Motor output protocol selection
 */
enum class MotorProtocol {
    PWM = 0,        ///< Standard PWM (50Hz-400Hz)
    DSHOT150,      ///< DShot150 (150kHz)
    DSHOT300,      ///< DShot300 (300kHz)
    DSHOT600,      ///< DShot600 (600kHz)
    DSHOT1200      ///< DShot1200 (1.2MHz)
};

/**
 * @brief STM32 motor output configuration
 */
struct MotorConfig {
    uint32_t timer_instance;        ///< TIM peripheral (TIM1, TIM2, etc.)
    uint32_t timer_channel;         ///< Timer channel (1-4)
    uint16_t pin;                   ///< GPIO pin number
    uint16_t pin_af;                ///< Alternate function
    MotorProtocol protocol;         ///< Output protocol
    uint16_t pwm_frequency;         ///< PWM frequency in Hz (for PWM mode)
    uint16_t dshot_rate;           ///< DShot rate in kHz (for DShot mode)
};

/**
 * @brief STM32 motor output driver
 *
 * Manages motor output using STM32 timers. Supports both PWM and DShot
 * protocols. Each motor is assigned to a specific timer channel.
 */
class STM32MotorOutput : public HAL::IMotorOutput {
public:
    /**
     * @brief Construct with motor configuration array
     * @param configs Array of motor configurations (one per motor)
     * @param motor_count Number of motors
     */
    STM32MotorOutput(const MotorConfig* configs, size_t motor_count);

    ~STM32MotorOutput() override;

    // ----------------------------------------------------------
    //  IMotorOutput interface
    // ----------------------------------------------------------

    size_t motorCount() const override;

    /**
     * @brief Write throttle to a specific motor
     * @param motor_index Motor index [0, motor_count)
     * @param throttle_0_1 Normalized throttle [0, 1]
     */
    void write(size_t motor_index, double throttle_0_1) override;

    /**
     * @brief Disarm all motors (force output to minimum/stop)
     */
    void disarmAll() override;

    /**
     * @brief Arm ESCs (protocol-specific initialization)
     */
    void arm() override;

    // ----------------------------------------------------------
    //  STM32-specific methods
    // ----------------------------------------------------------

    /**
     * @brief Initialize hardware peripherals
     * Call this after system initialization but before first write()
     */
    void initialize();

    /**
     * @brief Update all motor outputs (call periodically)
     * For PWM mode, this updates CCR registers. For DShot, it sends
     * the next frame if using DMA-based transmission.
     */
    void update();

    /**
     * @brief Get current throttle value for a motor
     * @param motor_index Motor index
     * @return Normalized throttle [0, 1]
     */
    double getThrottle(size_t motor_index) const;

private:
    std::array<MotorConfig, 8> configs_;  ///< Motor configurations (max 8 motors)
    size_t motor_count_;
    std::array<double, 8> throttles_;     ///< Current throttle values
    bool initialized_;

    /**
     * @brief Initialize GPIO for a motor pin
     */
    void initGPIO(const MotorConfig& config);

    /**
     * @brief Initialize timer for PWM output
     */
    void initTimerPWM(const MotorConfig& config);

    /**
     * @brief Initialize timer for DShot output
     */
    void initTimerDShot(const MotorConfig& config);

    /**
     * @brief Convert normalized throttle to PWM pulse width
     * @param throttle Normalized throttle [0, 1]
     * @param frequency PWM frequency in Hz
     * @return Pulse width in microseconds
     */
    uint16_t throttleToPWM(double throttle, uint16_t frequency) const;

    /**
     * @brief Convert normalized throttle to DShot value
     * @param throttle Normalized throttle [0, 1]
     * @return DShot throttle value [0, 2047]
     */
    uint16_t throttleToDShot(double throttle) const;

    /**
     * @brief Write PWM value to timer CCR register
     */
    void writePWM(size_t motor_index, uint16_t pwm_us);

    /**
     * @brief Write DShot frame to timer
     */
    void writeDShot(size_t motor_index, uint16_t dshot_value);
};

} // namespace STM32
} // namespace Platform
} // namespace AeroCore
