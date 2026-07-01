/**
 * @file STM32MotorOutput.cpp
 * @brief STM32 motor output implementation
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "platforms/stm32/STM32MotorOutput.h"
#include "Utilities/Logger.h"
#include <algorithm>
#include <cmath>

namespace AeroCore {
namespace Platform {
namespace STM32 {

STM32MotorOutput::STM32MotorOutput(const MotorConfig* configs, size_t motor_count)
    : motor_count_(std::min(motor_count, (size_t)8))
    , initialized_(false)
{
    throttles_.fill(0.0);
    for (size_t i = 0; i < motor_count_; ++i) {
        configs_[i] = configs[i];
    }
}

STM32MotorOutput::~STM32MotorOutput() {
    disarmAll();
}

size_t STM32MotorOutput::motorCount() const {
    return motor_count_;
}

void STM32MotorOutput::initialize() {
    if (initialized_) {
        Utilities::Logger::getInstance().warning("STM32MotorOutput already initialized");
        return;
    }

    Utilities::Logger::getInstance().info("Initializing STM32 motor output for " +
                                        std::to_string(motor_count_) + " motors");

    for (size_t i = 0; i < motor_count_; ++i) {
        const auto& config = configs_[i];
        initGPIO(config);

        if (config.protocol == MotorProtocol::PWM) {
            initTimerPWM(config);
        } else {
            initTimerDShot(config);
        }
    }

    initialized_ = true;
    Utilities::Logger::getInstance().info("STM32 motor output initialized");
}

void STM32MotorOutput::write(size_t motor_index, double throttle_0_1) {
    if (motor_index >= motor_count_) {
        Utilities::Logger::getInstance().error("Invalid motor index in write: " +
                                                std::to_string(motor_index));
        return;
    }

    // Clamp throttle to valid range
    throttles_[motor_index] = std::clamp(throttle_0_1, 0.0, 1.0);

    if (!initialized_) {
        // Not initialized yet, just store the value
        return;
    }

    const auto& config = configs_[motor_index];

    if (config.protocol == MotorProtocol::PWM) {
        uint16_t pwm_us = throttleToPWM(throttles_[motor_index], config.pwm_frequency);
        writePWM(motor_index, pwm_us);
    } else {
        uint16_t dshot_value = throttleToDShot(throttles_[motor_index]);
        writeDShot(motor_index, dshot_value);
    }
}

void STM32MotorOutput::disarmAll() {
    Utilities::Logger::getInstance().info("Disarming all motors");
    for (size_t i = 0; i < motor_count_; ++i) {
        write(i, 0.0);
    }
    update();
}

void STM32MotorOutput::arm() {
    Utilities::Logger::getInstance().info("Arming ESCs");
    // For PWM mode, send minimum throttle for ESC initialization
    // For DShot mode, send arming command
    for (size_t i = 0; i < motor_count_; ++i) {
        const auto& config = configs_[i];
        if (config.protocol == MotorProtocol::PWM) {
            // Send minimum PWM for ESC arming
            write(i, 0.05);  // 5% throttle for ESC init
        } else {
            // DShot arming command (value 0)
            writeDShot(i, 0);
        }
    }
    update();
}

void STM32MotorOutput::update() {
    // For DMA-based DShot, this triggers the next frame transmission
    // For PWM, this is typically handled automatically by hardware
    // This is a placeholder for any periodic updates needed
}

double STM32MotorOutput::getThrottle(size_t motor_index) const {
    if (motor_index >= motor_count_) {
        return 0.0;
    }
    return throttles_[motor_index];
}

// ============================================================
//  Private methods
// ============================================================

void STM32MotorOutput::initGPIO(const MotorConfig& config) {
    // Placeholder for STM32 GPIO initialization
    // Real implementation would use HAL_GPIO_Init()
    Utilities::Logger::getInstance().debug("Init GPIO for motor timer " +
                                           std::to_string(config.timer_instance));
}

void STM32MotorOutput::initTimerPWM(const MotorConfig& config) {
    // Placeholder for STM32 timer PWM initialization
    // Real implementation would:
    // 1. Enable timer clock
    // 2. Configure timer for PWM generation
    // 3. Set prescaler and period for desired frequency
    // 4. Configure output channel
    Utilities::Logger::getInstance().debug("Init PWM timer " +
                                           std::to_string(config.timer_instance) +
                                           " at " + std::to_string(config.pwm_frequency) + " Hz");
}

void STM32MotorOutput::initTimerDShot(const MotorConfig& config) {
    // Placeholder for STM32 timer DShot initialization
    // Real implementation would:
    // 1. Enable timer clock
    // 2. Configure timer for high-speed PWM (DShot uses encoded PWM)
    // 3. Set up DMA for frame transmission
    // 4. Configure output channel
    Utilities::Logger::getInstance().debug("Init DShot timer " +
                                           std::to_string(config.timer_instance) +
                                           " at DShot" + std::to_string(config.dshot_rate));
}

uint16_t STM32MotorOutput::throttleToPWM(double throttle, uint16_t frequency) const {
    // Convert normalized throttle to PWM pulse width in microseconds
    // Standard ESC range: 1000us (min) to 2000us (max)
    constexpr uint16_t pwm_min_us = 1000;
    constexpr uint16_t pwm_max_us = 2000;

    uint16_t pulse_us = static_cast<uint16_t>(
        pwm_min_us + throttle * (pwm_max_us - pwm_min_us)
    );

    return pulse_us;
}

uint16_t STM32MotorOutput::throttleToDShot(double throttle) const {
    // Convert normalized throttle to DShot value
    // DShot range: 0 (command) to 2047 (max throttle)
    // Throttle 0-48 is reserved for special commands
    constexpr uint16_t dshot_min = 48;   // Minimum throttle
    constexpr uint16_t dshot_max = 2047; // Maximum throttle

    double scaled = dshot_min + throttle * (dshot_max - dshot_min);
    return static_cast<uint16_t>(std::clamp(scaled, 0.0, 2047.0));
}

void STM32MotorOutput::writePWM(size_t motor_index, uint16_t pwm_us) {
    // Placeholder for writing PWM value to timer CCR register
    // Real implementation would write to TIMx->CCRy
    const auto& config = configs_[motor_index];
    Utilities::Logger::getInstance().debug("Write PWM " + std::to_string(pwm_us) +
                                           "us to motor " + std::to_string(motor_index) +
                                           " (timer " + std::to_string(config.timer_instance) + ")");
}

void STM32MotorOutput::writeDShot(size_t motor_index, uint16_t dshot_value) {
    // Placeholder for writing DShot frame
    // Real implementation would:
    // 1. Encode DShot value to 16-bit frame with telemetry bit and CRC
    // 2. Trigger DMA transmission or manual bit-banging
    const auto& config = configs_[motor_index];
    Utilities::Logger::getInstance().debug("Write DShot " + std::to_string(dshot_value) +
                                           " to motor " + std::to_string(motor_index) +
                                           " (timer " + std::to_string(config.timer_instance) + ")");
}

} // namespace STM32
} // namespace Platform
} // namespace AeroCore
