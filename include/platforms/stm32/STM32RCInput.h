/**
 * @file STM32RCInput.h
 * @brief STM32 RC input implementation for SBUS and CRSF protocols.
 *
 * This implementation provides RC input for STM32-based flight controllers
 * using common RC receiver protocols:
 * - SBUS: Futaba S.BUS (100kbps inverted UART)
 * - CRSF: Crossfire (ELRS) (416kbps standard UART)
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "HAL/IRCInput.h"
#include <array>
#include <cstdint>

namespace AeroCore {
namespace Platform {
namespace STM32 {

/**
 * @brief RC input protocol selection
 */
enum class RCProtocol {
    SBUS = 0,        ///< Futaba S.BUS (inverted UART, 100kbps)
    CRSF = 1,        ///< Crossfire (standard UART, 416kbps)
    SBUS_INVERTED = 2 ///< Non-inverted SBUS (some receivers)
};

/**
 * @brief STM32 RC input configuration
 */
struct RCConfig {
    uint32_t uart_instance;        ///< USART/UART peripheral (USART1, USART2, etc.)
    uint32_t baud_rate;           ///< Baud rate (100000 for SBUS, 416667 for CRSF)
    uint16_t rx_pin;              ///< GPIO pin number for RX
    uint16_t pin_af;               ///< Alternate function
    RCProtocol protocol;           ///< Protocol to use
    bool inverted;                ///< Invert UART signals (for SBUS)
};

/**
 * @brief STM32 RC input driver
 *
 * Manages RC input using STM32 UART peripherals. Supports SBUS and CRSF
 * protocols with automatic frame parsing and channel extraction.
 */
class STM32RCInput : public HAL::IRCInput {
public:
    /**
     * @brief Construct with RC configuration
     * @param config RC input configuration
     */
    explicit STM32RCInput(const RCConfig& config);

    ~STM32RCInput() override;

    // ----------------------------------------------------------
    //  IRCInput interface
    // ----------------------------------------------------------

    /**
     * @brief Read current RC channels
     * @return RCChannels structure with current channel data
     */
    HAL::RCChannels read() override;

    /**
     * @brief Get time since last valid frame in milliseconds
     * @return Time since last frame [ms]
     */
    uint32_t msSinceLastFrame() const override;

    // ----------------------------------------------------------
    //  STM32-specific methods
    // ----------------------------------------------------------

    /**
     * @brief Initialize hardware peripherals
     * Call this after system initialization but before first read()
     */
    void initialize();

    /**
     * @brief Process incoming UART data (call from UART ISR)
     * @param data Received byte
     */
    void processByte(uint8_t data);

    /**
     * @brief Get number of frames received
     * @return Total frame count
     */
    uint32_t getFrameCount() const;

    /**
     * @brief Get number of frame errors
     * @return Total error count
     */
    uint32_t getErrorCount() const;

private:
    RCConfig config_;
    bool initialized_;

    // Channel state
    std::array<double, 18> channels_;     ///< Current channel values [-1, 1]
    HAL::RCChannels rc_channels_;         ///< Output structure
    uint32_t last_frame_time_;           ///< Timestamp of last valid frame
    uint32_t frame_count_;               ///< Total frames received
    uint32_t error_count_;               ///< Frame errors

    // SBUS parsing state
    std::array<uint8_t, 25> sbus_buffer_;  ///< SBUS frame buffer
    size_t sbus_index_;                     ///< Current buffer index
    bool sbus_synced_;                      ///< Frame sync state

    // CRSF parsing state
    std::array<uint8_t, 64> crsf_buffer_;  ///< CRSF frame buffer
    size_t crsf_index_;                     ///< Current buffer index
    bool crsf_synced_;                      ///< Frame sync state

    /**
     * @brief Initialize UART peripheral
     */
    void initUART();

    /**
     * @brief Initialize GPIO for UART RX
     */
    void initGPIO();

    /**
     * @brief Process SBUS frame
     * @return true if frame was valid
     */
    bool processSBUSFrame();

    /**
     * @brief Process CRSF frame
     * @return true if frame was valid
     */
    bool processCRSFFrame();

    /**
     * @brief Extract SBUS channels from frame data
     */
    void extractSBUSChannels();

    /**
     * @brief Extract CRSF channels from frame data
     */
    void extractCRSFChannels();

    /**
     * @brief Normalize raw channel value to [-1, 1]
     * @param raw Raw channel value
     * @param min Minimum raw value
     * @param max Maximum raw value
     * @return Normalized value [-1, 1]
     */
    double normalizeChannel(uint16_t raw, uint16_t min, uint16_t max) const;

    /**
     * @brief Get current time in milliseconds
     * @return Current time [ms]
     */
    uint32_t getCurrentTime() const;
};

} // namespace STM32
} // namespace Platform
} // namespace AeroCore
