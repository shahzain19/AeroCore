/**
 * @file STM32RCInput.cpp
 * @brief STM32 RC input implementation
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "platforms/stm32/STM32RCInput.h"
#include "Utilities/Logger.h"
#include <algorithm>
#include <cstring>

namespace AeroCore {
namespace Platform {
namespace STM32 {

STM32RCInput::STM32RCInput(const RCConfig& config)
    : config_(config)
    , initialized_(false)
    , channels_()
    , last_frame_time_(0)
    , frame_count_(0)
    , error_count_(0)
    , sbus_index_(0)
    , sbus_synced_(false)
    , crsf_index_(0)
    , crsf_synced_(false)
{
    channels_.fill(0.0);
    rc_channels_.channels.fill(0.0);
    rc_channels_.valid = false;
    rc_channels_.link_status = HAL::RCLinkStatus::Lost;
    rc_channels_.channel_count = 0;
}

STM32RCInput::~STM32RCInput() {
    // Cleanup if needed
}

void STM32RCInput::initialize() {
    if (initialized_) {
        Utilities::Logger::getInstance().warning("STM32RCInput already initialized");
        return;
    }

    Utilities::Logger::getInstance().info("Initializing STM32 RC input");

    initGPIO();
    initUART();

    initialized_ = true;
    Utilities::Logger::getInstance().info("STM32 RC input initialized");
}

HAL::RCChannels STM32RCInput::read() {
    // Update timestamp and link status
    uint32_t current_time = getCurrentTime();
    uint32_t time_since_frame = current_time - last_frame_time_;

    if (time_since_frame > 100) {  // 100ms timeout
        rc_channels_.link_status = HAL::RCLinkStatus::Lost;
        rc_channels_.valid = false;
    } else if (time_since_frame > 50) {  // 50ms warning threshold
        rc_channels_.link_status = HAL::RCLinkStatus::Degraded;
        rc_channels_.valid = true;
    } else {
        rc_channels_.link_status = HAL::RCLinkStatus::OK;
        rc_channels_.valid = true;
    }

    // Copy current channel values
    for (size_t i = 0; i < std::min(channels_.size(), rc_channels_.channels.size()); ++i) {
        rc_channels_.channels[i] = channels_[i];
    }

    rc_channels_.channel_count = std::min((size_t)16, channels_.size());

    return rc_channels_;
}

uint32_t STM32RCInput::msSinceLastFrame() const {
    return getCurrentTime() - last_frame_time_;
}

void STM32RCInput::processByte(uint8_t data) {
    if (!initialized_) {
        return;
    }

    if (config_.protocol == RCProtocol::SBUS || config_.protocol == RCProtocol::SBUS_INVERTED) {
        // SBUS frame processing
        if (!sbus_synced_) {
            if (data == 0x0F) {  // SBUS header byte
                sbus_buffer_[0] = data;
                sbus_index_ = 1;
                sbus_synced_ = true;
            }
        } else {
            sbus_buffer_[sbus_index_++] = data;

            if (sbus_index_ >= 25) {  // SBUS frame is 25 bytes
                if (processSBUSFrame()) {
                    frame_count_++;
                } else {
                    error_count_++;
                }
                sbus_synced_ = false;
                sbus_index_ = 0;
            }
        }
    } else if (config_.protocol == RCProtocol::CRSF) {
        // CRSF frame processing
        if (!crsf_synced_) {
            if (data == 0xC8) {  // CRSF sync byte
                crsf_buffer_[0] = data;
                crsf_index_ = 1;
                crsf_synced_ = true;
            }
        } else {
            crsf_buffer_[crsf_index_++] = data;

            // CRSF frame length is in byte 1
            if (crsf_index_ >= 2) {
                uint8_t frame_length = crsf_buffer_[1];
                if (crsf_index_ >= frame_length + 2) {  // +2 for sync and length bytes
                    if (processCRSFFrame()) {
                        frame_count_++;
                    } else {
                        error_count_++;
                    }
                    crsf_synced_ = false;
                    crsf_index_ = 0;
                }
            }
        }
    }
}

uint32_t STM32RCInput::getFrameCount() const {
    return frame_count_;
}

uint32_t STM32RCInput::getErrorCount() const {
    return error_count_;
}

// ============================================================
//  Private methods
// ============================================================

void STM32RCInput::initGPIO() {
    // Placeholder for STM32 GPIO initialization
    // Real implementation would use HAL_GPIO_Init()
    Utilities::Logger::getInstance().debug("Init GPIO for RC UART " +
                                           std::to_string(config_.uart_instance));
}

void STM32RCInput::initUART() {
    // Placeholder for STM32 UART initialization
    // Real implementation would:
    // 1. Enable UART clock
    // 2. Configure UART with specified baud rate
    // 3. Enable RX interrupt or DMA
    // 4. Configure GPIO for alternate function
    Utilities::Logger::getInstance().debug("Init RC UART " +
                                           std::to_string(config_.uart_instance) +
                                           " at " + std::to_string(config_.baud_rate) + " baud");
}

bool STM32RCInput::processSBUSFrame() {
    // Check SBUS footer
    if (sbus_buffer_[24] != 0x00) {
        return false;  // Invalid footer
    }

    // Check SBUS flags (byte 23)
    uint8_t flags = sbus_buffer_[23];
    // Bit 0: frame lost
    // Bit 1: failsafe
    if (flags & 0x03) {
        return false;  // Frame lost or failsafe
    }

    extractSBUSChannels();

    last_frame_time_ = getCurrentTime();
    return true;
}

bool STM32RCInput::processCRSFFrame() {
    // Check CRSF frame type (byte 2)
    uint8_t frame_type = crsf_buffer_[2];

    // We're interested in RC channels frame (type 0x16)
    if (frame_type != 0x16) {
        return false;  // Not the frame type we want
    }

    extractCRSFChannels();

    last_frame_time_ = getCurrentTime();
    return true;
}

void STM32RCInput::extractSBUSChannels() {
    // SBUS packs 16 channels (11 bits each) into 22 bytes
    // Channels 1-16 are in bytes 1-22

    for (size_t i = 0; i < 16; ++i) {
        uint16_t raw_value = 0;
        uint8_t byte_index = 1 + (i * 11) / 8;
        uint8_t bit_offset = (i * 11) % 8;

        if (bit_offset <= 5) {
            // Channel fits within 2 bytes
            raw_value = ((uint16_t)sbus_buffer_[byte_index] << 3) |
                        ((uint16_t)sbus_buffer_[byte_index + 1] >> (5 - bit_offset));
        } else {
            // Channel spans 3 bytes
            raw_value = ((uint16_t)sbus_buffer_[byte_index] << (bit_offset - 5)) |
                        ((uint16_t)sbus_buffer_[byte_index + 1] << (3 + (8 - bit_offset))) |
                        ((uint16_t)sbus_buffer_[byte_index + 2] >> (13 - bit_offset));
        }

        raw_value = raw_value & 0x07FF;  // Mask to 11 bits

        // SBUS range: 192-2047 (centered at 992)
        // Normalize to [-1, 1]
        channels_[i] = normalizeChannel(raw_value, 192, 2047);
    }
}

void STM32RCInput::extractCRSFChannels() {
    // CRSF RC channels frame format:
    // Byte 0: Sync (0xC8)
    // Byte 1: Length
    // Byte 2: Frame type (0x16)
    // Byte 3-22: Channel data (4 bytes per channel, 16-bit big-endian)

    size_t channel_count = (crsf_buffer_[1] - 2) / 4;  // -2 for sync and type bytes

    for (size_t i = 0; i < std::min(channel_count, (size_t)16); ++i) {
        size_t byte_index = 3 + (i * 4);

        // CRSF channels are 16-bit big-endian
        uint16_t raw_value = ((uint16_t)crsf_buffer_[byte_index] << 8) |
                            (uint16_t)crsf_buffer_[byte_index + 1];

        // CRSF range: 0-2047 (centered at 1024)
        // Normalize to [-1, 1]
        channels_[i] = normalizeChannel(raw_value, 0, 2047);
    }
}

double STM32RCInput::normalizeChannel(uint16_t raw, uint16_t min, uint16_t max) const {
    if (max <= min) {
        return 0.0;
    }

    double normalized = 2.0 * (raw - min) / (max - min) - 1.0;
    return std::clamp(normalized, -1.0, 1.0);
}

uint32_t STM32RCInput::getCurrentTime() const {
    // Placeholder for getting current time
    // Real implementation would use HAL_GetTick() or similar
    // For now, return frame count as a proxy
    return frame_count_;
}

} // namespace STM32
} // namespace Platform
} // namespace AeroCore
