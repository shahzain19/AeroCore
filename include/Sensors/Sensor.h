/**
 * @file Sensor.h
 * @brief Abstract base class for all simulated sensors in AeroCore.
 *
 * ## Sensor Model
 *
 * Every sensor is modelled as:
 *   measured_value = true_value + drift(t) + gaussian_noise
 *
 * where:
 *  - `gaussian_noise` ~ N(0, noise_stddev²)
 *  - `drift(t)`       is a random-walk bias with variance = (drift_rate · t)²
 *
 * Sensors are sampled at a fixed `update_rate` [Hz].  Between samples the
 * last measurement is returned.  The `isReady()` flag is true only in the
 * tick that produced a new sample.
 *
 * ## Design Notes
 *  - The base class handles scalar values.  Multi-axis sensors (Accelerometer,
 *    Gyroscope) override the update method and apply noise per axis.
 *  - The RNG is seeded from the system clock at construction time so each
 *    sensor instance has statistically independent noise.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include <random>

namespace AeroCore {
namespace Sensors {

/**
 * @brief Simulated scalar sensor base class.
 *
 * Concrete sensor classes inherit from this and override `update()` to add
 * per-axis or more specialised noise models.
 */
class Sensor {
public:
    /**
     * @brief Construct a sensor with noise/drift parameters.
     *
     * @param update_rate   How often the sensor produces a new reading [Hz].
     * @param noise_stddev  Standard deviation of Gaussian noise [sensor units].
     * @param drift_rate    Bias random-walk rate [sensor units/s].
     */
    Sensor(double update_rate, double noise_stddev, double drift_rate);

    virtual ~Sensor() = default;

    // ----------------------------------------------------------
    //  Update
    // ----------------------------------------------------------

    /**
     * @brief Advance sensor simulation by @p dt seconds.
     *
     * Advances the internal timer.  When a new sample is due, adds noise
     * and drift to @p true_value and stores the result.
     *
     * @param dt          Time step [s].
     * @param true_value  The ground-truth physical quantity.
     */
    virtual void update(double dt, double true_value);

    // ----------------------------------------------------------
    //  Accessors
    // ----------------------------------------------------------

    /// Returns the most recent measured value.
    double getValue() const;

    /**
     * @brief True if this tick produced a fresh sample.
     *
     * Applications that need to respect sensor update rates should check
     * this before using getValue().
     */
    bool isReady() const;

    /// Sample rate [Hz].
    double getUpdateRate() const;

    /// Noise standard deviation.
    double getNoiseStddev() const;

    // ----------------------------------------------------------
    //  Configuration
    // ----------------------------------------------------------

    void setUpdateRate(double hz);
    void setNoise(double stddev);
    void setDrift(double rate);

    /// Zero internal drift accumulator.
    void resetDrift();

protected:
    // ----------------------------------------------------------
    //  Noise / drift helpers (available to derived classes)
    // ----------------------------------------------------------

    /**
     * @brief Add Gaussian noise to @p value.
     * @return Noisy value.
     */
    double addNoise(double value);

    /**
     * @brief Advance the bias random-walk by @p dt seconds.
     */
    void updateDrift(double dt);

    // Sensor configuration
    double update_rate_;   ///< Sample rate [Hz]
    double noise_stddev_;  ///< Noise standard deviation
    double drift_rate_;    ///< Bias drift rate [units/s]

    // State
    double current_value_;       ///< Last measured value
    double drift_;               ///< Accumulated bias
    double time_since_update_;   ///< Time since last sample [s]
    bool   ready_;               ///< True if a new sample was produced this tick

    // Random number generation
    std::mt19937                        rng_;         ///< Mersenne Twister RNG
    std::normal_distribution<double>    noise_dist_;  ///< Noise distribution N(0, σ²)
};

} // namespace Sensors
} // namespace AeroCore
