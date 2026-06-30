#include "Flight/PIDController.h"
#include "test_common.h"

using AeroCore::Flight::AntiWindupMode;
using AeroCore::Flight::PIDController;

int main() {
    PIDController pid(1.0, 0.5, 0.25);
    pid.setOutputLimits(-1.0, 1.0);
    pid.setIntegralMax(0.2);
    pid.setDerivativeFilter(1.0);

    // First update should not inject derivative kick.
    const double out_first = pid.update(0.1, 1.0, 0.0);
    AeroCore::Tests::expectNear(out_first, 1.0, 1e-9, "first output saturates at max");
    AeroCore::Tests::expectNear(pid.getDerivative(), 0.0, 1e-9, "first derivative is zero");

    // In clamp anti-windup mode, integrator should not grow while saturated.
    for (int i = 0; i < 50; ++i) {
        pid.update(0.1, 1.0, 0.0);
    }
    AeroCore::Tests::expectNear(pid.getIntegral(), 0.0, 1e-6,
                                "integral is held when output is saturated");

    // Derivative-on-measurement should react to rising measurement with negative derivative.
    pid.reset();
    pid.update(0.1, 0.0, 0.0);
    pid.update(0.1, 0.0, 2.0);
    AeroCore::Tests::expectTrue(pid.getDerivative() < 0.0, "derivative opposes increasing measurement");

    // Back-calc mode should still keep output within limits.
    pid.reset();
    pid.setAntiWindupMode(AntiWindupMode::BACK_CALC);
    const double out_backcalc = pid.update(0.1, 100.0, 0.0);
    AeroCore::Tests::expectTrue(out_backcalc <= 1.0 && out_backcalc >= -1.0,
                                "back-calc output obeys limits");

    AeroCore::Tests::expectThrows(
        [&]() { pid.setOutputLimits(1.0, 1.0); },
        "invalid output limits throw");
    AeroCore::Tests::expectThrows(
        [&]() { pid.setIntegralMax(-1.0); },
        "negative integral max throws");

    return AeroCore::Tests::finish();
}
