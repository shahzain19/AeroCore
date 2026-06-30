#pragma once

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace AeroCore::Tests {

inline int tests_failed = 0;
inline int tests_run = 0;

inline void expectTrue(bool condition, const std::string& message) {
    ++tests_run;
    if (!condition) {
        ++tests_failed;
        std::cerr << "[FAIL] " << message << "\n";
    }
}

inline void expectNear(double actual, double expected, double tolerance, const std::string& message) {
    ++tests_run;
    if (std::fabs(actual - expected) > tolerance) {
        ++tests_failed;
        std::cerr << "[FAIL] " << message << " expected=" << expected
                  << " actual=" << actual << " tol=" << tolerance << "\n";
    }
}

template <typename Func>
inline void expectThrows(Func&& f, const std::string& message) {
    ++tests_run;
    try {
        f();
        ++tests_failed;
        std::cerr << "[FAIL] " << message << " (no exception)\n";
    } catch (...) {
    }
}

inline int finish() {
    if (tests_failed == 0) {
        std::cout << "[PASS] " << tests_run << " assertions\n";
        return 0;
    }
    std::cerr << "[FAIL] " << tests_failed << " of " << tests_run << " assertions failed\n";
    return 1;
}

} // namespace AeroCore::Tests
