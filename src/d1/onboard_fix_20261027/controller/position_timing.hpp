#pragma once
#include <algorithm>
#include <cmath>

// Preserve the recorded controller's nonzero-distance timing formula and caps.
// These are implementation constants, not manufacturer-rated capabilities.
namespace d1_timing_fix {
inline bool delay_ms(double current, double target, int requested_ms, int& result) {
    if (!std::isfinite(current) || !std::isfinite(target) || requested_ms < 0)
        return false;
    const double delta = std::abs(target-current);
    const double requested = std::max(1, requested_ms);
    // A same-position request must never compute omega/alpha == 0/0.
    if (delta == 0.) {
        result = static_cast<int>(requested);
        return result <= 65535;
    }
    const double omega = std::min(delta/requested*1000., 28.64789*4.);
    const double alpha = std::min(omega/requested*1000., 286.4789/4.);
    // Rationalized form of 1800*(sqrt(omega²+2*delta*alpha)-omega)/alpha.
    const double duration = 3600.*delta/(std::sqrt(omega*omega+2.*delta*alpha)+omega);
    if (!std::isfinite(duration) || duration > 65535. || duration < 0.)
        return false;
    result = std::max(1, static_cast<int>(duration));
    return true;
}
}
