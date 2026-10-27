#include "moordyn_tether/moordyn_tether_filter.hpp"

#include <algorithm>
#include <cmath>

namespace moordyn_tether
{

namespace
{
constexpr double kDamping = 0.70710678118654752;   // Butterworth: zeta = 1 / sqrt(2)
constexpr double kMaxSubstep = 0.005;              // s, integration substep (omega * h ~ 0.02 at 0.7 Hz)
}  // namespace

void ForceFilter::init(double cutoff_hz)
{
    omega_ = cutoff_hz > 0.0 ? 2.0 * M_PI * cutoff_hz : 0.0;
    reset();
}

void ForceFilter::reset()
{
    has_initial_state_ = false;
    state_.clear();
    rate_.clear();
}

void ForceFilter::apply(const std::vector<double> & in_forces, double dt, std::vector<double> & out_forces)
{
    if (in_forces.empty())
    {
        out_forces.clear();
        return;
    }
    if (omega_ <= 0.0)
    {
        out_forces = in_forces;
        return;
    }

    if (!has_initial_state_ || state_.size() != in_forces.size())
    {
        // First run: the filter starts from zero, the tension of the straight model when MoorDyn takes over, so the
        // transient of a line initialised at rest while its ends already move reaches the vehicles filtered
        state_.assign(in_forces.size(), 0.0);
        rate_.assign(in_forces.size(), 0.0);
        has_initial_state_ = true;
    }

    // y'' + 2 zeta w y' + w^2 y = w^2 u, with the input held over the step (semi-implicit Euler substeps)
    const int n = std::max(1, static_cast<int>(std::ceil(dt / kMaxSubstep)));
    const double h = dt / n;
    for (size_t i = 0; i < in_forces.size(); ++i)
    {
        for (int k = 0; k < n; ++k)
        {
            rate_[i] += h * (omega_ * omega_ * (in_forces[i] - state_[i]) - 2.0 * kDamping * omega_ * rate_[i]);
            state_[i] += h * rate_[i];
        }
    }

    out_forces = state_;
}

}  // namespace moordyn_tether
