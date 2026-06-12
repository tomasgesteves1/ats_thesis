#include "moordyn_tether/moordyn_tether_filter.hpp"

namespace moordyn_tether
{

void ForceFilter::init(double alpha)
{
    // Clamp alpha between 0.001 and 1.0 to prevent instability
    alpha_ = alpha;
    if (alpha_ > 1.0) alpha_ = 1.0;
    if (alpha_ < 0.0) alpha_ = 0.0;
    
    reset();
}

void ForceFilter::reset()
{
    has_initial_state_ = false;
    state_.clear();
}

void ForceFilter::apply(const std::vector<double> & in_forces, std::vector<double> & out_forces)
{
    if (in_forces.empty())
    {
        out_forces.clear();
        return;
    }

    if (!has_initial_state_ || state_.size() != in_forces.size())
    {
        // First run or dimension mismatch, initialize state directly to input
        state_ = in_forces;
        has_initial_state_ = true;
    }
    else
    {
        // Apply EMA: y_n = alpha * x_n + (1 - alpha) * y_{n-1}
        for (size_t i = 0; i < in_forces.size(); ++i)
        {
            state_[i] = alpha_ * in_forces[i] + (1.0 - alpha_) * state_[i];
        }
    }

    out_forces = state_;
}

}  // namespace moordyn_tether
