#ifndef MOORDYN_TETHER__MOORDYN_TETHER_FILTER_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_FILTER_HPP_

#include <vector>

namespace moordyn_tether
{

/// @brief Exponential Moving Average (EMA) / Low-Pass filter for force vectors.
///
/// Reduces high-frequency noise and spikes from the physics solver.
class ForceFilter
{
public:
    ForceFilter() = default;

    /// @brief Initialize the filter with an alpha parameter.
    /// @param alpha Smoothing factor [0.0, 1.0]. 
    ///              1.0 = no filtering (instant update).
    ///              0.01 = heavy smoothing.
    void init(double alpha);

    /// @brief Clear internal states (e.g. after a large jump or reset).
    void reset();

    /// @brief Apply the filter to an array of forces.
    /// @param in_forces Raw forces from MoorDyn.
    /// @param out_forces Output filtered forces.
    void apply(const std::vector<double> & in_forces, std::vector<double> & out_forces);

private:
    double alpha_{1.0};
    bool   has_initial_state_{false};
    std::vector<double> state_;
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_FILTER_HPP_
