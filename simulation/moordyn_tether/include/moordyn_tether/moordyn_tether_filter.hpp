#ifndef MOORDYN_TETHER__MOORDYN_TETHER_FILTER_HPP_
#define MOORDYN_TETHER__MOORDYN_TETHER_FILTER_HPP_

#include <vector>

namespace moordyn_tether
{

/// @brief Second-order Butterworth low-pass filter for force vectors.
///
/// Removes the residual axial vibration of the cable solver (a few hertz) while delaying the slow force (waves,
/// manoeuvres) no more than a first-order filter of the same smoothing would. It is integrated in continuous time
/// with the elapsed step, since the steps of the node follow the readings of the line ends and are not uniform.
class ForceFilter
{
public:
    ForceFilter() = default;

    /// @brief Initialize the filter with its cutoff frequency.
    /// @param cutoff_hz Cutoff frequency [Hz]; zero or negative disables the filter.
    void init(double cutoff_hz);

    /// @brief Clear internal states: the filter starts again from zero force.
    void reset();

    /// @brief Apply the filter to an array of forces over a step.
    /// @param in_forces Raw forces from MoorDyn.
    /// @param dt Duration of the step [s].
    /// @param out_forces Output filtered forces.
    void apply(const std::vector<double> & in_forces, double dt, std::vector<double> & out_forces);

private:
    double omega_{0.0};                ///< cutoff angular frequency [rad/s]
    bool   has_initial_state_{false};
    std::vector<double> state_;        ///< filtered forces
    std::vector<double> rate_;         ///< their rates of change
};

}  // namespace moordyn_tether

#endif  // MOORDYN_TETHER__MOORDYN_TETHER_FILTER_HPP_
