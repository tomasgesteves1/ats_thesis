#include "moordyn_tether/moordyn_tether_winch.hpp"

#include <algorithm>

namespace moordyn_tether
{

void VirtualWinch::setConfig(const WinchConfig & config)
{
    cfg_ = config;
}

void VirtualWinch::update(MoorDynLine line, const std::array<BodyState, 2> & body_states, double dt)
{
    if (!cfg_.enabled || !line)
    {
        return;
    }

    double desired = 0.0;

    if (cfg_.mode == WinchMode::GEOMETRIC)
    {
        const double span = Kinematics::distance3(body_states[0].anchor_pos,
                                                  body_states[1].anchor_pos);
        desired = span * cfg_.slack_factor;
    }
    else if (cfg_.mode == WinchMode::TENSION)
    {
        double current_tension = 0.0;
        MoorDyn_GetLineFairTen(line, &current_tension);

        double error = current_tension - cfg_.target_tension;
        
        // P-control: if tension is too high (error > 0), we need to release cable (delta_length > 0)
        double delta_length = cfg_.kp_tension * error * dt;
        
        // Limit spooling speed
        double max_delta = cfg_.winch_speed_limit * dt;
        delta_length = std::clamp(delta_length, -max_delta, max_delta);

        double current_length = 0.0;
        MoorDyn_GetLineUnstretchedLength(line, &current_length);
        
        desired = current_length + delta_length;
    }

    if (desired < cfg_.min_length)
    {
        desired = cfg_.min_length;
    }
    if (desired > cfg_.max_length)
    {
        desired = cfg_.max_length;
    }

    MoorDyn_SetLineUnstretchedLength(line, desired);
}

}  // namespace moordyn_tether
