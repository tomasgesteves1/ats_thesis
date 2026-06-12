#include "moordyn_tether/moordyn_tether_winch.hpp"

#include <algorithm>

namespace moordyn_tether
{

void VirtualWinch::setConfig(const WinchConfig & config)
{
    cfg_ = config;
}

void VirtualWinch::update(MoorDynLine line, const std::array<BodyState, 2> & body_states, double dt, const std::vector<double>& forces)
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
        double current_length = 0.0;
        MoorDyn_GetLineUnstretchedLength(line, &current_length);

        if (forces.size() >= 3)
        {
            double fz = forces[2]; // Z-force exerted BY the mooring ON the boat
            
            double error = fz - cfg_.target_tension;
            
            double d_error = 0.0;
            if (has_last_error_ && dt > 0.0)
            {
                d_error = (error - last_error_) / dt;
            }
            last_error_ = error;
            has_last_error_ = true;

            // PD-control
            double delta_length = (cfg_.kp_tension * error + cfg_.kd_tension * d_error) * dt;
            
            double max_delta = cfg_.winch_speed_limit * dt;
            delta_length = std::clamp(delta_length, -max_delta, max_delta);

            desired = current_length + delta_length;
        }
        else
        {
            desired = current_length;
        }
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
