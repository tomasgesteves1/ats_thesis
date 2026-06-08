#ifndef ATS_TETHER_MOORDYN_SYSTEM_HPP
#define ATS_TETHER_MOORDYN_SYSTEM_HPP

#include <string>
#include <vector>
#include "moordyn/MoorDyn2.h"
#include <rclcpp/rclcpp.hpp>

namespace ats_tether
{

    class TetherMoorDynSystem
    {
    public:
        TetherMoorDynSystem(const std::string &package_share_dir, rclcpp::Logger logger);
        ~TetherMoorDynSystem();

        // Inicializa a simulação dadas as posições iniciais
        bool initialize(const std::vector<double> &initial_positions);

        // Avança a simulação no tempo
        bool step(const std::vector<double> &current_positions, double dt, std::vector<double> &out_forces, std::vector<std::vector<double>> &out_cable_nodes);

    private:
        MoorDyn system_;
        rclcpp::Logger logger_;
    };

} // namespace ats_tether

#endif // ATS_TETHER_MOORDYN_SYSTEM_HPP
