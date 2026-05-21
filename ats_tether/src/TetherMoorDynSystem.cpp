#include "ats_tether/TetherMoorDynSystem.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace ats_tether
{

    TetherMoorDynSystem::TetherMoorDynSystem(const std::string &package_share_dir, rclcpp::Logger logger)
        : logger_(logger)
    {
        std::string lines_file = package_share_dir + "/config/lines.txt";

        if (!std::filesystem::exists(lines_file))
        {
            RCLCPP_ERROR(logger_, "Ficheiro MoorDyn não encontrado: %s", lines_file.c_str());
            return;
        }

        system_ = MoorDyn_Create(lines_file.c_str());
        if (!system_)
        {
            RCLCPP_ERROR(logger_, "Falha ao criar o sistema MoorDyn a partir de: %s", lines_file.c_str());
        }
        else
        {
            RCLCPP_INFO(logger_, "Sistema MoorDyn criado com sucesso: %s", lines_file.c_str());
        }
    }

    TetherMoorDynSystem::~TetherMoorDynSystem()
    {
        if (system_)
        {
            MoorDyn_Close(system_);
        }
    }

    bool TetherMoorDynSystem::initialize(const std::vector<double> &initial_positions)
    {
        if (!system_)
            return false;

        // No MoorDyn v2, precisamos de fornecer posições e velocidades iniciais
        // Como estamos a começar, as velocidades são zero.
        std::vector<double> initial_velocities(initial_positions.size(), 0.0);

        int result = MoorDyn_Init(system_, initial_positions.data(), initial_velocities.data());

        if (result != MOORDYN_SUCCESS)
        {
            RCLCPP_ERROR(logger_, "Falha na inicialização do MoorDyn! Código: %d", result);
            return false;
        }

        RCLCPP_INFO(logger_, "MoorDyn inicializado corretamente.");
        return true;
    }

    bool TetherMoorDynSystem::step(const std::vector<double> &current_positions, double dt, std::vector<double> &out_forces)
    {
        if (!system_)
            return false;

        // Redimensionar o vetor de forças de saída se necessário
        // Cada ponto Coupled devolve 3 forças (Fx, Fy, Fz)
        out_forces.resize(current_positions.size());

        // No MoorDyn v2, as velocidades também são importantes para o amortecimento.
        // Para simplificar agora, vamos assumir velocidades zero,
        // mas o ideal será passarmos as velocidades reais do drone/barco no futuro.
        std::vector<double> current_velocities(current_positions.size(), 0.0);

        double time = 0.0; // O MoorDyn gere o tempo internamente a partir do dt
        double step_dt = dt;

        int result = MoorDyn_Step(system_, current_positions.data(), current_velocities.data(), out_forces.data(), &time,
                                  &step_dt);

        if (result != MOORDYN_SUCCESS)
        {
            RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 1000, "Erro no MoorDyn_Step! Código: %d",
                                  result);
            return false;
        }

        return true;
    }

} // namespace ats_tether
