#include "ats_tether/TetherMoorDynSystem.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace ats_tether
{

    TetherMoorDynSystem::TetherMoorDynSystem(const std::string &package_share_dir, rclcpp::Logger logger)
        : system_(nullptr), logger_(logger)
    {
        std::string lines_file = package_share_dir + "/config/lines.txt";

        RCLCPP_INFO(logger_, "A carregar MoorDyn do ficheiro: %s", lines_file.c_str());

        if (!std::filesystem::exists(lines_file))
        {
            RCLCPP_ERROR(logger_, "Ficheiro MoorDyn nao encontrado!");
            return;
        }

        system_ = MoorDyn_Create(lines_file.c_str());
        if (!system_)
        {
            RCLCPP_ERROR(logger_, "MoorDyn_Create FALHOU! Verifique se a biblioteca MoorDyn esta bem instalada.");
        }
        else
        {
            unsigned int n_dof = 0;
            MoorDyn_NCoupledDOF(system_, &n_dof);
            RCLCPP_INFO(logger_, "MoorDyn_Create SUCESSO. Sistema espera %u graus de liberdade (DOF).", n_dof);
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
        if (!system_) {
            RCLCPP_ERROR(logger_, "Impossivel inicializar: o sistema MoorDyn e NULL.");
            return false;
        }

        unsigned int n_dof = 0;
        MoorDyn_NCoupledDOF(system_, &n_dof);
        if (initial_positions.size() != n_dof) {
            RCLCPP_ERROR(logger_, "Tamanho do vetor ( %ld ) nao condiz com o esperado pelo MoorDyn ( %u ).", 
                         initial_positions.size(), n_dof);
            return false;
        }

        std::vector<double> initial_velocities(initial_positions.size(), 0.0);
        int result = MoorDyn_Init(system_, initial_positions.data(), initial_velocities.data());

        if (result != MOORDYN_SUCCESS)
        {
            RCLCPP_ERROR(logger_, "MoorDyn_Init FALHOU com codigo %d. Geometria inicial invalida?", result);
            return false;
        }

        RCLCPP_INFO(logger_, "MoorDyn inicializado corretamente!");
        return true;
    }

    bool TetherMoorDynSystem::step(const std::vector<double> &current_positions, double dt, std::vector<double> &out_forces, std::vector<std::vector<double>> &out_cable_nodes)
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

        // --- Extrair geometria do cabo (Linha 1) ---
        out_cable_nodes.clear();
        MoorDynLine line = MoorDyn_GetLine(system_, 1); // 1-indexed in MoorDyn
        if (line) {
            unsigned int n_nodes = 0;
            MoorDyn_GetLineNumberNodes(line, &n_nodes);
            for (unsigned int i = 0; i < n_nodes; ++i) {
                double pos[3];
                MoorDyn_GetLineNodePos(line, i, pos);
                out_cable_nodes.push_back({pos[0], pos[1], pos[2]});
            }
        }

        return true;
    }

} // namespace ats_tether
