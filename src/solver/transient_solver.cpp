#include "solver/transient_solver.hpp"
#include <iostream>
#include <iomanip>

namespace asic {

void TransientSolver::stamp_capacitors(const Circuit& circuit, MnaMatrix& target_matrix, const Eigen::VectorXd& v_old, double dt) const {
    for (const auto& comp : circuit.get_components()) {
        if (auto c = dynamic_cast<const Passives::Capacitor*>(comp.get())) {
            size_t node_a = c->nodes[0];
            size_t node_b = c->nodes[1];
            
            double g_eq = c->capacitance / dt;
            
            double v_a_old = (node_a > 0) ? v_old(node_a - 1) : 0.0;
            double v_b_old = (node_b > 0) ? v_old(node_b - 1) : 0.0;
            
            // FIX: Invert the Norton equivalent current direction!
            // stamp_current_source assumes current leaves the node, 
            // so we must negate the historical injection.
            double i_eq = -g_eq * (v_a_old - v_b_old);

            target_matrix.stamp_conductance(node_a, node_b, g_eq);
            target_matrix.stamp_current_source(node_a, node_b, i_eq);
        }
    }
}

bool TransientSolver::solve(const Circuit& circuit, MnaMatrix& state, std::ostream& csv_out) {
    size_t num_nodes = circuit.num_nodes();
    
    // 1. PRE-SETTLING: Run DC Op-Point to establish starting equilibrium
    // This prevents the "shock" of starting from 0V at t=0
    std::cout << "[libasic] Calculating DC Operating Point...\n";
    if (!nr_engine.solve_dc(circuit, state)) {
        std::cerr << "[libasic] Failed to find DC Operating Point. Circuit is unstable.\n";
        return false;
    }

    // --- CRITICAL FIX: Synchronize v_old with the DC solution ---
    Eigen::VectorXd v_old = Eigen::VectorXd::Zero(num_nodes);
    for(size_t i = 1; i <= num_nodes; ++i) {
        v_old(i-1) = state.get_node_voltage(i);
    }

    // Write CSV Header
    csv_out << "Time(s)";
    for (size_t i = 1; i <= num_nodes; ++i) csv_out << ",V_" << i;
    csv_out << "\n";

    // The Time Loop
    double damping = 0.5; // Damping factor for transient stability

    for (double t = 0; t <= t_stop; t += t_step) {
        csv_out << std::scientific << std::setprecision(6) << t;
        for (size_t i = 1; i <= num_nodes; ++i) {
            csv_out << "," << state.get_node_voltage(i);
        }
        csv_out << "\n";

        bool converged = false;
        Eigen::VectorXd v_prev_iter = Eigen::VectorXd::Zero(num_nodes);

        for (size_t iter = 0; iter < 100; ++iter) {
            MnaMatrix iter_matrix(circuit);
            iter_matrix.stamp_static_elements(circuit, t); 
            stamp_capacitors(circuit, iter_matrix, v_old, t_step);
            nr_engine.stamp_nonlinear_devices(circuit, state, iter_matrix);

            if (!iter_matrix.solve()) return false;

            // Apply Voltage Damping (The "Safety Rail")
            for (size_t i = 1; i <= num_nodes; ++i) {
                double delta = iter_matrix.get_node_voltage(i) - state.get_node_voltage(i);
                double damped_v = state.get_node_voltage(i) + (damping * delta);
                iter_matrix.x(i - 1) = damped_v; 
            }

            // Check convergence
            bool done = true;
            for (size_t i = 1; i <= num_nodes; ++i) {
                if (std::abs(iter_matrix.get_node_voltage(i) - state.get_node_voltage(i)) > 1e-4) {
                    done = false; break;
                }
            }
            
            state = iter_matrix;
            if (done) { converged = true; break; }
        }

        if (!converged) {
            std::cerr << "[libasic] Transient integration failed at t = " << t << "s\n";
            return false;
        }

        // Update historical memory
        for (size_t i = 1; i <= num_nodes; ++i) v_old(i - 1) = state.get_node_voltage(i);
    }

    return true;
}

} // namespace asic