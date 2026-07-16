#include "asic/transient_solver.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace asic {

void TransientSolver::stamp_capacitors(const Circuit& circuit, MnaMatrix& target_matrix, const Eigen::VectorXd& v_old, double dt) const {
    for (const auto& comp : circuit.get_components()) {
        if (auto c = dynamic_cast<const Passives::Capacitor*>(comp.get())) {
            size_t node_a = c->nodes[0];
            size_t node_b = c->nodes[1];
            
            double g_eq = c->capacitance / dt;
            double v_a_old = (node_a > 0) ? v_old(node_a - 1) : 0.0;
            double v_b_old = (node_b > 0) ? v_old(node_b - 1) : 0.0;
            
            double i_eq = -g_eq * (v_a_old - v_b_old);

            target_matrix.stamp_conductance(node_a, node_b, g_eq);
            target_matrix.stamp_current_source(node_a, node_b, i_eq);
        }
    }
}

bool TransientSolver::solve(const Circuit& circuit, MnaMatrix& state, std::ostream& csv_out) {
    size_t num_nodes = circuit.num_nodes();
    
    std::cout << "[libasic] Calculating initial DC Operating Point...\n";
    if (!nr_engine.solve_dc(circuit, state)) {
        std::cerr << "[libasic] Failed to find initial DC equilibrium.\n";
        return false;
    }

    Eigen::VectorXd v_old = Eigen::VectorXd::Zero(num_nodes);
    for(size_t i = 1; i <= num_nodes; ++i) {
        v_old(i-1) = state.get_node_voltage(i);
    }

    csv_out << "Time(s)";
    for (size_t i = 1; i <= num_nodes; ++i) csv_out << ",V_" << i;
    csv_out << "\n";

    double t = 0.0;
    double current_dt = t_step;
    double damping = 0.3; // Gentle transient damping

    while (t < t_stop) {
        bool converged = false;
        MnaMatrix current_state = state; // Seed NR with the last known stable timestep

        for (size_t iter = 0; iter < 100; ++iter) {
            MnaMatrix next_state(circuit);
            next_state.stamp_static_elements(circuit, t + current_dt); 
            stamp_capacitors(circuit, next_state, v_old, current_dt);
            nr_engine.stamp_nonlinear_devices(circuit, current_state, next_state);

            if (!next_state.solve()) break; // Internal singular state triggers step shrinkage

            double max_delta = 0.0;
            for (size_t i = 0; i < num_nodes; ++i) {
                double raw_step = next_state.x(i) - current_state.x(i);
                double damped_step = std::clamp(raw_step * damping, -0.1, 0.1);
                double updated_val = std::clamp(current_state.x(i) + damped_step, -1.3, 1.3); 
                next_state.x(i) = updated_val; 
                max_delta = std::max(max_delta, std::abs(raw_step));
            }
            
            for (size_t i = num_nodes; i < next_state.x.size(); ++i) {
                double raw_step = next_state.x(i) - current_state.x(i);
                next_state.x(i) = current_state.x(i) + (raw_step * damping);
            }

            current_state = next_state;

            // Transients allow a slightly looser tolerance due to capacitor conductance sizing
            if (max_delta < 1e-4) {
                converged = true; 
                break; 
            }
        }

        if (converged) {
            t += current_dt;
            state = current_state;
            for (size_t i = 1; i <= num_nodes; ++i) v_old(i - 1) = state.get_node_voltage(i);
            
            for (const auto& comp : circuit.get_components()) {
                if (auto mem = dynamic_cast<Passives::Memristor*>(comp.get())) {
                    double v_pos = (mem->nodes[0] > 0) ? state.get_node_voltage(mem->nodes[0]) : 0.0;
                    double v_neg = (mem->nodes[1] > 0) ? state.get_node_voltage(mem->nodes[1]) : 0.0;
                    mem->step_time(v_pos - v_neg, current_dt);
                }
            }
            
            csv_out << std::scientific << std::setprecision(6) << t;
            for (size_t i = 1; i <= num_nodes; ++i) csv_out << "," << state.get_node_voltage(i);
            csv_out << "\n";

            // Ramp speed back up during steady states
            if (current_dt < t_step) {
                current_dt *= 1.5;
                if (current_dt > t_step) current_dt = t_step;
            }
        } else {
            // Adaptive timestep shrinkage
            current_dt /= 2.0;
            if (current_dt < 1e-12) {
                std::cerr << "[libasic] Transient Failure: Timestep collapsed at t = " << t << "s\n";
                return false;
            }
        }
    }

    return true;
}

} // namespace asic