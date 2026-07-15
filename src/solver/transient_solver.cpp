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
    Eigen::VectorXd v_old = Eigen::VectorXd::Zero(num_nodes);

    // Write CSV Header
    csv_out << "Time(s)";
    for (size_t i = 1; i <= num_nodes; ++i) csv_out << ",V_" << i;
    csv_out << "\n";

    // Establish Initial Conditions (t = 0)
    // For a real run, this would be a pure DC Op-Point solve to settle the circuit first.
    v_old.setZero();
    for (const auto& comp : circuit.get_components()) {
        if (auto c = dynamic_cast<const Passives::Capacitor*>(comp.get())) {
            if (c->nodes[0] > 0) v_old(c->nodes[0] - 1) = c->initial_voltage;
        }
    }

    // The Time Loop
    for (double t = 0; t <= t_stop; t += t_step) {
        
        // Log current state to CSV
        csv_out << std::scientific << std::setprecision(6) << t;
        for (size_t i = 1; i <= num_nodes; ++i) {
            csv_out << "," << state.get_node_voltage(i);
        }
        csv_out << "\n";

        // To cleanly integrate the transient capacitor stamps into the NR loop, 
        // we override the static matrix generation for this specific time step.
        // *Note: In a full architecture, we'd pass a callback to NR, but for clarity 
        // we apply the transient stamps manually during the NR loop's static phase.*
        
        // Custom NR loop for Transient Step
        bool converged = false;
        for (size_t iter = 0; iter < 100; ++iter) {
            MnaMatrix iter_matrix(circuit);
            iter_matrix.stamp_static_elements(circuit);
            
            // INJECT TIME MEMORY: Stamp capacitor companion models
            stamp_capacitors(circuit, iter_matrix, v_old, t_step);
            
            // INJECT PHYSICS: Stamp non-linear transistors
            nr_engine.stamp_nonlinear_devices(circuit, state, iter_matrix);

            if (!iter_matrix.solve()) return false;

            // Check convergence
            bool done = true;
            for (size_t i = 1; i <= num_nodes; ++i) {
                if (std::abs(iter_matrix.get_node_voltage(i) - state.get_node_voltage(i)) > 1e-6) {
                    done = false; break;
                }
            }
            
            state = iter_matrix;
            if (done) { converged = true; break; }
        }

        if (!converged) {
            std::cerr << "[libasic] Transient integration failed to converge at t = " << t << "s\n";
            return false;
        }

        // Update historical memory for the next tick
        for (size_t i = 1; i <= num_nodes; ++i) {
            v_old(i - 1) = state.get_node_voltage(i);
        }
    }

    return true;
}

} // namespace asic