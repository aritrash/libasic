#include "solver/newton_raphson.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace asic {

bool NewtonRaphson::check_convergence(const Eigen::VectorXd& old_v, const MnaMatrix& matrix, size_t num_nodes) const {
    for (size_t i = 1; i <= num_nodes; ++i) {
        double diff = std::abs(matrix.get_node_voltage(i) - old_v(i - 1));
        if (diff > tolerance) {
            return false;
        }
    }
    return true;
}

void NewtonRaphson::stamp_nonlinear_devices(const Circuit& circuit, const MnaMatrix& current_state, MnaMatrix& target_matrix) {
    for (const auto& comp : circuit.get_components()) {
        
        // ---------------------------------------------------------
        // 1. MOSFET Physics Engine (Planar Velocity Saturated)
        // ---------------------------------------------------------
        if (auto tx = dynamic_cast<const fet::mos*>(comp.get())) {
            size_t d = tx->nodes[0], g = tx->nodes[1], s = tx->nodes[2], b = tx->nodes[3];
            double v_d = current_state.get_node_voltage(d);
            double v_g = current_state.get_node_voltage(g);
            double v_s = current_state.get_node_voltage(s);
            double v_b = current_state.get_node_voltage(b);

            double v_gs = v_g - v_s, v_ds = v_d - v_s, v_bs = v_b - v_s;
            double sign = (tx->polarity == Polarity::N_TYPE) ? 1.0 : -1.0;

            v_gs *= sign; v_ds *= sign; v_bs *= sign;

            // Prevent forward bias blowup during early Newton iterations
            if (v_bs > 0.3) v_bs = 0.3; 

            // Body Effect Calculation
            double vth_dynamic = tx->v_threshold + tx->gamma * (std::sqrt(std::abs(tx->phi - v_bs)) - std::sqrt(tx->phi));
            
            double mu_0 = 0.05, cox = 1.5e-2, vsat = 1e5, lambda = 0.05, vt = 0.0259;
            double beta = mu_0 * cox * (tx->width / tx->length);
            double i_ds = 0.0, g_m = 0.0, g_ds = 0.0;

            if (v_gs <= vth_dynamic) {
                // Subthreshold 
                double i0 = beta * vt * vt, sub_slope = 1.3;
                double exp_factor = std::exp((v_gs - vth_dynamic) / (sub_slope * vt));
                double volt_drop = 1.0 - std::exp(-v_ds / vt);
                i_ds = i0 * exp_factor * volt_drop;
                g_m  = (i0 / (sub_slope * vt)) * exp_factor * volt_drop;
                g_ds = (i0 / vt) * exp_factor * std::exp(-v_ds / vt);
            } else {
                // Strong Inversion
                double esat_l = vsat * tx->length / mu_0;
                double v_gate_overdrive = v_gs - vth_dynamic;
                double v_dsat = v_gate_overdrive * esat_l / (v_gate_overdrive + esat_l);

                if (v_ds < v_dsat) {
                    double den = 1.0 + (v_ds / esat_l);
                    double core_current = beta * (v_gate_overdrive * v_ds - 0.5 * v_ds * v_ds) / den;
                    i_ds = core_current * (1.0 + lambda * v_ds);
                    g_m  = (beta * v_ds / den) * (1.0 + lambda * v_ds);
                    g_ds = (beta * (v_gate_overdrive - v_ds) / den - (beta * (v_gate_overdrive * v_ds - 0.5 * v_ds * v_ds) / (esat_l * den * den))) * (1.0 + lambda * v_ds) + core_current * lambda;
                } else {
                    double den = 1.0 + (v_dsat / esat_l);
                    double core_current = beta * (v_gate_overdrive * v_dsat - 0.5 * v_dsat * v_dsat) / den;
                    i_ds = core_current * (1.0 + lambda * v_ds);
                    g_m  = (beta * v_dsat / den) * (1.0 + lambda * v_ds);
                    g_ds = core_current * lambda;
                }
            }

            // Bulk Transconductance (g_mb)
            double g_mb = (tx->gamma * g_m) / (2.0 * std::sqrt(std::max(tx->phi - v_bs, 0.001)));

            i_ds *= sign; g_m *= sign; g_mb *= sign;
            
            // Equivalent Norton Current
            double i_eq = i_ds - (g_m * (v_g - v_s)) - (g_ds * (v_d - v_s)) - (g_mb * (v_b - v_s));

            // Stamp Conductance (g_ds) and Equivalent Source
            target_matrix.stamp_conductance(d, s, g_ds);
            target_matrix.stamp_current_source(d, s, i_eq);

            // Stamp g_m (Gate Control)
            if (d > 0 && g > 0) target_matrix.stamp_conductance_direct(d - 1, g - 1, g_m);
            if (d > 0 && s > 0) target_matrix.stamp_conductance_direct(d - 1, s - 1, -g_m);
            if (s > 0 && g > 0) target_matrix.stamp_conductance_direct(s - 1, g - 1, -g_m);
            if (s > 0 && s > 0) target_matrix.stamp_conductance_direct(s - 1, s - 1, g_m);

            // Stamp g_mb (Bulk Control)
            if (d > 0 && b > 0) target_matrix.stamp_conductance_direct(d - 1, b - 1, g_mb);
            if (d > 0 && s > 0) target_matrix.stamp_conductance_direct(d - 1, s - 1, -g_mb);
            if (s > 0 && b > 0) target_matrix.stamp_conductance_direct(s - 1, b - 1, -g_mb);
            if (s > 0 && s > 0) target_matrix.stamp_conductance_direct(s - 1, s - 1, g_mb);
        }
        
        // ---------------------------------------------------------
        // 2. BJT Physics Engine (Ebers-Moll / Gummel-Poon base)
        // ---------------------------------------------------------
        else if (dynamic_cast<const bjt::npn*>(comp.get()) || dynamic_cast<const bjt::pnp*>(comp.get())) {
            
            bool is_npn = dynamic_cast<const bjt::npn*>(comp.get()) != nullptr;
            
            size_t c = comp->nodes[0], b = comp->nodes[1], e = comp->nodes[2];
            double v_c = current_state.get_node_voltage(c);
            double v_b = current_state.get_node_voltage(b);
            double v_e = current_state.get_node_voltage(e);

            double is_val = is_npn ? dynamic_cast<const bjt::npn*>(comp.get())->is : dynamic_cast<const bjt::pnp*>(comp.get())->is;
            double bf     = is_npn ? dynamic_cast<const bjt::npn*>(comp.get())->bf : dynamic_cast<const bjt::pnp*>(comp.get())->bf;
            double br     = is_npn ? dynamic_cast<const bjt::npn*>(comp.get())->br : dynamic_cast<const bjt::pnp*>(comp.get())->br;

            double vt = 0.0259; // Thermal Voltage
            double v_be = v_b - v_e;
            double v_bc = v_b - v_c;
            
            double sign = is_npn ? 1.0 : -1.0;
            v_be *= sign;
            v_bc *= sign;

            // Clamp voltages to prevent exp() overflow during initial large step guesses
            v_be = std::min(v_be, 0.8);
            v_bc = std::min(v_bc, 0.8);

            // Transport Model Currents
            double I_cc = is_val * (std::exp(v_be / vt) - 1.0);
            double I_ec = is_val * (std::exp(v_bc / vt) - 1.0);
            double I_ct = I_cc - I_ec;

            // Dynamic Small-Signal Conductances (Jacobian entries)
            double g_pi = (is_val / (bf * vt)) * std::exp(v_be / vt); // Base-Emitter
            double g_mu = (is_val / (br * vt)) * std::exp(v_bc / vt); // Base-Collector
            double g_mf = (is_val / vt) * std::exp(v_be / vt);        // Forward Transconductance
            double g_mr = (is_val / vt) * std::exp(v_bc / vt);        // Reverse Transconductance

            // Norton Equivalent Currents for iterating
            double i_eq_be = (I_cc / bf) - g_pi * v_be;
            double i_eq_bc = (I_ec / br) - g_mu * v_bc;
            double i_eq_ct = I_ct - g_mf * v_be + g_mr * v_bc;

            i_eq_be *= sign; i_eq_bc *= sign; i_eq_ct *= sign;

            // Stamp Passive Conductances
            target_matrix.stamp_conductance(b, e, g_pi);
            target_matrix.stamp_conductance(b, c, g_mu);

            // Stamp Current Sources
            target_matrix.stamp_current_source(b, e, i_eq_be);
            target_matrix.stamp_current_source(b, c, i_eq_bc);
            target_matrix.stamp_current_source(c, e, i_eq_ct);

            // Stamp Forward VCCS (gm_f * V_be) dependent on V_be controlling C-E
            if (c > 0 && b > 0) target_matrix.stamp_conductance_direct(c - 1, b - 1, g_mf);
            if (c > 0 && e > 0) target_matrix.stamp_conductance_direct(c - 1, e - 1, -g_mf);
            if (e > 0 && b > 0) target_matrix.stamp_conductance_direct(e - 1, b - 1, -g_mf);
            if (e > 0 && e > 0) target_matrix.stamp_conductance_direct(e - 1, e - 1, g_mf);

            // Stamp Reverse VCCS (gm_r * V_bc) dependent on V_bc controlling E-C
            if (e > 0 && b > 0) target_matrix.stamp_conductance_direct(e - 1, b - 1, g_mr);
            if (e > 0 && c > 0) target_matrix.stamp_conductance_direct(e - 1, c - 1, -g_mr);
            if (c > 0 && b > 0) target_matrix.stamp_conductance_direct(c - 1, b - 1, -g_mr);
            if (c > 0 && c > 0) target_matrix.stamp_conductance_direct(c - 1, c - 1, g_mr);
        }
    }
}

bool NewtonRaphson::solve_dc(const Circuit& circuit, MnaMatrix& matrix) {
    size_t num_nodes = circuit.num_nodes();
    Eigen::VectorXd old_voltages = Eigen::VectorXd::Zero(num_nodes);

    for (size_t iter = 0; iter < max_iterations; ++iter) {
        for (size_t i = 1; i <= num_nodes; ++i) {
            old_voltages(i - 1) = matrix.get_node_voltage(i);
        }

        MnaMatrix iter_matrix(circuit);
        iter_matrix.stamp_static_elements(circuit);
        stamp_nonlinear_devices(circuit, matrix, iter_matrix);

        if (!iter_matrix.solve()) {
            std::cerr << "[libasic] Newton-Raphson failed: Matrix system structure singular at iteration " << iter << "\n";
            return false;
        }

        if (check_convergence(old_voltages, iter_matrix, num_nodes)) {
            matrix = iter_matrix;
            return true;
        }

        matrix = iter_matrix;
    }

    std::cerr << "[libasic] Warning: Newton-Raphson did not converge within " << max_iterations << " iterations.\n";
    return false;
}

} // namespace asic