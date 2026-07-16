#include "asic/newton_raphson.hpp"
#include "asic/exceptions.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>

namespace asic {

bool NewtonRaphson::check_convergence(const Eigen::VectorXd& old_v, const MnaMatrix& matrix, size_t num_nodes) const {
    for (size_t i = 1; i <= num_nodes; ++i) {
        double diff = std::abs(matrix.get_node_voltage(i) - old_v(i - 1));
        if (diff > tolerance) return false;
    }
    return true;
}

void NewtonRaphson::stamp_single_mosfet(size_t d, size_t g, size_t s, size_t b, 
                                        Polarity polarity, double width, double length, 
                                        double v_threshold, double gamma, double phi, 
                                        const MnaMatrix& current_state, MnaMatrix& target_matrix) const {
    
    double v_d = current_state.get_node_voltage(d);
    double v_g = current_state.get_node_voltage(g);
    double v_s = current_state.get_node_voltage(s);
    double v_b = current_state.get_node_voltage(b);

    if ((polarity == Polarity::N_TYPE && v_d < v_s) || 
        (polarity == Polarity::P_TYPE && v_d > v_s)) {
        std::swap(d, s);
        std::swap(v_d, v_s);
    }

    double real_v_gs = v_g - v_s;
    double real_v_ds = v_d - v_s;
    double real_v_bs = v_b - v_s;

    double sign = (polarity == Polarity::N_TYPE) ? 1.0 : -1.0;

    double v_gs = real_v_gs * sign; 
    double v_ds = real_v_ds * sign; 
    double v_bs = real_v_bs * sign;

    v_ds = std::clamp(v_ds, -0.1, 3.0); 
    v_gs = std::clamp(v_gs, -1.0, 3.0);
    v_bs = std::clamp(v_bs, -1.0, 0.5);

    double vth_base = std::abs(v_threshold);
    double vth_dynamic = vth_base + gamma * (std::sqrt(std::abs(phi - v_bs)) - std::sqrt(phi));
    
    double mu_0 = 0.05, cox = 1.5e-2, vsat = 1e5, lambda = 0.05, vt = 0.0259;
    double beta = mu_0 * cox * (width / length);
    double i_ds = 0.0, g_m = 0.0, g_ds = 0.0;
    double esat_l = vsat * length / mu_0;

    if (v_gs <= vth_dynamic) {
        double i0 = beta * vt * vt, sub_slope = 1.3;
        double arg = std::clamp((v_gs - vth_dynamic) / (sub_slope * vt), -20.0, 20.0);
        double exp_factor = std::exp(arg);
        double v_ds_clamped = std::max(v_ds, 0.001); 
        double volt_drop = 1.0 - std::exp(-v_ds_clamped / vt);
        
        i_ds = i0 * exp_factor * volt_drop;
        g_m  = (i0 / (sub_slope * vt)) * exp_factor * volt_drop;
        g_ds = (i0 / vt) * exp_factor * std::exp(-v_ds_clamped / vt);
    } else {
        double v_gate_overdrive = v_gs - vth_dynamic;
        double den_dsat = std::max(v_gate_overdrive + esat_l, 0.001);
        double v_dsat = v_gate_overdrive * esat_l / den_dsat;

        if (v_ds < v_dsat) {
            double den = std::max(1.0 + (v_ds / esat_l), 0.001);
            double core_current = beta * (v_gate_overdrive * v_ds - 0.5 * v_ds * v_ds) / den;
            i_ds = core_current * (1.0 + lambda * v_ds);
            g_m  = (beta * v_ds / den) * (1.0 + lambda * v_ds);
            g_ds = (beta * (v_gate_overdrive - v_ds) / den - (beta * (v_gate_overdrive * v_ds - 0.5 * v_ds * v_ds) / (esat_l * den * den))) * (1.0 + lambda * v_ds) + core_current * lambda;
        } else {
            double den = std::max(1.0 + (v_dsat / esat_l), 0.001);
            double core_current = beta * (v_gate_overdrive * v_dsat - 0.5 * v_dsat * v_dsat) / den;
            i_ds = core_current * (1.0 + lambda * v_ds);
            g_m  = (beta * v_dsat / den) * (1.0 + lambda * v_ds);
            g_ds = core_current * lambda;
        }
    }

    double g_mb = (gamma * g_m) / (2.0 * std::sqrt(std::max(phi - v_bs, 0.001)));

    i_ds *= sign; 

    double i_eq = i_ds - (g_m * real_v_gs) - (g_ds * real_v_ds) - (g_mb * real_v_bs);

    i_eq = std::clamp(i_eq, -0.1, 0.1); 
    g_ds = std::clamp(g_ds, 1e-9, 10.0); 

    target_matrix.stamp_conductance(d, s, g_ds);
    target_matrix.stamp_current_source(d, s, i_eq);

    if (d > 0 && g > 0) target_matrix.stamp_conductance_direct(d - 1, g - 1, g_m);
    if (d > 0 && s > 0) target_matrix.stamp_conductance_direct(d - 1, s - 1, -g_m);
    if (s > 0 && g > 0) target_matrix.stamp_conductance_direct(s - 1, g - 1, -g_m);
    if (s > 0 && s > 0) target_matrix.stamp_conductance_direct(s - 1, s - 1, g_m);

    if (d > 0 && b > 0) target_matrix.stamp_conductance_direct(d - 1, b - 1, g_mb);
    if (d > 0 && s > 0) target_matrix.stamp_conductance_direct(d - 1, s - 1, -g_mb);
    if (s > 0 && b > 0) target_matrix.stamp_conductance_direct(s - 1, b - 1, -g_mb);
    if (s > 0 && s > 0) target_matrix.stamp_conductance_direct(s - 1, s - 1, g_mb);
}

void NewtonRaphson::stamp_nonlinear_devices(const Circuit& circuit, const MnaMatrix& current_state, MnaMatrix& target_matrix) {
    for (const auto& comp : circuit.get_components()) {        
        if (auto tx = dynamic_cast<const fet::mos*>(comp.get())) {
            stamp_single_mosfet(tx->nodes[0], tx->nodes[1], tx->nodes[2], tx->nodes[3],
                                tx->polarity, tx->width, tx->length, tx->v_threshold,
                                tx->gamma, tx->phi, current_state, target_matrix);
        }
        else if (auto tg = dynamic_cast<const fet::tgate*>(comp.get())) {
            size_t in = tg->nodes[0], out = tg->nodes[1];
            size_t ctrl_n = tg->nodes[2], ctrl_p = tg->nodes[3];
            size_t bulk_n = tg->nodes[4], bulk_p = tg->nodes[5];

            stamp_single_mosfet(out, ctrl_n, in, bulk_n, Polarity::N_TYPE, 
                                tg->width_n, tg->length, tg->v_threshold_n, tg->gamma, tg->phi, current_state, target_matrix);
            
            stamp_single_mosfet(out, ctrl_p, in, bulk_p, Polarity::P_TYPE, 
                                tg->width_p, tg->length, tg->v_threshold_p, tg->gamma, tg->phi, current_state, target_matrix);
        }
        else if (dynamic_cast<const bjt::npn*>(comp.get()) || dynamic_cast<const bjt::pnp*>(comp.get())) {
            bool is_npn = dynamic_cast<const bjt::npn*>(comp.get()) != nullptr;
            
            size_t c = comp->nodes[0], b = comp->nodes[1], e = comp->nodes[2];
            double v_c = current_state.get_node_voltage(c);
            double v_b = current_state.get_node_voltage(b);
            double v_e = current_state.get_node_voltage(e);

            double is_val = is_npn ? dynamic_cast<const bjt::npn*>(comp.get())->is : dynamic_cast<const bjt::pnp*>(comp.get())->is;
            double bf     = is_npn ? dynamic_cast<const bjt::npn*>(comp.get())->bf : dynamic_cast<const bjt::pnp*>(comp.get())->bf;
            double br     = is_npn ? dynamic_cast<const bjt::npn*>(comp.get())->br : dynamic_cast<const bjt::pnp*>(comp.get())->br;

            double vt = 0.0259; 
            double v_be = (v_b - v_e) * (is_npn ? 1.0 : -1.0);
            double v_bc = (v_b - v_c) * (is_npn ? 1.0 : -1.0);

            v_be = std::min(v_be, 0.8);
            v_bc = std::min(v_bc, 0.8);

            double I_cc = is_val * (std::exp(v_be / vt) - 1.0);
            double I_ec = is_val * (std::exp(v_bc / vt) - 1.0);
            double I_ct = I_cc - I_ec;

            double g_pi = (is_val / (bf * vt)) * std::exp(v_be / vt);
            double g_mu = (is_val / (br * vt)) * std::exp(v_bc / vt);
            double g_mf = (is_val / vt) * std::exp(v_be / vt);
            double g_mr = (is_val / vt) * std::exp(v_bc / vt);

            double sign = is_npn ? 1.0 : -1.0;
            double i_eq_be = ((I_cc / bf) - g_pi * v_be) * sign;
            double i_eq_bc = ((I_ec / br) - g_mu * v_bc) * sign;
            double i_eq_ct = (I_ct - g_mf * v_be + g_mr * v_bc) * sign;

            target_matrix.stamp_conductance(b, e, g_pi);
            target_matrix.stamp_conductance(b, c, g_mu);
            target_matrix.stamp_current_source(b, e, i_eq_be);
            target_matrix.stamp_current_source(b, c, i_eq_bc);
            target_matrix.stamp_current_source(c, e, i_eq_ct);

            if (c > 0 && b > 0) target_matrix.stamp_conductance_direct(c - 1, b - 1, g_mf);
            if (c > 0 && e > 0) target_matrix.stamp_conductance_direct(c - 1, e - 1, -g_mf);
            if (e > 0 && b > 0) target_matrix.stamp_conductance_direct(e - 1, b - 1, -g_mf);
            if (e > 0 && e > 0) target_matrix.stamp_conductance_direct(e - 1, e - 1, g_mf);

            if (e > 0 && b > 0) target_matrix.stamp_conductance_direct(e - 1, b - 1, g_mr);
            if (e > 0 && c > 0) target_matrix.stamp_conductance_direct(e - 1, c - 1, -g_mr);
            if (c > 0 && b > 0) target_matrix.stamp_conductance_direct(c - 1, b - 1, -g_mr);
            if (c > 0 && c > 0) target_matrix.stamp_conductance_direct(c - 1, c - 1, g_mr);
        }
    }
}

bool NewtonRaphson::solve_dc(const Circuit& circuit, MnaMatrix& final_matrix) {
    MnaMatrix current_state(circuit);
    current_state.x.setZero();
    
    size_t num_nodes = circuit.num_nodes();
    for(size_t i = 0; i < num_nodes; ++i) current_state.x(i) = 0.01;

    double damping = 0.3;

    for (size_t iter = 0; iter < max_iterations; ++iter) {
        MnaMatrix next_state(circuit);
        next_state.stamp_static_elements(circuit, 0.0);
        stamp_nonlinear_devices(circuit, current_state, next_state);

        // --- INJECTED: Singular Matrix Exception Handling ---
        if (!next_state.solve()) {
            throw asic::SingularMatrixError("MNA Matrix is singular at iteration " + std::to_string(iter) + 
                                            ". Check for floating nodes, shorted voltage sources, or unanchored topologies.");
        }

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
            max_delta = std::max(max_delta, std::abs(raw_step));
        }

        if (max_delta < tolerance) {
            final_matrix = next_state;
            return true;
        }
        current_state = next_state;
    }

    // --- INJECTED: Convergence Exception Handling ---
    // Replaced the silent 'return true;' which masked non-converging oscillatory states.
    throw asic::ConvergenceError("Solver exceeded " + std::to_string(max_iterations) + 
                                 " iterations without reaching tolerance limit.");
}

} // namespace asic