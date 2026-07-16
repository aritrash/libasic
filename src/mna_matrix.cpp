#include "asic/mna_matrix.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

namespace asic {

MnaMatrix::MnaMatrix(const Circuit& circuit) {
    num_nodes = circuit.num_nodes();
    num_sources = 0;

    // First pass: Count independent voltage sources and index map them
    for (const auto& comp : circuit.get_components()) {
        if (dynamic_cast<const Passives::VoltageSource*>(comp.get()) ||
            dynamic_cast<const Passives::PulseSource*>(comp.get()) ||
            dynamic_cast<const Passives::TernaryClock*>(comp.get())) { 
            source_index_map[comp->name] = num_sources++;
        }
    }

    // Dimension: Node voltages (excluding GND) + Source currents
    matrix_dim = num_nodes + num_sources;
    
    A = Eigen::MatrixXd::Zero(matrix_dim, matrix_dim);
    z = Eigen::VectorXd::Zero(matrix_dim);
    x = Eigen::VectorXd::Zero(matrix_dim);
}

void MnaMatrix::clear() {
    A.setZero();
    z.setZero();
}

void MnaMatrix::stamp_conductance(size_t node_a, size_t node_b, double conductance) {
    if (node_a > 0) A(node_a - 1, node_a - 1) += conductance;
    if (node_b > 0) A(node_b - 1, node_b - 1) += conductance;
    
    if (node_a > 0 && node_b > 0) {
        A(node_a - 1, node_b - 1) -= conductance;
        A(node_b - 1, node_a - 1) -= conductance;
    }
}

void MnaMatrix::stamp_current_source(size_t pos_node, size_t neg_node, double current) {
    if (pos_node > 0) z(pos_node - 1) -= current;
    if (neg_node > 0) z(neg_node - 1) += current;
}

void MnaMatrix::stamp_static_elements(const Circuit& circuit, double t) {
    for (const auto& comp : circuit.get_components()) {
        
        if (auto r = dynamic_cast<const Passives::Resistor*>(comp.get())) {
            double g = 1.0 / r->resistance;
            stamp_conductance(r->nodes[0], r->nodes[1], g);
        }
        else if (auto v = dynamic_cast<const Passives::VoltageSource*>(comp.get())) {
            size_t pos = v->nodes[0];
            size_t neg = v->nodes[1];
            size_t idx = num_nodes + source_index_map[v->name];
            double val = (v->type == VoltageType::NDC) ? -std::abs(v->value) : v->value;
            
            if (pos > 0) { A(pos - 1, idx) += 1.0; A(idx, pos - 1) += 1.0; }
            if (neg > 0) { A(neg - 1, idx) -= 1.0; A(idx, neg - 1) -= 1.0; }
            z(idx) = val;
        }
        else if (auto p = dynamic_cast<const Passives::PulseSource*>(comp.get())) {
            size_t pos = p->nodes[0];
            size_t neg = p->nodes[1];
            size_t idx = num_nodes + source_index_map[p->name];
            
            double val = p->get_voltage(t);
            
            if (pos > 0) { A(pos - 1, idx) += 1.0; A(idx, pos - 1) += 1.0; }
            if (neg > 0) { A(neg - 1, idx) -= 1.0; A(idx, neg - 1) -= 1.0; }
            z(idx) = val;
        }
        else if (auto tclk = dynamic_cast<const Passives::TernaryClock*>(comp.get())) {
            size_t pos = tclk->nodes[0];
            size_t neg = tclk->nodes[1];
            size_t idx = num_nodes + source_index_map[tclk->name];
            
            double val = tclk->get_voltage(t);
            
            if (pos > 0) { A(pos - 1, idx) += 1.0; A(idx, pos - 1) += 1.0; }
            if (neg > 0) { A(neg - 1, idx) -= 1.0; A(idx, neg - 1) -= 1.0; }
            z(idx) = val;
        } 
        else if (auto mem = dynamic_cast<const Passives::Memristor*>(comp.get())) {
            stamp_conductance(mem->nodes[0], mem->nodes[1], mem->get_conductance());
        }
    }

    double gmin = 1e-7;
    for (size_t i = 0; i < num_nodes; ++i) A(i, i) += gmin;
}

bool MnaMatrix::solve() {
    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
    if (!lu.isInvertible()) {
        std::cerr << "[libasic] CRITICAL: Singular matrix detected.\n";
        return false;
    }
    x = lu.solve(z);
    return true;
}

double MnaMatrix::get_node_voltage(size_t node_id) const {
    if (node_id == 0 || node_id > num_nodes) return 0.0;
    return x(node_id - 1);
}

double MnaMatrix::get_source_current(const std::string& source_name) const {
    auto it = source_index_map.find(source_name);
    if (it == source_index_map.end()) return 0.0;
    return x(num_nodes + it->second);
}

void MnaMatrix::print_system() const {
    std::cout << std::setprecision(4) << std::fixed;
    std::cout << "--- MNA Matrix System System (A) ---\n" << A << "\n";
    std::cout << "--- Contributions Vector (z) ---\n" << z.transpose() << "\n";
    std::cout << "--- Solved Solution Vector (x) ---\n" << x.transpose() << "\n\n";
}

} // namespace asic