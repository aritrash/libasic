#ifndef LIBASIC_MNA_MATRIX_HPP
#define LIBASIC_MNA_MATRIX_HPP

#include "core/circuit.hpp"
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>

namespace asic {

class MnaMatrix {
private:
    size_t num_nodes;
    size_t num_sources;
    size_t matrix_dim;

    // Track which components map to which extra rows/columns in the MNA matrix
    std::unordered_map<std::string, size_t> source_index_map;

    // The A * x = z linear framework components
    Eigen::MatrixXd A; // System Coefficient Matrix
    Eigen::VectorXd z; // Contributions Vector (Knowns)
    Eigen::VectorXd x; // Operating Point Vector (Unknown Voltages/Currents)

public:
    explicit MnaMatrix(const Circuit& circuit);

    // Core Builders
    void clear();
    void stamp_static_elements(const Circuit& circuit);
    
    // Non-linear interaction hook for Newton-Raphson iteration
    void stamp_conductance(size_t node_a, size_t node_b, double conductance);
    void stamp_current_source(size_t pos_node, size_t neg_node, double current);
    void stamp_conductance_direct(size_t row, size_t col, double value) { A(row, col) += value; }

    // System Solvers
    bool solve();

    // Data Accessors
    double get_node_voltage(size_t node_id) const;
    double get_source_current(const std::string& source_name) const;
    
    // Debug helper to print matrix state to console
    void print_system() const;
};

} // namespace asic

#endif // LIBASIC_MNA_MATRIX_HPP