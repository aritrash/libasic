#ifndef LIBASIC_NEWTON_RAPHSON_HPP
#define LIBASIC_NEWTON_RAPHSON_HPP

#include "asic/circuit.hpp"
#include "asic/mna_matrix.hpp"

namespace asic {

class NewtonRaphson {
private:
    double tolerance;
    size_t max_iterations;

    // Evaluates whether node voltages have converged within the tolerance bounds
    bool check_convergence(const Eigen::VectorXd& old_v, const MnaMatrix& matrix, size_t num_nodes) const;

public:
    explicit NewtonRaphson(double tol = 1e-6, size_t max_iter = 100)
        : tolerance(tol), max_iterations(max_iter) {}

    // Solves the DC operating point of the circuit for a static state
    bool solve_dc(const Circuit& circuit, MnaMatrix& matrix);
    
    // Linearly approximates active silicon devices based on current voltage guesses
    void stamp_nonlinear_devices(const Circuit& circuit, const MnaMatrix& current_state, MnaMatrix& target_matrix);
    void stamp_single_mosfet(size_t d, size_t g, size_t s, size_t b, 
                             Polarity polarity, double w, double l, double vth, 
                             double gamma, double phi, 
                             const MnaMatrix& current_state, MnaMatrix& target_matrix) const;
};

} // namespace asic

#endif // LIBASIC_NEWTON_RAPHSON_HPP