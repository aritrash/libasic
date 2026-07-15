#ifndef LIBASIC_TRANSIENT_SOLVER_HPP
#define LIBASIC_TRANSIENT_SOLVER_HPP

#include "core/circuit.hpp"
#include "solver/mna_matrix.hpp"
#include "solver/newton_raphson.hpp"
#include <vector>
#include <ostream>

namespace asic {

class TransientSolver {
private:
    double t_step;
    double t_stop;
    NewtonRaphson nr_engine;

    // Stamps the Backward Euler Companion Model for all capacitors
    void stamp_capacitors(const Circuit& circuit, MnaMatrix& target_matrix, const Eigen::VectorXd& v_old, double dt) const;

public:
    explicit TransientSolver(double step_size, double stop_time, double nr_tol = 1e-6, size_t nr_max_iter = 100)
        : t_step(step_size), t_stop(stop_time), nr_engine(nr_tol, nr_max_iter) {}

    // Executes the time-stepping loop and outputs CSV data for waveform plotting
    bool solve(const Circuit& circuit, MnaMatrix& state, std::ostream& csv_out);
};

} // namespace asic

#endif // LIBASIC_TRANSIENT_SOLVER_HPP