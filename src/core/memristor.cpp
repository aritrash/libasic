#include "core/components.hpp"
#include <algorithm>

namespace asic {

Passives::Memristor::Memristor(const std::string& n, size_t pos, size_t neg, double ron, double roff, double init_w, double mu)
    : Component(n, {pos, neg})
    , r_on(ron)
    , r_off(roff)
    , w(init_w)
    , mobility(mu) {
}

double Passives::Memristor::get_conductance() const {
    // At a frozen point in time, it acts as a static resistor
    double current_r = r_on * w + r_off * (1.0 - w);
    return 1.0 / current_r;
}

void Passives::Memristor::step_time(double v_drop, double dt) {
    // The continuous-time state derivative for an electronic memristor
    // dw/dt = mobility * (R_on / D^2) * V(t)
    // Assuming physical thickness D is ~10nm, D^2 = 1e-16
    double dw_dt = mobility * (r_on / 1e-16) * v_drop;
    
    // Euler Integration step
    w += dw_dt * dt;
    
    // Hard boundary clamps: You cannot exceed 100% ON or 0% ON physically.
    // We clamp slightly inside the absolute boundaries to prevent divide-by-zero 
    // edge cases in the solvers.
    w = std::clamp(w, 0.001, 0.999); 
}

} // namespace asic