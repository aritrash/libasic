#include "core/circuit.hpp"
#include "solver/mna_matrix.hpp"
#include "solver/newton_raphson.hpp"
#include <iostream>

int main() {
    asic::Circuit circuit;
    size_t gnd = circuit.get_gnd(), vcc = 1, v_out = 2, v_base = 3;

    // Build a basic BJT Inverter / Amplifier
    circuit.add<asic::Passives::VoltageSource>("V_CC", vcc, gnd, 5.0, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_IN", v_base, gnd, 0.75, asic::VoltageType::DC); // Forward biases the BJT
    circuit.add<asic::Passives::Resistor>("R_LOAD", vcc, v_out, 1000.0);
    
    // Add the NPN BJT: Collector, Base, Emitter
    circuit.add<asic::bjt::npn>("Q_SWITCH", v_out, v_base, gnd);

    // Export layout for verification
    circuit.export_spice(std::cout);

    // Run the physical simulation
    asic::MnaMatrix sim_state(circuit);
    asic::NewtonRaphson solver(1e-6, 100);

    if (solver.solve_dc(circuit, sim_state)) {
        sim_state.print_system();
        std::cout << "Simulated Output Node Voltage: " << sim_state.get_node_voltage(v_out) << " V\n";
    }

    return 0;
}