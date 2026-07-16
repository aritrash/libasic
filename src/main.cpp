#include "core/circuit.hpp"
#include "solver/transient_solver.hpp"
#include <iostream>
#include <fstream>

int main() {
    asic::Circuit circuit;
    size_t gnd = 0, in_node = 1, mid_node = 2;

    // Apply a +1.2V pulse to switch the memristor ON
    circuit.add<asic::Passives::PulseSource>("V_IN", in_node, gnd, 0.0, 1.2, 1e-3, 1e-5, 1e-5, 2e-3, 10e-3);

    // Memristor parameters: Ron = 1k, Roff = 100k, init_w = 0.01 (mostly OFF), mobility = 2e-14
    circuit.add<asic::Passives::Memristor>("MEM1", in_node, mid_node, 1000.0, 100000.0, 0.01, 2e-14);

    // A static read resistor to create a voltage divider so we can measure the state change
    circuit.add<asic::Passives::Resistor>("R_LOAD", mid_node, gnd, 10000.0);

    asic::MnaMatrix sim_state(circuit);
    asic::TransientSolver solver(1e-5, 5e-3); // Run for 5ms

    std::ofstream csv_file("memristor_test.csv");
    if (csv_file.is_open()) {
        std::cout << "[libasic] Simulating Memristor State Dynamics...\n";
        solver.solve(circuit, sim_state, csv_file);
    }
    
    return 0;
}