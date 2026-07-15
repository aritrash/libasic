#include "core/circuit.hpp"
#include "solver/transient_solver.hpp"
#include <iostream>
#include <fstream>

int main() {
    asic::Circuit circuit;
    size_t gnd = circuit.get_gnd(), vcc = 1, v_out = 2;

    // Build a standard RC Pull-Up Circuit
    // Time constant (tau) = R * C = 1000 ohms * 1e-6 Farads = 1 millisecond
    circuit.add<asic::Passives::VoltageSource>("V_CC", vcc, gnd, 5.0, asic::VoltageType::DC);
    circuit.add<asic::Passives::Resistor>("R_PULLUP", vcc, v_out, 1000.0);
    
    // 1uF Capacitor, explicitly starting fully discharged (0.0V)
    circuit.add<asic::Passives::Capacitor>("C_LOAD", v_out, gnd, 1e-6, 0.0); 

    std::cout << "--- Exporting Layout ---\n";
    circuit.export_spice(std::cout);

    // Run the Transient Simulation
    asic::MnaMatrix sim_state(circuit);
    
    // Step size: 50 microseconds. Stop time: 5 milliseconds.
    asic::TransientSolver solver(50e-6, 5e-3);

    std::ofstream csv_file("transient_output.csv");
    if (csv_file.is_open()) {
        std::cout << "[libasic] Executing Transient Analysis...\n";
        if (solver.solve(circuit, sim_state, csv_file)) {
            std::cout << "[libasic] Transient simulation complete. Data written to 'transient_output.csv'.\n";
            std::cout << "[libasic] Final V_out: " << sim_state.get_node_voltage(v_out) << " V\n";
        }
        csv_file.close();
    } else {
        std::cerr << "[libasic] Fatal Error: Could not open CSV file for writing.\n";
    }

    return 0;
}