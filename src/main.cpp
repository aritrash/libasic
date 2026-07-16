#include "core/circuit.hpp"
#include "solver/transient_solver.hpp"
#include <iostream>
#include <fstream>

int main() {
    asic::Circuit circuit;
    
    size_t gnd = 0, vdd = 1, in_s = 2, in_r = 3, q = 4, q_bar = 5, n_p1 = 6, n_p2 = 7;

    circuit.add<asic::Passives::VoltageSource>("V_DD", vdd, gnd, 1.2, asic::VoltageType::DC);

    // Dynamic inputs: V_S pulses at 0.5ms, V_R pulses at 2.0ms
    circuit.add<asic::Passives::PulseSource>("V_S", in_s, gnd, 0.0, 1.2, 0.5e-3, 5e-5, 5e-5, 0.5e-3, 10e-3);
    circuit.add<asic::Passives::PulseSource>("V_R", in_r, gnd, 0.0, 1.2, 2.0e-3, 5e-5, 5e-5, 0.5e-3, 10e-3);

    // --- SR LATCH TOPOLOGY ---
    circuit.add<asic::fet::mos>("MP1", asic::Polarity::P_TYPE, n_p1, in_r, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MP2", asic::Polarity::P_TYPE, q, q_bar, n_p1, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MN1", asic::Polarity::N_TYPE, q, in_r, gnd, gnd, 2.0, 1.0, 0.4);
    circuit.add<asic::fet::mos>("MN2", asic::Polarity::N_TYPE, q, q_bar, gnd, gnd, 2.0, 1.0, 0.4);

    circuit.add<asic::fet::mos>("MP3", asic::Polarity::P_TYPE, n_p2, in_s, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MP4", asic::Polarity::P_TYPE, q_bar, q, n_p2, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MN3", asic::Polarity::N_TYPE, q_bar, in_s, gnd, gnd, 2.0, 1.0, 0.4);
    circuit.add<asic::fet::mos>("MN4", asic::Polarity::N_TYPE, q_bar, q, gnd, gnd, 2.0, 1.0, 0.4);

    // --- PHYSICAL MEMORY ---
    circuit.add<asic::Passives::Capacitor>("C_Q", q, gnd, 50e-12, 0.0);
    circuit.add<asic::Passives::Capacitor>("C_QBAR", q_bar, gnd, 50e-12, 0.0);

    // DC stabilization anchors for the initial transient Op-Point
    circuit.add<asic::Passives::Resistor>("GMIN_Q", q, gnd, 1e9); 
    circuit.add<asic::Passives::Resistor>("GMIN_QB", q_bar, gnd, 1e9); 

    asic::MnaMatrix sim_state(circuit);
    asic::TransientSolver solver(1e-5, 4e-3); 

    std::ofstream csv_file("sr_latch_waveform.csv");
    if (csv_file.is_open()) {
        std::cout << "[libasic] Starting 4ms Adaptive Transient Simulation...\n";
        solver.solve(circuit, sim_state, csv_file);
    }
    
    return 0;
}