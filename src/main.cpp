#include "core/circuit.hpp"
#include "solver/newton_raphson.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

struct SimStep {
    double s;
    double r;
    std::string name;
};

int main() {
    asic::Circuit circuit;
    
    // Nodes: GND, VDD(1.2), Set, Reset, Q, Q_bar, and two internal PMOS stack nodes
    size_t gnd = 0, vdd = 1, in_s = 2, in_r = 3, q = 4, q_bar = 5, n_p1 = 6, n_p2 = 7;

    // Power Rails & Inputs (Standard Binary 0V to 1.2V)
    circuit.add<asic::Passives::VoltageSource>("V_DD", vdd, gnd, 1.2, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_S", in_s, gnd, 0.0, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_R", in_r, gnd, 0.0, asic::VoltageType::DC);

    // --- SR LATCH TOPOLOGY (Cross-Coupled NOR Gates) ---

    // NOR Gate 1 (Output = Q)
    // PMOS Stack (Series)
    circuit.add<asic::fet::mos>("MP1", asic::Polarity::P_TYPE, n_p1, in_r, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MP2", asic::Polarity::P_TYPE, q, q_bar, n_p1, vdd, 4.0, 1.0, -0.4);
    // NMOS Stack (Parallel)
    circuit.add<asic::fet::mos>("MN1", asic::Polarity::N_TYPE, q, in_r, gnd, gnd, 2.0, 1.0, 0.4);
    circuit.add<asic::fet::mos>("MN2", asic::Polarity::N_TYPE, q, q_bar, gnd, gnd, 2.0, 1.0, 0.4);

    // NOR Gate 2 (Output = Q_bar)
    // PMOS Stack (Series)
    circuit.add<asic::fet::mos>("MP3", asic::Polarity::P_TYPE, n_p2, in_s, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MP4", asic::Polarity::P_TYPE, q_bar, q, n_p2, vdd, 4.0, 1.0, -0.4);
    // NMOS Stack (Parallel)
    circuit.add<asic::fet::mos>("MN3", asic::Polarity::N_TYPE, q_bar, in_s, gnd, gnd, 2.0, 1.0, 0.4);
    circuit.add<asic::fet::mos>("MN4", asic::Polarity::N_TYPE, q_bar, q, gnd, gnd, 2.0, 1.0, 0.4);

    // The sequence of events to test memory
    std::vector<SimStep> sequence = {
        {1.2, 0.0, "SET     "}, // Force Q to 1.2V
        {0.0, 0.0, "HOLD(1) "}, // Remove inputs, Q should remain 1.2V
        {0.0, 1.2, "RESET   "}, // Force Q to 0.0V
        {0.0, 0.0, "HOLD(0) "}, // Remove inputs, Q should remain 0.0V
        {1.2, 1.2, "INVALID "}, // Both high (forces both to 0V)
    };
    
    std::cout << "--------------------------------------------------------\n";
    std::cout << " Action   |  S(V)  |  R(V)  |    Q(V)    |  Q_bar(V)  \n";
    std::cout << "--------------------------------------------------------\n";
    std::cout << std::fixed << std::setprecision(4);

    // Instantiate state OUTSIDE the loop so it remembers voltages across steps!
    asic::MnaMatrix state(circuit);
    asic::NewtonRaphson solver;

    for (const auto& step : sequence) {
        
        circuit.update_voltage("V_S", step.s);
        circuit.update_voltage("V_R", step.r);
        
        if (solver.solve_dc(circuit, state)) {
            std::cout << " " << step.name << " | " 
                      << std::setw(6) << step.s << " | " 
                      << std::setw(6) << step.r << " | " 
                      << std::setw(10) << state.get_node_voltage(q) << " | "
                      << std::setw(10) << state.get_node_voltage(q_bar) << "\n";
        } else {
            std::cout << " " << step.name << " | " << step.s << " | " << step.r << " | FAIL\n";
        }
    }
    std::cout << "--------------------------------------------------------\n";
    
    return 0;
}