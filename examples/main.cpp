#include "asic/circuit.hpp"
#include "asic/newton_raphson.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cstdlib> // For std::system

void generate_gnuplot() {
    std::cout << "[libasic] Generating Gnuplot script...\n";
    std::ofstream script("plot_vtc.plt");
    
    if (!script.is_open()) {
        std::cerr << "[libasic] Failed to create plot_vtc.plt\n";
        return;
    }

    // Write the Gnuplot commands
    script << "set datafile separator ','\n";
    // We use pngcairo for high quality anti-aliased rendering
    script << "set terminal pngcairo size 800,800 enhanced font 'Segoe UI,12'\n";
    script << "set output 'cmos_inverter_verification.png'\n";
    script << "set multiplot layout 2,1 title 'CMOS Inverter: Physical Verification' font ',14'\n\n";

    // --- Top Plot: VT Curve ---
    script << "set title 'VT Curve (Voltage Transfer)'\n";
    script << "set ylabel 'V_OUT (V)'\n";
    script << "set grid back ls 12 lc rgb '#cccccc'\n";
    // Draw a vertical line at the exact 0.6V switching threshold
    script << "set arrow 1 from 0.6, graph 0 to 0.6, graph 1 nohead dt 3 lc rgb 'gray'\n";
    // Plot skipping the first line (header) using columns 1 and 2
    script << "plot 'inverter_vtc.csv' every ::1 using 1:2 with lines lw 2.5 lc rgb '#1f77b4' title ''\n\n";

    // --- Bottom Plot: IT Curve ---
    script << "set title 'IT Curve (Current Transfer)'\n";
    script << "set xlabel 'V_IN (V)'\n";
    script << "set ylabel 'Shoot-Through Current ({/Symbol m}A)'\n";
    // Plot using columns 1 and 3. We take the absolute value of I_VDD and multiply by 1e6 for microamps
    script << "plot 'inverter_vtc.csv' every ::1 using 1:(abs($3)*1e6) with lines lw 2.5 lc rgb '#d62728' title ''\n\n";

    script << "unset multiplot\n";
    script.close();

    // Command the OS to run the script
    std::cout << "[libasic] Executing Gnuplot...\n";
    int ret = std::system("gnuplot plot_vtc.plt");
    
    if (ret == 0) {
        std::cout << "[libasic] Success! Plot saved to cmos_inverter_verification.png\n";
    } else {
        std::cerr << "[libasic] ERROR: Gnuplot failed. Make sure 'gnuplot' is installed and in your system PATH.\n";
    }
}

void test_cmos_inverter_vtc() {
    std::cout << "--- Running CMOS Inverter VTC Sweep ---\n";
    asic::Circuit circuit;
    size_t gnd = 0, vdd = 1, in = 2, out = 3;

    circuit.add<asic::Passives::VoltageSource>("V_DD", vdd, gnd, 1.2, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_IN", in, gnd, 0.0, asic::VoltageType::DC);

    // Standard CMOS Inverter
    circuit.add<asic::fet::mos>("MP1", asic::Polarity::P_TYPE, out, in, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MN1", asic::Polarity::N_TYPE, out, in, gnd, gnd, 2.0, 1.0, 0.4);

    std::ofstream netlist_file("cmos_inverter.sp");
    if (netlist_file.is_open()) circuit.export_spice(netlist_file);

    asic::MnaMatrix sim_state(circuit);
    asic::NewtonRaphson solver;

    std::ofstream csv("inverter_vtc.csv");
    csv << "V_IN,V_OUT,I_VDD\n";

    for (double v_in = 0.0; v_in <= 1.2; v_in += 0.01) {
        circuit.update_voltage("V_IN", v_in);
        if (solver.solve_dc(circuit, sim_state)) {
            double v_out = sim_state.get_node_voltage(out);
            double i_vdd = sim_state.get_source_current("V_DD"); 
            csv << v_in << "," << v_out << "," << i_vdd << "\n";
        }
    }
    std::cout << "[libasic] VTC Data written to inverter_vtc.csv\n";
    
    // Call the newly created gnuplot generator
    generate_gnuplot();
    std::cout << "\n";
}

void test_cmos_nor() {
    std::cout << "--- Running CMOS NOR Gate Truth Table ---\n";
    asic::Circuit circuit;
    size_t gnd = 0, vdd = 1, a = 2, b = 3, out = 4, mid = 5;

    circuit.add<asic::Passives::VoltageSource>("V_DD", vdd, gnd, 1.2, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_A", a, gnd, 0.0, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_B", b, gnd, 0.0, asic::VoltageType::DC);

    circuit.add<asic::fet::mos>("MP1", asic::Polarity::P_TYPE, mid, a, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MP2", asic::Polarity::P_TYPE, out, b, mid, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MN1", asic::Polarity::N_TYPE, out, a, gnd, gnd, 2.0, 1.0, 0.4);
    circuit.add<asic::fet::mos>("MN2", asic::Polarity::N_TYPE, out, b, gnd, gnd, 2.0, 1.0, 0.4);

    asic::MnaMatrix sim_state(circuit);
    asic::NewtonRaphson solver;

    std::vector<double> logic_levels = {0.0, 1.2};
    std::cout << "  A(V)  |  B(V)  | OUT(V) \n";
    std::cout << "--------------------------\n";
    std::cout << std::fixed << std::setprecision(4);

    for (double v_a : logic_levels) {
        for (double v_b : logic_levels) {
            circuit.update_voltage("V_A", v_a);
            circuit.update_voltage("V_B", v_b);
            if (solver.solve_dc(circuit, sim_state)) {
                std::cout << " " << v_a << " | " << v_b << " | " << sim_state.get_node_voltage(out) << "\n";
            }
        }
    }
    std::cout << "--------------------------\n";
}

int main() {
    test_cmos_inverter_vtc();
    test_cmos_nor();
    return 0;
}