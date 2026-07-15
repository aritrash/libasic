# libasic

**libasic** is a high-performance, general-purpose, object-oriented ASIC (Application-Specific Integrated Circuit) simulation and design library written in C++20. 

It is designed to bridge the gap between advanced analytical simulation (including future Machine Learning surrogate modeling) and strict, foundry-compliant physical verification.

## Core Architecture
`libasic` utilizes a **Dual-Engine Pipeline** philosophy:
1. **The Unified Device API (C++ Frontend):** A strictly typed, highly intuitive namespace architecture (`asic::fet`, `asic::bjt`) that allows designers to build complex silicon topologies programmatically without wrangling plaintext SPICE files.
2. **The Physics Dispatcher (Backend):** A custom Newton-Raphson non-linear solver operating over an Eigen-backed Modified Nodal Analysis (MNA) matrix. It calculates highly precise DC operating points using advanced short-channel and transport physics.
3. **The Foundry Exporter:** Automatically compiles the C++ structural graph into an industry-standard SPICE netlist (`.sp`) for verification in standard EDA tools (e.g., KLayout, Cadence, ngspice).

## Supported Semiconductor Physics
Unlike basic academic solvers, `libasic` implements true planar and transport physics:
* **MOSFETs (`asic::fet::mos`):** 4-terminal architecture supporting subthreshold conduction, velocity saturation, channel length modulation, and dynamic body effects ($V_{th}$ shifting).
* **BJTs (`asic::bjt::npn`, `asic::bjt::pnp`):** Ebers-Moll / Gummel-Poon base implementations accounting for forward/reverse transconductance and exponential thermal carrier transport.

## Quick Start

### 1. Requirements
* C++20 Compiler (GCC, Clang, or MSVC)
* CMake 3.20+
* *Note: Eigen3 is fetched automatically via CMake.*

### 2. Building the Project
```bash
git clone [https://github.com/aritrash/libasic.git](https://github.com/aritrash/libasic.git)
cd libasic
cmake -B build -S .
cmake --build build
```

### 3. Example Usage: BJT Inverter / Amplifier
```cpp
#include "core/circuit.hpp"
#include "solver/mna_matrix.hpp"
#include "solver/newton_raphson.hpp"
#include <iostream>

int main() {
    asic::Circuit circuit;
    size_t gnd = circuit.get_gnd(), vcc = 1, v_out = 2, v_base = 3;

    // Build the topology
    circuit.add<asic::Passives::VoltageSource>("V_CC", vcc, gnd, 5.0, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_IN", v_base, gnd, 0.75, asic::VoltageType::DC);
    circuit.add<asic::Passives::Resistor>("R_LOAD", vcc, v_out, 1000.0);
    circuit.add<asic::bjt::npn>("Q_SWITCH", v_out, v_base, gnd);

    // Export Foundry Netlist
    circuit.export_spice(std::cout);

    // Run Physics Engine
    asic::MnaMatrix sim_state(circuit);
    asic::NewtonRaphson solver(1e-6, 100);

    if (solver.solve_dc(circuit, sim_state)) {
        sim_state.print_system();
        std::cout << "Output Voltage: " << sim_state.get_node_voltage(v_out) << " V\n";
    }
    return 0;
}
```