# libasic

**libasic** is a continuous-time, non-linear semiconductor physics engine and circuit simulator written entirely in native C++. Designed to break away from the limitations of standard binary truth-table simulators, `libasic` evaluates the fundamental differential physics of silicon to research and model exotic logic topologies, including Balanced Ternary systems, Memristor crossbar arrays, and foundational Compute-in-Memory (CIM) architectures.

## Overview

Unlike standard digital logic simulators, `libasic` utilizes a **Modified Nodal Analysis (MNA)** core driven by a custom **Newton-Raphson** numerical solver. This allows the engine to accurately simulate real-world analog silicon effects within digital architectures, including:

* **Metastability & Hysteresis:** Natively resolves cross-coupled feedback loops (e.g., SR Latches) and saddle-point voltages without artificial boolean forcing.
* **Shoot-Through Currents:** Accurately models rail-to-rail tug-of-wars during intermediate voltage states.
* **Non-Volatile Analog State:** Simulates continuous-time electron trapping/de-trapping in electronic memristors for multi-level (ternary) resistance states.
* **Physical Propagation Delay:** Evaluates RC time constants and capacitive charging curves dynamically.

## Core Capabilities

### Solvers
* **Stateful DC Operating Point (`solve_dc`):** A robust Newton-Raphson solver featuring physical rail-clamping (±1.3V) and relaxation damping to guarantee convergence even in highly non-linear, positive-feedback topologies.
* **Adaptive Transient Engine (`solve`):** A time-domain solver utilizing Backward Euler integration. It features an adaptive time-stepper that dynamically shrinks the time delta (`dt`) to resolve instantaneous voltage shocks, then accelerates during steady-state holds.

### Component Library
* **Active Silicon:** Planar MOSFETs (NMOS/PMOS via Shichman-Hodges), BJTs (NPN/PNP via Ebers-Moll/Gummel-Poon), and composite Transmission Gates (T-Gates).
* **Dynamic Passives:** Linear Resistors, Capacitors, and **Electronic Memristors** (Voltage-driven HP models adapted for continuous amorphous silicon physics).
* **Stimulus Sources:** DC Voltage, AC Voltage, dynamic SPICE-accurate Pulse Clocks, and Ternary Staircase Clocks.

### Output & Data
* **Data Pipelining:** Direct integration with CSV file generation for high-resolution, time-domain waveform plotting.
* **EDA Compatibility:** Natively exports circuit topologies to standard SPICE netlists (`.export_spice`) for cross-verification.

## Build Instructions

The project utilizes CMake for cross-platform, dependency-free compilation. 

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

(Ensure your compiler supports C++17 or higher for std::make_unique and std::clamp capabilities).

## Usage Example: Transient Memristor Simulation
libasic provides a highly semantic, object-oriented API for defining circuits and sweeping temporal states.

```cpp
#include "core/circuit.hpp"
#include "core/memristor.hpp"
#include "solver/transient_solver.hpp"
#include <iostream>
#include <fstream>

int main() {
    asic::Circuit circuit;
    size_t gnd = 0, in_node = 1, mid_node = 2;

    // Apply a +1.2V write pulse
    circuit.add<asic::Passives::PulseSource>("V_IN", in_node, gnd, 0.0, 1.2, 1e-3, 1e-5, 1e-5, 2e-3, 10e-3);

    // Instantiate a Non-Volatile Electronic Memristor (1kΩ to 100kΩ)
    circuit.add<asic::Passives::Memristor>("MEM1", in_node, mid_node, 1000.0, 100000.0, 0.01, 2e-14);

    // Static read resistor to create a voltage divider
    circuit.add<asic::Passives::Resistor>("R_LOAD", mid_node, gnd, 10000.0);

    // Initialize Solvers
    asic::MnaMatrix sim_state(circuit);
    asic::TransientSolver solver(1e-5, 5e-3); // t_step = 10µs, t_stop = 5ms

    // Execute Adaptive Transient Integration
    std::ofstream csv_file("memristor_waveform.csv");
    if (csv_file.is_open()) {
        solver.solve(circuit, sim_state, csv_file);
    }
    
    return 0;
}
```

## Architecture Notes
- Decoupled Physics: Dynamic state variables (like the derivative $dw/dt$ of a memristor) are evaluated in isolated component files. The Newton-Raphson engine queries these objects strictly for their instantaneous linear conductance at a frozen point in time ($t$), ensuring $O(N^3)$ matrix inversion remains heavily optimized.
- Polymorphic Stamping: The MNA matrix dynamically casts and stamps active devices into the Jacobian matrix using a uniform Component base class with RTTI enabled.

## Author & Research Context
Author: Aritrash Sarkar
Application: Ternary Photonics Research & Non-Binary Polarization-Coded Photonic Computing

libasic was developed as a foundational testbed to model the physical bridging between traditional planar silicon integration (FEOL) and novel ternary computational models (BEOL memristor arrays), paving the way for next-generation compute-in-memory hardware.