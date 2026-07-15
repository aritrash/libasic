# libasic

`libasic` is a custom-built, continuous-time non-linear semiconductor physics engine and circuit simulator written entirely in C++. It is designed specifically to research and model exotic logic topologies, including Balanced Ternary systems, Memristor architectures, and standard CMOS/BJT layouts.

## Core Architecture

Unlike standard logic-gate simulators, `libasic` does not use boolean truth tables. It builds a **Modified Nodal Analysis (MNA)** matrix and solves the underlying differential physics of the silicon using a custom **Newton-Raphson** numerical solver. 

This allows for the accurate simulation of real-world analog effects within digital circuits, including:
* Shoot-through currents and rail tug-of-wars.
* Metastability and saddle-point voltage resolution.
* Subthreshold leakage and non-linear conductance.
* Cross-coupled hysteresis (Flip-Flops/Latches).

## Features

* **Component Library:**
  * **Active:** NMOS, PMOS, NPN, PNP, Transmission Gates (T-Gates).
  * **Passive:** Resistors, Capacitors.
  * **Sources:** DC Voltage, AC Voltage, Pulse Clocks, Ternary Staircase Clocks.
* **Solvers:**
  * **DC Operating Point (`solve_dc`):** Stateful Newton-Raphson solver with physical rail-clamping and relaxation damping.
  * **Transient Solver (`solve`):** Time-domain integration using Backward Euler companion models (WIP).
* **Export:** Natively generates SPICE netlists (`.export_spice`) and CSV waveform data.

## Building the Engine

The project utilizes CMake for cross-platform compilation. 

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Usage Example: Building an SR Latch

libasic provides a highly semantic API for defining circuits and sweeping states.

```cpp
#include "core/circuit.hpp"
#include "solver/newton_raphson.hpp"

int main() {
    asic::Circuit circuit;
    size_t gnd = 0, vdd = 1, in_s = 2, in_r = 3, q = 4, q_bar = 5;

    // Define Power and Inputs
    circuit.add<asic::Passives::VoltageSource>("V_DD", vdd, gnd, 1.2, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_S", in_s, gnd, 0.0, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_R", in_r, gnd, 0.0, asic::VoltageType::DC);

    // Build a Cross-Coupled NOR Gate Latch
    circuit.add<asic::fet::mos>("MN1", asic::Polarity::N_TYPE, q, in_r, gnd, gnd, 2.0, 1.0, 0.4);
    // ... [add remaining MOSFETs]

    // Solve State
    asic::MnaMatrix state(circuit);
    asic::NewtonRaphson solver;
    
    if (solver.solve_dc(circuit, state)) {
        // Output automatically resolves metastability and hysteresis
        std::cout << "Q Voltage: " << state.get_node_voltage(q) << "\n";
    }

    return 0;
}
```

## License and Author
Author: Aritrash Sarkar

Focus: Non-Binary Computing & Systems Engineering

This project is proprietary research architecture.