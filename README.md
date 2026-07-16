# libasic 

**libasic** is a high-performance, continuous-time Electronic Design Automation (EDA) backend and physics engine written in C++17. 

Originally developed to model analog non-linear topologies, `libasic` serves as the foundational simulation substrate for research into **Balanced Ternary Memristive Systolic Arrays** and **Dataflow Architectures**. It bridges the gap between discrete digital logic and continuous-time silicon physics, enabling researchers to natively simulate Compute-in-Memory (CIM) crossbars, subthreshold CMOS leakage, and ternary data pathways.

---

## Features

* **Continuous-Time Non-Linear Physics**
  * Core Modified Nodal Analysis (MNA) matrix solver powered by `Eigen3`.
  * Robust Newton-Raphson iterations with physical damping, rail-clamping, and custom convergence validation.
  * Advanced semiconductor models: MOSFETs (handling subthreshold conduction, velocity saturation, dynamic threshold body-effects), BJTs (Ebers-Moll), and Transmission Gates natively unrolled.
* **Memristive & Ternary Compute Ready**
  * Native 1T1R (Transistor-Memristor) crossbar primitives.
  * Native simulation of Balanced Ternary states ($-1, 0, +1$) with split-rail continuous-time physics.
* **Interoperability & Verification**
  * **SPICE Export:** Procedurally generates Berkeley-standard `.sp` netlists for direct cross-verification against LTSpice or NGSpice.
  * **Native Graphing Pipeline:** Outputs structural `.csv` data and generates native `gnuplot` scripts for high-resolution Voltage Transfer (VTC) and Shoot-Through Current (IT) characteristic curves.
* **Production-Grade Architecture**
  * **Safe Executions:** Comprehensive C++ exception hierarchy (`asic::SingularMatrixError`, `asic::ConvergenceError`) to catch floating nodes and unanchored topologies gracefully.
  * **CI/CD Ready:** Fully anchored by an automated `GoogleTest` suite.
  * **System Deployment:** Native CMake `install` targets for system-wide module linking (`find_package(libasic)`).

---

## Prerequisites

* **C++ Compiler:** Requires full C++17 support (GCC 9+, Clang 10+, MSVC 19.2+).
* **CMake:** Version 3.14 or higher.
* **Gnuplot:** (Optional) Required only if utilizing the automated graphing pipeline for VTC/IT curves.
* *Note: Dependencies like `Eigen3` and `GoogleTest` are handled automatically via CMake `FetchContent`.*

---

## Build & Installation

`libasic` can be built and installed system-wide (or to a local prefix) so it can be seamlessly linked by other C++ projects.

```bash
# 1. Clone the repository
git clone [https://github.com/yourusername/libasic.git](https://github.com/yourusername/libasic.git)
cd libasic

# 2. Create the build environment
mkdir build && cd build

# 3. Configure CMake 
# (Optional: specify an install directory with -DCMAKE_INSTALL_PREFIX="/your/path")
cmake .. 

# 4. Compile the library and examples
cmake --build .

# 5. Run the validation suite
ctest --output-on-failure

# 6. Install to system (requires admin/sudo depending on prefix)
cmake --install .
```

## Quick Start: Simulating a CMOS Inverter
Once installed, you can link libasic to any CMake project using find_package(libasic REQUIRED) and target_link_libraries(your_target PRIVATE libasic::asic).

```cpp
#include <asic/circuit.hpp>
#include <asic/newton_raphson.hpp>
#include <asic/mna_matrix.hpp>
#include <iostream>

int main() {
    // 1. Initialize Circuit and Nodes
    asic::Circuit circuit;
    size_t gnd = 0, vdd = 1, in = 2, out = 3;

    // 2. Define Power and Input Rails
    circuit.add<asic::Passives::VoltageSource>("V_DD", vdd, gnd, 1.2, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_IN", in, gnd, 0.0, asic::VoltageType::DC);

    // 3. Build the CMOS Topology
    circuit.add<asic::fet::mos>("MP1", asic::Polarity::P_TYPE, out, in, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MN1", asic::Polarity::N_TYPE, out, in, gnd, gnd, 2.0, 1.0, 0.4);

    // 4. Initialize the Solver
    asic::MnaMatrix sim_state(circuit);
    asic::NewtonRaphson solver;

    // 5. Sweep the Input to generate a Voltage Transfer Curve (VTC)
    for (double v_in = 0.0; v_in <= 1.2; v_in += 0.1) {
        circuit.update_voltage("V_IN", v_in);
        
        if (solver.solve_dc(circuit, sim_state)) {
            std::cout << "IN: " << v_in << "V | OUT: " 
                      << sim_state.get_node_voltage(out) << "V\n";
        }
    }

    // 6. Export to SPICE for verification
    circuit.export_spice("cmos_inverter.sp");

    return 0;
}
```

## Project Structure

```text
libasic/
├── CMakeLists.txt          # Root CMake orchestrator
├── include/asic/           # Public API headers
│   ├── circuit.hpp         # Core netlist topology container
│   ├── components.hpp      # Primitives (R, C, V, MOS, BJT, Memristor)
│   ├── exceptions.hpp      # Custom Simulation Errors
│   ├── mna_matrix.hpp      # Matrix memory and dynamic stampers
│   └── newton_raphson.hpp  # Non-linear solver algorithms
├── src/                    # Private implementation files
├── tests/                  # GoogleTest automated CI suite
├── examples/               # Usage examples and structural testbenches
└── third_party/            # Dependency management (Eigen3, GTest)
```

## License
Distributed under the MIT License. See LICENSE for more information.