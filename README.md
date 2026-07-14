# libana

`libana` is a minimal, high-performance C++ analog circuit simulation and nodal analysis library. It provides an object-oriented API to construct continuous-time electrical networks, simulate non-linear device behaviors, and export standard SPICE netlists for physical ASIC layout generation.

Unlike traditional binary-constrained digital design tools, `libana` operates completely in continuous voltage domains, making it natively suited for exploring multi-valued logic (MVL), balanced ternary architectures, mixed-signal hardware blocks, and neuromorphic computing.

## Features

* **Continuous-Time Simulation:** Supports arbitrary voltage domains, including AC, DC, and Negative DC (`NDC`) configurations.
* **Non-Binary Friendly:** Perfect for designing architectures that rely on precise sub-threshold and multi-rail voltage switching.
* **SPICE Exporter:** Compiles object-oriented code-defined circuit graphs directly into industry-standard SPICE netlists compatible with tools like KLayout.
* **Zero-Crap Architecture:** Lightweight, header-clean structure built on modern C++20 and powered by the `Eigen` matrix math library.

## Project Structure

```text
libana/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── src/
│   ├── main.cpp            # Verification playground
│   ├── core/               # Electrical Network Topology
│   │   ├── circuit.hpp
│   │   ├── circuit.cpp
│   │   └── components.hpp
│   └── solver/             # Numerical Math Engine
|       ├── mna_matrix.hpp
│       ├── mna_matrix.cpp
│       └── newton_raphson.hpp
└── tests/
```
---

## Getting Started

### Prerequisites

- CMake (>= 3.14)
- A modern C++ compiler supporting C++20

### Building
The library automatically fetches the required Eigen dependency during the configuration step via CMake's FetchContent.

```bash
mkdir build && cd build
cmake ..
cmake --build .
```