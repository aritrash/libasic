#pragma once
#include <stdexcept>
#include <string>

namespace asic {

// Base class for all libasic errors
class SimulationError : public std::runtime_error {
public:
    explicit SimulationError(const std::string& message) : std::runtime_error(message) {}
};

// Thrown when the circuit topology is invalid (e.g., shorting two voltage sources)
class SingularMatrixError : public SimulationError {
public:
    explicit SingularMatrixError(const std::string& message) 
        : SimulationError("Singular Matrix Error: " + message) {}
};

// Thrown when the Newton-Raphson solver oscillates and fails to find a stable DC point
class ConvergenceError : public SimulationError {
public:
    explicit ConvergenceError(const std::string& message) 
        : SimulationError("Convergence Error: " + message) {}
};

} // namespace asic