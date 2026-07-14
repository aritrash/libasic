#ifndef LIBANA_COMPONENTS_HPP
#define LIBANA_COMPONENTS_HPP

#include <string>
#include <vector>

namespace ana {

// Universal designations for signal domains
enum class VoltageType {
    AC,
    DC,
    NDC // Negative DC domain support
};

// Base Component representing any electrical device in the network
class Component {
public:
    std::string name;
    std::vector<size_t> nodes; // Holds connected internal node IDs

    explicit Component(std::string id, std::vector<size_t> connected_nodes)
        : name(std::move(id)), nodes(std::move(connected_nodes)) {}
        
    virtual ~Component() = default;
};

// Independent Voltage Source (Supports AC, DC, and Negative DC rails)
class VoltageSource : public Component {
public:
    double value; 
    VoltageType type;

    VoltageSource(std::string id, size_t pos_node, size_t neg_node, double val, VoltageType t)
        : Component(std::move(id), {pos_node, neg_node}), value(val), type(t) {}
};

// Linear Resistor
class Resistor : public Component {
public:
    double resistance;

    Resistor(std::string id, size_t node_a, size_t node_b, double r)
        : Component(std::move(id), {node_a, node_b}), resistance(r) {}
};

// Generic Three-Terminal Transistor (BJT / FET structural mapping)
class Transistor : public Component {
public:
    std::string model; // e.g., "NPN", "PNP", "NMOS", "PMOS"
    double width;      // Geometry parameters for IC design scale
    double length;
    double v_threshold; 

    Transistor(std::string id, std::string model_type, size_t terminal_1, size_t terminal_2, size_t terminal_3, 
               double w = 1.0, double l = 1.0, double vth = 0.4)
        : Component(std::move(id), {terminal_1, terminal_2, terminal_3}), 
          model(std::move(model_type)), width(w), length(l), v_threshold(vth) {}
};

} // namespace ana

#endif // LIBANA_COMPONENTS_HPP