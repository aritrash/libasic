#ifndef LIBASIC_COMPONENTS_HPP
#define LIBASIC_COMPONENTS_HPP

#include <string>
#include <vector>

namespace asic {

enum class VoltageType { AC, DC, NDC };
enum class Polarity { N_TYPE, P_TYPE }; // Unified polarity for NMOS/PMOS and NPN/PNP

class Component {
public:
    std::string name;
    std::vector<size_t> nodes;
    
    explicit Component(std::string id, std::vector<size_t> connected_nodes)
        : name(std::move(id)), nodes(std::move(connected_nodes)) {}
    virtual ~Component() = default;
};

class Passives {
public:
    class Resistor : public Component {
    public:
        double resistance;
        Resistor(std::string id, size_t node_a, size_t node_b, double r)
            : Component(std::move(id), {node_a, node_b}), resistance(r) {}
    };

    class VoltageSource : public Component {
    public:
        double value; 
        VoltageType type;
        VoltageSource(std::string id, size_t pos_node, size_t neg_node, double val, VoltageType t)
            : Component(std::move(id), {pos_node, neg_node}), value(val), type(t) {}
    };

    class Capacitor : public Component {
    public:
        double capacitance;
        double initial_voltage; // Used for SPICE .IC and transient startup state

        Capacitor(std::string id, size_t node_a, size_t node_b, double c, double ic = 0.0)
            : Component(std::move(id), {node_a, node_b}), capacitance(c), initial_voltage(ic) {}
    };
};

// Field Effect Transistor API
namespace fet {
    class mos : public Component {
    public:
        Polarity polarity; // N_TYPE (NMOS) or P_TYPE (PMOS)
        double width;      
        double length;
        double v_threshold; // Zero-bias threshold (Vth0)
        double gamma;       // Body effect coefficient
        double phi;         // Surface potential (2*Phi_F)

        mos(std::string id, Polarity p, size_t drain, size_t gate, size_t source, size_t bulk, 
            double w = 1.0, double l = 1.0, double vth = 0.4, double g = 0.4, double ph = 0.65)
            : Component(std::move(id), {drain, gate, source, bulk}), 
              polarity(p), width(w), length(l), v_threshold(vth), gamma(g), phi(ph) {}
    };

    class j : public Component {
        // JFET Implementation (Placeholder for future)
    };
}

// Bipolar Junction Transistor API
namespace bjt {
    class npn : public Component {
    public:
        double is; // Saturation current
        double bf; // Forward common-emitter current gain (Beta)
        double br; // Reverse common-emitter current gain

        npn(std::string id, size_t collector, size_t base, size_t emitter, 
            double saturation_current = 1e-16, double forward_beta = 100.0, double reverse_beta = 1.0)
            : Component(std::move(id), {collector, base, emitter}), 
              is(saturation_current), bf(forward_beta), br(reverse_beta) {}
    };

    class pnp : public Component {
    public:
        double is; 
        double bf; 
        double br; 

        pnp(std::string id, size_t collector, size_t base, size_t emitter, 
            double saturation_current = 1e-16, double forward_beta = 100.0, double reverse_beta = 1.0)
            : Component(std::move(id), {collector, base, emitter}), 
              is(saturation_current), bf(forward_beta), br(reverse_beta) {}
    };
}

} // namespace asic

#endif