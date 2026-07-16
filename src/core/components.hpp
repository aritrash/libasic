#ifndef LIBASIC_COMPONENTS_HPP
#define LIBASIC_COMPONENTS_HPP

#include <string>
#include <vector>
#include <cmath>

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

    class PulseSource : public Component {
    public:
        double v_initial;
        double v_peak;
        double t_delay;
        double t_rise;
        double t_fall;
        double t_pulse_width;
        double t_period;

        PulseSource(std::string id, size_t pos_node, size_t neg_node, 
                    double v1, double v2, double td, double tr, double tf, double pw, double per)
            : Component(std::move(id), {pos_node, neg_node}), 
              v_initial(v1), v_peak(v2), t_delay(td), t_rise(tr), t_fall(tf), t_pulse_width(pw), t_period(per) {}

        // Evaluates the piecewise linear waveform at a specific nanosecond
        double get_voltage(double t) const {
            if (t < t_delay) return v_initial;
            
            double t_active = std::fmod(t - t_delay, t_period);
            
            if (t_active < t_rise) {
                return v_initial + (v_peak - v_initial) * (t_active / t_rise);
            } else if (t_active < t_rise + t_pulse_width) {
                return v_peak;
            } else if (t_active < t_rise + t_pulse_width + t_fall) {
                return v_peak - (v_peak - v_initial) * ((t_active - t_rise - t_pulse_width) / t_fall);
            } else {
                return v_initial;
            }
        }
    };

    class TernaryClock : public Component {
    public:
        double v_neg, v_mid, v_pos;
        double t_step_duration;

        TernaryClock(std::string id, size_t pos_node, size_t neg_node, 
                     double vn, double vm, double vp, double duration)
            : Component(std::move(id), {pos_node, neg_node}), 
              v_neg(vn), v_mid(vm), v_pos(vp), t_step_duration(duration) {}

        // Generates a staircase waveform: Neg -> Mid -> Pos -> Mid -> Repeat
        double get_voltage(double t) const {
            int step = static_cast<int>(t / t_step_duration) % 4;
            switch(step) {
                case 0: return v_neg;
                case 1: return v_mid;
                case 2: return v_pos;
                case 3: return v_mid;
                default: return v_mid;
            }
        }
    };

    class Memristor : public Component {
    public:
        double r_on;
        double r_off;
        double mobility; // How fast the state changes
        double w;        // Internal state: 1.0 = Fully ON (LRS), 0.0 = Fully OFF (HRS)

        // Only declare the signatures here
        Memristor(const std::string& n, size_t pos, size_t neg, double ron, double roff, double init_w, double mu = 1e-14);
        
        double get_conductance() const;
        void step_time(double v_drop, double dt);
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

    class tgate : public Component {
    public:
        double width_n, width_p;      
        double length;
        double v_threshold_n, v_threshold_p; 
        double gamma;       
        double phi;         

        tgate(std::string id, size_t input, size_t output, 
              size_t ctrl_n, size_t ctrl_p, size_t bulk_n, size_t bulk_p,
              double wn = 1.0, double wp = 2.0, double l = 1.0, 
              double vtn = 0.4, double vtp = 0.4, double g = 0.4, double ph = 0.65)
            : Component(std::move(id), {input, output, ctrl_n, ctrl_p, bulk_n, bulk_p}), 
              width_n(wn), width_p(wp), length(l), 
              v_threshold_n(vtn), v_threshold_p(vtp), gamma(g), phi(ph) {}
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