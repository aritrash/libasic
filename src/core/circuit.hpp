#ifndef LIBANA_CIRCUIT_HPP
#define LIBANA_CIRCUIT_HPP

#include "core/components.hpp"
#include <memory>
#include <vector>
#include <ostream>

namespace ana {

class Circuit {
private:
    size_t max_node_id = 0;
    std::vector<std::unique_ptr<Component>> components;

    // Helper to dynamically track the matrix dimensions needed
    void update_node_bounds(const std::vector<size_t>& connected_nodes);

public:
    Circuit() = default;

    // Node Factories
    size_t get_gnd() const { return 0; }
    
    // Core Graph Mutators
    void add_resistor(std::string id, size_t node_a, size_t node_b, double resistance);
    
    void add_voltage_source(std::string id, size_t pos_node, size_t neg_node, 
                            double value, VoltageType type = VoltageType::DC);
                            
    void add_transistor(std::string id, std::string model_type, 
                        size_t t1, size_t t2, size_t t3, 
                        double w = 1.0, double l = 1.0, double vth = 0.4);

    // Metadata Accessors for the Solver Engine
    size_t num_nodes() const { return max_node_id; }
    const std::vector<std::unique_ptr<Component>>& get_components() const { return components; }

    // Export Compilers
    void export_spice(std::ostream& os) const;
};

} // namespace ana

#endif // LIBANA_CIRCUIT_HPP