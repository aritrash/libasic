#ifndef LIBASIC_CIRCUIT_HPP
#define LIBASIC_CIRCUIT_HPP

#include "core/components.hpp"
#include <memory>
#include <vector>
#include <ostream>

namespace asic {

class Circuit {
private:
    size_t max_node_id = 0;
    std::vector<std::unique_ptr<Component>> components;
    void update_node_bounds(const std::vector<size_t>& connected_nodes);

public:
    Circuit() = default;
    size_t get_gnd() const { return 0; }
    
    // Core insertion method via move semantics
    template<typename T, typename... Args>
    void add(Args&&... args) {
        auto comp = std::make_unique<T>(std::forward<Args>(args)...);
        update_node_bounds(comp->nodes);
        components.push_back(std::move(comp));
    }

    size_t num_nodes() const { return max_node_id; }
    const std::vector<std::unique_ptr<Component>>& get_components() const { return components; }
    void export_spice(std::ostream& os) const;
    void update_voltage(const std::string& name, double new_value);
};

} // namespace asic

#endif