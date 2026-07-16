#include <gtest/gtest.h>
#include "asic/circuit.hpp"
#include "asic/newton_raphson.hpp"
#include "asic/exceptions.hpp"

TEST(CmosLogicTest, InverterStates) {
    asic::Circuit circuit;
    size_t gnd = 0, vdd = 1, in = 2, out = 3;

    circuit.add<asic::Passives::VoltageSource>("V_DD", vdd, gnd, 1.2, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_IN", in, gnd, 0.0, asic::VoltageType::DC);

    circuit.add<asic::fet::mos>("MP1", asic::Polarity::P_TYPE, out, in, vdd, vdd, 4.0, 1.0, -0.4);
    circuit.add<asic::fet::mos>("MN1", asic::Polarity::N_TYPE, out, in, gnd, gnd, 2.0, 1.0, 0.4);

    asic::MnaMatrix sim_state(circuit);
    asic::NewtonRaphson solver;

    // Test State 0 -> 1
    circuit.update_voltage("V_IN", 0.0);
    ASSERT_TRUE(solver.solve_dc(circuit, sim_state));
    // Expect output to be very close to 1.2V
    EXPECT_NEAR(sim_state.get_node_voltage(out), 1.2, 1e-3); 

    // Test State 1 -> 0
    circuit.update_voltage("V_IN", 1.2);
    ASSERT_TRUE(solver.solve_dc(circuit, sim_state));
    // Expect output to be very close to 0.0V
    EXPECT_NEAR(sim_state.get_node_voltage(out), 0.0, 1e-3);
}

TEST(ErrorHandlingTest, CatchesSingularMatrix) {
    asic::Circuit circuit;
    size_t gnd = 0, node1 = 1;

    // Topology Error: Shorting a voltage source directly back to itself/ground
    circuit.add<asic::Passives::VoltageSource>("V_SHORT", node1, gnd, 1.2, asic::VoltageType::DC);
    circuit.add<asic::Passives::VoltageSource>("V_SHORT2", node1, gnd, 0.0, asic::VoltageType::DC);

    asic::MnaMatrix sim_state(circuit);
    asic::NewtonRaphson solver;

    // EXPECT_THROW asserts that when we call solve_dc, it MUST throw a SingularMatrixError.
    // If it doesn't throw, or throws a different error, the test fails.
    EXPECT_THROW({
        solver.solve_dc(circuit, sim_state);
    }, asic::SingularMatrixError);
}