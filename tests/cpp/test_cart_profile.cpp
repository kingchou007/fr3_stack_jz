// load_cart_impedance_profile: the daemon's --initial-cart-profile must read
// the SDK's Cartesian profile YAMLs exactly as the Python client sends them.
//
// Build manually (franka mock + yaml-cpp):
//   g++ -std=c++17 -I tests/cpp/franka_mock -I include -I /usr/include/eigen3
//       -DFR3_SOURCE_DIR='"'$PWD'"' tests/cpp/test_cart_profile.cpp
//       src/controllers/cart_profile.cpp -lyaml-cpp -o /tmp/test_cart_profile

#include <fr3_stack/controllers/cart_profile.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef FR3_SOURCE_DIR
#error "FR3_SOURCE_DIR must point at the repository root"
#endif

static int g_fail = 0;
#define CHECK(cond, msg) do {                                   \
    if (cond) std::cout << "  PASS: " << msg << "\n";           \
    else { std::cout << "  FAIL: " << msg << "\n"; ++g_fail; }  \
} while (0)

static std::string profile(const char* name) {
    return std::string(FR3_SOURCE_DIR) + "/fr3_stack/configs/" + name;
}

static bool throws(const std::string& yaml) {
    const std::string path = "/tmp/fr3_test_cart_profile.yaml";
    std::ofstream(path) << yaml;
    try { load_cart_impedance_profile(path); } catch (const std::runtime_error&) {
        std::remove(path.c_str());
        return true;
    }
    std::remove(path.c_str());
    return false;
}

int main() {
    const CartesianImpedanceCfg def = load_cart_impedance_profile(
        profile("cartesian_impedance.yaml"));
    const CartesianImpedanceCfg builtin;
    CHECK(def.K.isApprox(builtin.K) && def.D.isApprox(builtin.D)
          && def.K_null == builtin.K_null && def.filter_alpha == builtin.filter_alpha,
          "default profile matches the daemon's built-in defaults");

    const CartesianImpedanceCfg s = load_cart_impedance_profile(
        profile("cartesian_impedance.sysid_stiff.yaml"));
    CHECK(s.K.head<3>().isConstant(500.0) && s.K.tail<3>().isConstant(20.0),
          "sysid_stiff K = 500 translation / 20 rotation");
    CHECK(std::abs(s.D[0] - 2.0 * std::sqrt(500.0)) < 1e-6 && s.D.tail<3>().isConstant(5.0),
          "sysid_stiff D = 2 sqrt(500) translation / 5 rotation");
    CHECK(s.K_null == 100.0 && s.ema && s.linear_interp && s.filter_alpha == 0.05,
          "sysid_stiff keeps nullspace and target smoothing");
    CHECK(s.target_wrench.isZero(), "target_wrench is never loaded");

    bool missing = false;
    try { load_cart_impedance_profile("/nonexistent/profile.yaml"); }
    catch (const std::runtime_error&) { missing = true; }
    CHECK(missing, "missing file throws");
    CHECK(throws("K: [1, 2, 3]\n"), "wrong K length throws");
    CHECK(throws("K: [0, 200, 200, 20, 20, 20]\n"), "non-positive K throws");
    CHECK(throws("filter_alpha: 0\n"), "filter_alpha 0 throws");
    CHECK(throws("D: [1, 1, 1, 1, 1, -1]\n"), "negative D throws");
    CHECK(!throws("K: [300, 300, 300, 25, 25, 25]\n"), "partial profile keeps other defaults");

    std::cout << (g_fail ? "FAILED\n" : "all passed\n");
    return g_fail ? 1 : 0;
}
