// Load initial Cartesian-impedance gains from a controller profile YAML.
//
// Reads the same file format as the Python SDK profiles
// (fr3_stack/configs/cartesian_impedance[.<profile>].yaml): K, D, q_null,
// K_null, D_null, max_tau_null, filter_alpha, linear_interp, ema, max_delta,
// use_friction. Keys that are absent keep their CartesianImpedanceCfg default.
// target and target_wrench are not read: the daemon anchors the boot target at
// the live pose and never starts with a feed-forward wrench.
//
// The daemon uses this for --initial-cart-profile, so a freshly started
// cartesian_impedance daemon holds with the same gains a client would send.

#pragma once

#include <fr3_stack/controllers/controller_base.hpp>

#include <string>

// Throws std::runtime_error on a missing/unparsable file or invalid values.
CartesianImpedanceCfg load_cart_impedance_profile(const std::string& path);
