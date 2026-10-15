#include <fr3_stack/controllers/cart_profile.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace {

[[noreturn]] void fail(const std::string& path, const std::string& msg) {
    throw std::runtime_error("cart profile '" + path + "': " + msg);
}

template <int N>
void read_vec(const YAML::Node& doc, const char* key, const std::string& path,
              Eigen::Matrix<double, N, 1>& out) {
    const YAML::Node v = doc[key];
    if (!v) return;
    if (!v.IsSequence() || v.size() != size_t(N))
        fail(path, std::string(key) + " must be a list of " + std::to_string(N));
    for (int i = 0; i < N; ++i) out[i] = v[i].as<double>();
    if (!out.allFinite()) fail(path, std::string(key) + " must be finite");
}

void read_scalar(const YAML::Node& doc, const char* key, const std::string& path,
                 double& out) {
    if (!doc[key]) return;
    out = doc[key].as<double>();
    if (!std::isfinite(out)) fail(path, std::string(key) + " must be finite");
}

void read_bool(const YAML::Node& doc, const char* key, bool& out) {
    if (doc[key]) out = doc[key].as<bool>();
}

}  // namespace

CartesianImpedanceCfg load_cart_impedance_profile(const std::string& path) {
    YAML::Node doc;
    try {
        doc = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        fail(path, e.what());
    }
    if (!doc.IsMap()) fail(path, "top level must be a mapping");

    CartesianImpedanceCfg c;
    try {
        read_vec<6>(doc, "K", path, c.K);
        read_vec<6>(doc, "D", path, c.D);
        read_vec<7>(doc, "q_null", path, c.q_null);
        read_vec<6>(doc, "max_delta", path, c.max_delta);
        read_scalar(doc, "K_null", path, c.K_null);
        read_scalar(doc, "D_null", path, c.D_null);
        read_scalar(doc, "max_tau_null", path, c.max_tau_null);
        read_scalar(doc, "filter_alpha", path, c.filter_alpha);
        read_bool(doc, "linear_interp", c.linear_interp);
        read_bool(doc, "ema", c.ema);
        read_bool(doc, "use_friction", c.use_friction);
    } catch (const YAML::Exception& e) {
        fail(path, e.what());
    }
    if ((c.K.array() <= 0.0).any()) fail(path, "K must be > 0");
    if ((c.D.array() < 0.0).any() || (c.max_delta.array() < 0.0).any())
        fail(path, "D and max_delta must be >= 0");
    if (c.K_null < 0.0 || c.D_null < 0.0 || c.max_tau_null < 0.0)
        fail(path, "K_null, D_null and max_tau_null must be >= 0");
    if (!(c.filter_alpha > 0.0 && c.filter_alpha <= 1.0))
        fail(path, "filter_alpha must be in (0, 1]");
    return c;
}
