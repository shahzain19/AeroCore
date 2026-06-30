#include "Utilities/Config.h"
#include "test_common.h"

#include <cstdio>
#include <fstream>

using AeroCore::Utilities::Config;

int main() {
    const std::string path = "tmp_test_config.toml";
    {
        std::ofstream out(path);
        out << "# comment-only line\n";
        out << "[simulation]\n";
        out << "dt = 0.004\n";
        out << "name = \"Aero Core\" # inline comment\n";
        out << "value = 42\n";
    }

    Config cfg(path);

    AeroCore::Tests::expectNear(cfg.get<double>("simulation", "dt"), 0.004, 1e-12, "parse double");
    AeroCore::Tests::expectTrue(cfg.get<int>("simulation", "value") == 42, "parse int");
    AeroCore::Tests::expectTrue(cfg.get<std::string>("simulation", "name") == "Aero Core",
                                "parse quoted string");

    cfg.set("runtime", "enabled", 1);
    AeroCore::Tests::expectTrue(cfg.get<int>("runtime", "enabled") == 1, "set/get runtime key");

    AeroCore::Tests::expectThrows(
        [&]() { (void)cfg.get<int>("missing", "key"); },
        "missing section throws");
    AeroCore::Tests::expectThrows(
        [&]() { (void)cfg.get<int>("simulation", "missing_key"); },
        "missing key throws");

    std::remove(path.c_str());
    return AeroCore::Tests::finish();
}
