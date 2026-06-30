#include "Utilities/CliArgs.h"
#include "test_common.h"

using AeroCore::Utilities::CliArgs;

int main() {
    {
        std::string err;
        const char* argv[] = {"AeroCore", "--help"};
        const auto args = CliArgs::parse(2, const_cast<char**>(argv), err);
        AeroCore::Tests::expectTrue(args.show_help, "--help sets show_help");
        AeroCore::Tests::expectTrue(err.empty(), "--help produces no error");
    }

    {
        std::string err;
        const char* argv[] = {"AeroCore", "--headless", "--duration", "12.5",
                              "config/fixed_wing.toml"};
        const auto args = CliArgs::parse(5, const_cast<char**>(argv), err);
        AeroCore::Tests::expectTrue(args.headless, "parses --headless");
        AeroCore::Tests::expectNear(args.max_sim_time, 12.5, 1e-9, "parses --duration");
        AeroCore::Tests::expectTrue(args.config_path == "config/fixed_wing.toml",
                                    "positional config path");
        AeroCore::Tests::expectTrue(err.empty(), "valid args produce no error");
    }

    {
        std::string err;
        const char* argv[] = {"AeroCore", "--duration"};
        const auto args = CliArgs::parse(2, const_cast<char**>(argv), err);
        AeroCore::Tests::expectTrue(!err.empty(), "missing --duration value errors");
        (void)args;
    }

    {
        std::string err;
        const char* argv[] = {"AeroCore", "--unknown-flag"};
        const auto args = CliArgs::parse(2, const_cast<char**>(argv), err);
        AeroCore::Tests::expectTrue(!err.empty(), "unknown flag errors");
        (void)args;
    }

    const auto help = CliArgs::helpText();
    AeroCore::Tests::expectTrue(help.find("--headless") != std::string::npos,
                                "help mentions --headless");
    AeroCore::Tests::expectTrue(help.find("--help") != std::string::npos,
                                "help mentions --help");

    return AeroCore::Tests::finish();
}
