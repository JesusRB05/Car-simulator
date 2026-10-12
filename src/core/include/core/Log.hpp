// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

#include <spdlog/spdlog.h>

namespace sim::core {

// Central logging facility. Loggers are named: the base application uses
// "app", subsystems use their subsystem name and mods / plugins use a
// "mod:<id>" or "plugin:<id>" name so their output is easy to spot.
class Log
{
public:
    struct Config
    {
        std::string appName = "sim";
        std::filesystem::path logFile = "sim.log";
        bool console = true;
        bool file = true;
        spdlog::level::level_enum level = spdlog::level::trace;
    };

    // Installs the default logger and returns it. Safe to call once at startup.
    static void init(const Config& config);

    // Flushes and releases all sinks. Safe to call multiple times.
    static void shutdown();

    static void setLevel(spdlog::level::level_enum level);
    static void setPattern(std::string_view pattern);

    // Returns the default (application) logger.
    static std::shared_ptr<spdlog::logger> get();

    // Returns a named logger, creating it on first use. The name is used as the
    // per-line prefix, which is how mod and plugin output is identified.
    static std::shared_ptr<spdlog::logger> get(std::string_view name);

    // Convenience helpers for the two namespaced logger families.
    static std::shared_ptr<spdlog::logger> forMod(std::string_view modId);
    static std::shared_ptr<spdlog::logger> forPlugin(std::string_view pluginId);
};

} // namespace sim::core
