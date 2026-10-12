// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "core/Log.hpp"

#include <mutex>
#include <vector>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace sim::core {
namespace {

std::mutex g_logMutex;
std::vector<spdlog::sink_ptr> g_sinks;
bool g_initialised = false;

constexpr std::string_view kPattern = "%^[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v%$";

} // namespace

void Log::init(const Config& config)
{
    std::lock_guard lock(g_logMutex);
    if (g_initialised)
    {
        return;
    }

    std::vector<spdlog::sink_ptr> sinks;
    if (config.console)
    {
        auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        sinks.push_back(console);
    }
    if (config.file)
    {
        if (config.logFile.has_parent_path())
        {
            std::error_code ec;
            std::filesystem::create_directories(config.logFile.parent_path(), ec);
        }
        auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>(config.logFile.string(), true);
        sinks.push_back(file);
    }
    if (sinks.empty())
    {
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }

    g_sinks = sinks;

    auto defaultLogger = std::make_shared<spdlog::logger>(config.appName, sinks.begin(), sinks.end());
    defaultLogger->set_pattern(std::string(kPattern));
    defaultLogger->set_level(config.level);
    defaultLogger->flush_on(spdlog::level::warn);
    spdlog::set_default_logger(defaultLogger);
    spdlog::set_level(config.level);

    g_initialised = true;
    defaultLogger->info("logging initialised (level '{}', {} sink(s))", spdlog::level::to_string_view(config.level),
                        sinks.size());
}

void Log::shutdown()
{
    std::lock_guard lock(g_logMutex);
    if (!g_initialised)
    {
        return;
    }
    spdlog::shutdown();
    g_sinks.clear();
    g_initialised = false;
}

void Log::setLevel(spdlog::level::level_enum level)
{
    spdlog::set_level(level);
}

void Log::setPattern(std::string_view pattern)
{
    spdlog::set_pattern(std::string(pattern));
}

std::shared_ptr<spdlog::logger> Log::get()
{
    return spdlog::default_logger();
}

std::shared_ptr<spdlog::logger> Log::get(std::string_view name)
{
    const std::string key(name);
    if (auto existing = spdlog::get(key))
    {
        return existing;
    }

    std::lock_guard lock(g_logMutex);
    if (auto existing = spdlog::get(key))
    {
        return existing;
    }

    auto logger = std::make_shared<spdlog::logger>(key, g_sinks.begin(), g_sinks.end());
    logger->set_pattern(std::string(kPattern));
    logger->set_level(spdlog::get_level());
    logger->flush_on(spdlog::level::warn);
    spdlog::register_logger(logger);
    return logger;
}

std::shared_ptr<spdlog::logger> Log::forMod(std::string_view modId)
{
    return get("mod:" + std::string(modId));
}

std::shared_ptr<spdlog::logger> Log::forPlugin(std::string_view pluginId)
{
    return get("plugin:" + std::string(pluginId));
}

} // namespace sim::core
