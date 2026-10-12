// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "core/Config.hpp"

#include <fstream>
#include <utility>

#include "core/Log.hpp"

namespace sim::core {
namespace {

std::vector<std::string> splitKey(std::string_view key)
{
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (start <= key.size())
    {
        const std::size_t dot = key.find('.', start);
        const std::size_t end = (dot == std::string_view::npos) ? key.size() : dot;
        if (end > start)
        {
            parts.emplace_back(key.substr(start, end - start));
        }
        if (dot == std::string_view::npos)
        {
            break;
        }
        start = dot + 1;
    }
    return parts;
}

} // namespace

Config::Config(std::filesystem::path file) : file_(std::move(file)) {}

bool Config::load()
{
    std::error_code ec;
    if (!std::filesystem::exists(file_, ec))
    {
        table_ = toml::table{};
        return true;
    }

    try
    {
        table_ = toml::parse_file(file_.string());
        return true;
    }
    catch (const toml::parse_error& error)
    {
        Log::get("config")->error("failed to parse '{}': {}", file_.string(), error.description());
        return false;
    }
}

bool Config::save() const
{
    std::error_code ec;
    if (file_.has_parent_path())
    {
        std::filesystem::create_directories(file_.parent_path(), ec);
    }

    std::ofstream out(file_);
    if (!out)
    {
        Log::get("config")->error("cannot write config file '{}'", file_.string());
        return false;
    }

    out << "# Car Simulator user configuration. Editing this file by hand is supported.\n\n";
    out << table_ << '\n';
    return static_cast<bool>(out);
}

const toml::node* Config::find(std::string_view dottedKey) const
{
    const toml::table* current = &table_;
    const auto parts = splitKey(dottedKey);
    for (std::size_t i = 0; i < parts.size(); ++i)
    {
        const toml::node* node = current->get(parts[i]);
        if (node == nullptr)
        {
            return nullptr;
        }
        if (i + 1 == parts.size())
        {
            return node;
        }
        current = node->as_table();
        if (current == nullptr)
        {
            return nullptr;
        }
    }
    return nullptr;
}

toml::node* Config::find(std::string_view dottedKey)
{
    return const_cast<toml::node*>(std::as_const(*this).find(dottedKey));
}

toml::table& Config::ensureTable(std::string_view dottedPath)
{
    toml::table* current = &table_;
    for (const auto& part : splitKey(dottedPath))
    {
        toml::node* node = current->get(part);
        if (node == nullptr || !node->is_table())
        {
            auto [it, inserted] = current->insert_or_assign(part, toml::table{});
            (void)inserted;
            node = &it->second;
        }
        current = node->as_table();
    }
    return *current;
}

bool Config::contains(std::string_view dottedKey) const
{
    return find(dottedKey) != nullptr;
}

std::string Config::getString(std::string_view key, std::string defaultValue) const
{
    if (const toml::node* node = find(key))
    {
        if (auto value = node->value<std::string>())
        {
            return *value;
        }
    }
    return defaultValue;
}

std::int64_t Config::getInt(std::string_view key, std::int64_t defaultValue) const
{
    if (const toml::node* node = find(key))
    {
        if (auto value = node->value<std::int64_t>())
        {
            return *value;
        }
    }
    return defaultValue;
}

double Config::getDouble(std::string_view key, double defaultValue) const
{
    if (const toml::node* node = find(key))
    {
        if (auto value = node->value<double>())
        {
            return *value;
        }
        if (auto value = node->value<std::int64_t>())
        {
            return static_cast<double>(*value);
        }
    }
    return defaultValue;
}

bool Config::getBool(std::string_view key, bool defaultValue) const
{
    if (const toml::node* node = find(key))
    {
        if (auto value = node->value<bool>())
        {
            return *value;
        }
    }
    return defaultValue;
}

std::vector<std::string> Config::getStringArray(std::string_view key, std::vector<std::string> defaultValue) const
{
    if (const toml::node* node = find(key))
    {
        if (const toml::array* array = node->as_array())
        {
            std::vector<std::string> result;
            result.reserve(array->size());
            for (const toml::node& element : *array)
            {
                result.push_back(element.value<std::string>().value_or(std::string{}));
            }
            return result;
        }
    }
    return defaultValue;
}

void Config::setString(std::string_view key, std::string_view value)
{
    const auto parts = splitKey(key);
    toml::table& parent = ensureTable(key.substr(0, key.size() - parts.back().size()));
    parent.insert_or_assign(parts.back(), std::string(value));
}

void Config::setInt(std::string_view key, std::int64_t value)
{
    const auto parts = splitKey(key);
    toml::table& parent = ensureTable(key.substr(0, key.size() - parts.back().size()));
    parent.insert_or_assign(parts.back(), value);
}

void Config::setDouble(std::string_view key, double value)
{
    const auto parts = splitKey(key);
    toml::table& parent = ensureTable(key.substr(0, key.size() - parts.back().size()));
    parent.insert_or_assign(parts.back(), value);
}

void Config::setBool(std::string_view key, bool value)
{
    const auto parts = splitKey(key);
    toml::table& parent = ensureTable(key.substr(0, key.size() - parts.back().size()));
    parent.insert_or_assign(parts.back(), value);
}

void Config::setStringArray(std::string_view key, const std::vector<std::string>& value)
{
    toml::array array;
    for (const auto& item : value)
    {
        array.push_back(item);
    }
    const auto parts = splitKey(key);
    toml::table& parent = ensureTable(key.substr(0, key.size() - parts.back().size()));
    parent.insert_or_assign(parts.back(), std::move(array));
}

void Config::erase(std::string_view key)
{
    const auto parts = splitKey(key);
    if (parts.empty())
    {
        return;
    }
    const std::size_t parentLength = key.size() - parts.back().size();
    const std::string_view parentPath = key.substr(0, parentLength);
    if (toml::node* parentNode = find(parentPath))
    {
        if (toml::table* parent = parentNode->as_table())
        {
            parent->erase(parts.back());
        }
    }
}

} // namespace sim::core
