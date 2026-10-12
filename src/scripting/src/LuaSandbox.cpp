// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "scripting/LuaSandbox.hpp"

#include "core/Log.hpp"

namespace sim::scripting {
namespace {

// Address used as a light-userdata key into the Lua registry.
const char kBudgetKey = 'B';
constexpr int kHookCount = 1000;

std::string popString(lua_State* state)
{
    const char* text = lua_tostring(state, -1);
    std::string result = text != nullptr ? text : "unknown Lua error";
    lua_pop(state, 1);
    return result;
}

} // namespace

LuaSandbox::LuaSandbox() : state_(luaL_newstate()), view_(state_)
{
    budgetState_ = new Budget();
    lua_pushlightuserdata(state_, const_cast<char*>(&kBudgetKey));
    lua_pushlightuserdata(state_, budgetState_);
    lua_rawset(state_, LUA_REGISTRYINDEX);

    // Only safe libraries are opened. io, os, package, debug and ffi stay closed.
    view_.open_libraries(sol::lib::base, sol::lib::coroutine, sol::lib::math, sol::lib::string, sol::lib::table);

    // Remove everything that could escape the sandbox or load bytecode.
    sol::table globals = view_.globals();
    for (const char* name : {"load", "loadstring", "loadfile", "dofile", "require", "collectgarbage", "print"})
    {
        globals[name] = sol::nil;
    }
    globals["string"]["dump"] = sol::nil;
    globals["os"] = sol::nil;
    globals["io"] = sol::nil;
    globals["package"] = sol::nil;
    globals["debug"] = sol::nil;
}

LuaSandbox::~LuaSandbox()
{
    if (state_ != nullptr)
    {
        lua_close(state_);
        state_ = nullptr;
    }
    delete budgetState_;
    budgetState_ = nullptr;
}

void LuaSandbox::instructionHook(lua_State* state, lua_Debug* /*debug*/)
{
    lua_pushlightuserdata(state, const_cast<char*>(&kBudgetKey));
    lua_rawget(state, LUA_REGISTRYINDEX);
    auto* budget = static_cast<Budget*>(lua_touserdata(state, -1));
    lua_pop(state, 1);
    if (budget == nullptr)
    {
        return;
    }
    budget->used += static_cast<std::uint64_t>(kHookCount);
    if (budget->limit != 0 && budget->used >= budget->limit)
    {
        luaL_error(state, "script exceeded its instruction budget (%d)", static_cast<int>(budget->limit));
    }
}

void LuaSandbox::log(std::string_view message) const
{
    if (logSink_)
    {
        logSink_(message);
        return;
    }
    core::Log::get("lua")->info("{}", message);
}

bool LuaSandbox::run(std::string_view code, std::string_view chunkName, std::string& error)
{
    budgetState_->limit = budget_;
    budgetState_->used = 0;
    lua_sethook(state_, &LuaSandbox::instructionHook, LUA_MASKCOUNT, kHookCount);

    // "t" mode rejects precompiled bytecode: only text chunks are accepted.
    const std::string name(chunkName);
    const int status = luaL_loadbufferx(state_, code.data(), code.size(), name.c_str(), "t");
    if (status != LUA_OK)
    {
        error = popString(state_);
        lua_sethook(state_, nullptr, 0, 0);
        return false;
    }

    sol::protected_function function = sol::stack::pop<sol::protected_function>(state_);
    sol::protected_function_result result = function();

    lua_sethook(state_, nullptr, 0, 0);

    if (!result.valid())
    {
        error = result.get<sol::error>().what();
        return false;
    }
    return true;
}

bool LuaSandbox::call(sol::protected_function& function)
{
    budgetState_->limit = budget_;
    budgetState_->used = 0;
    lua_sethook(state_, &LuaSandbox::instructionHook, LUA_MASKCOUNT, kHookCount);

    sol::protected_function_result result = function();

    lua_sethook(state_, nullptr, 0, 0);

    if (!result.valid())
    {
        core::Log::get("lua")->error("callback error: {}", result.get<sol::error>().what());
        return false;
    }
    return true;
}

bool LuaSandbox::call(sol::protected_function& function, double argument)
{
    budgetState_->limit = budget_;
    budgetState_->used = 0;
    lua_sethook(state_, &LuaSandbox::instructionHook, LUA_MASKCOUNT, kHookCount);

    sol::protected_function_result result = function(argument);

    lua_sethook(state_, nullptr, 0, 0);

    if (!result.valid())
    {
        core::Log::get("lua")->error("callback error: {}", result.get<sol::error>().what());
        return false;
    }
    return true;
}

} // namespace sim::scripting
