// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "scripting/LuaSandbox.hpp"

#include "core/Log.hpp"

#include <luajit.h>

#include <cstdlib>

namespace sim::scripting {
namespace {

// Address used as a light-userdata key into the Lua registry.
const char kBudgetKey = 'B';
constexpr int kHookCount = 1000;

std::string popString(lua_State* state) {
    const char* text = lua_tostring(state, -1);
    std::string result = text != nullptr ? text : "unknown Lua error";
    lua_pop(state, 1);
    return result;
}

} // namespace

LuaSandbox::LuaSandbox()
    : allocator_(std::make_unique<Allocator>()),
      owner_(lua_newstate(&LuaSandbox::allocate, allocator_.get())),
      state_(owner_.state),
      view_(state_) {
    // The allocator exists before lua_newstate runs; the ceiling is applied once
    // the state is up (the initial allocation is tiny).
    allocator_->limit = 64u * 1024u * 1024u;

    budgetState_ = new Budget();
    lua_pushlightuserdata(state_, const_cast<char*>(&kBudgetKey));
    lua_pushlightuserdata(state_, budgetState_);
    lua_rawset(state_, LUA_REGISTRYINDEX);

    // LuaJIT does not run debug hooks inside JIT-compiled traces, so a hot loop
    // such as `while true do end` would escape the instruction budget once it
    // is compiled. Turn the JIT engine off for this state before any script is
    // loaded: everything runs in the interpreter, where the count hook fires.
    luaJIT_setmode(state_, 0, LUAJIT_MODE_ENGINE | LUAJIT_MODE_OFF);

    // Only safe libraries are opened. io, os, package, debug, ffi and jit stay
    // closed.
    view_.open_libraries(sol::lib::base, sol::lib::coroutine, sol::lib::math, sol::lib::string, sol::lib::table);

    // Remove everything that could escape the sandbox or load bytecode, and the
    // environment-manipulation helpers (setfenv/getfenv/newproxy) that let a
    // chunk swap its own environment for another table.
    sol::table globals = view_.globals();
    for (const char* name : {"load", "loadstring", "loadfile", "dofile", "require", "collectgarbage", "print",
                             "setfenv", "getfenv", "newproxy"}) {
        globals[name] = sol::nil;
    }
    globals["string"]["dump"] = sol::nil;
    globals["os"] = sol::nil;
    globals["io"] = sol::nil;
    globals["package"] = sol::nil;
    globals["debug"] = sol::nil;
    globals["ffi"] = sol::nil;
    globals["jit"] = sol::nil;
}

LuaSandbox::~LuaSandbox() {
    // The Lua state itself is closed by owner_ after view_ is destroyed;
    // allocator_ (a member declared before owner_) is destroyed last, so the
    // ceiling is still valid while lua_close calls the allocator.
    delete budgetState_;
    budgetState_ = nullptr;
}

// Custom allocator that enforces the per-state memory ceiling. Returning nullptr
// makes Lua raise "not enough memory" instead of reserving gigabytes.
void* LuaSandbox::allocate(void* userdata, void* pointer, std::size_t oldSize, std::size_t newSize) {
    auto* allocator = static_cast<Allocator*>(userdata);
    if (newSize == 0) {
        std::free(pointer);
        if (pointer != nullptr && oldSize <= allocator->used) {
            allocator->used -= oldSize;
        }
        return nullptr;
    }

    const std::size_t current =
        (pointer != nullptr && oldSize <= allocator->used) ? allocator->used - oldSize : allocator->used;
    const std::size_t next = current + newSize;
    if (allocator->limit != 0 && next > allocator->limit) {
        return nullptr;
    }

    void* block = std::realloc(pointer, newSize);
    if (block != nullptr) {
        allocator->used = next;
    }
    return block;
}

void LuaSandbox::instructionHook(lua_State* state, lua_Debug* /*debug*/) {
    lua_pushlightuserdata(state, const_cast<char*>(&kBudgetKey));
    lua_rawget(state, LUA_REGISTRYINDEX);
    auto* budget = static_cast<Budget*>(lua_touserdata(state, -1));
    lua_pop(state, 1);
    if (budget == nullptr) {
        return;
    }

    // Once the budget is spent, raise on every instruction. `pcall` normally
    // swallows a hook error and lets the caller loop again, which is how
    // `while true do pcall(function() while true do end end) end` froze the
    // host; with the hook firing on every instruction its own error handling
    // re-enters the hook and re-raises, so the error reaches the C boundary.
    if (budget->aborted) {
        luaL_error(state, "script exceeded its instruction budget (%d)", static_cast<int>(budget->limit));
    }

    budget->used += static_cast<std::uint64_t>(kHookCount);
    if (budget->limit != 0 && budget->used >= budget->limit) {
        budget->aborted = true;
        lua_sethook(state, &LuaSandbox::instructionHook, LUA_MASKCOUNT, 1);
        luaL_error(state, "script exceeded its instruction budget (%d)", static_cast<int>(budget->limit));
    }
}

void LuaSandbox::log(std::string_view message) const {
    if (logSink_) {
        logSink_(message);
        return;
    }
    core::Log::get("lua")->info("{}", message);
}

bool LuaSandbox::run(std::string_view code, std::string_view chunkName, std::string& error) {
    budgetState_->limit = budget_;
    budgetState_->used = 0;
    budgetState_->aborted = false;
    lua_sethook(state_, &LuaSandbox::instructionHook, LUA_MASKCOUNT, kHookCount);

    // "t" mode rejects precompiled bytecode: only text chunks are accepted.
    const std::string name(chunkName);
    const int status = luaL_loadbufferx(state_, code.data(), code.size(), name.c_str(), "t");
    if (status != LUA_OK) {
        error = popString(state_);
        lua_sethook(state_, nullptr, 0, 0);
        return false;
    }

    sol::protected_function function = sol::stack::pop<sol::protected_function>(state_);
    sol::protected_function_result result = function();

    lua_sethook(state_, nullptr, 0, 0);

    if (!result.valid()) {
        error = result.get<sol::error>().what();
        return false;
    }
    return true;
}

bool LuaSandbox::call(sol::protected_function& function) {
    budgetState_->limit = budget_;
    budgetState_->used = 0;
    budgetState_->aborted = false;
    lua_sethook(state_, &LuaSandbox::instructionHook, LUA_MASKCOUNT, kHookCount);

    sol::protected_function_result result = function();

    lua_sethook(state_, nullptr, 0, 0);

    if (!result.valid()) {
        core::Log::get("lua")->error("callback error: {}", result.get<sol::error>().what());
        return false;
    }
    return true;
}

bool LuaSandbox::call(sol::protected_function& function, double argument) {
    budgetState_->limit = budget_;
    budgetState_->used = 0;
    budgetState_->aborted = false;
    lua_sethook(state_, &LuaSandbox::instructionHook, LUA_MASKCOUNT, kHookCount);

    sol::protected_function_result result = function(argument);

    lua_sethook(state_, nullptr, 0, 0);

    if (!result.valid()) {
        core::Log::get("lua")->error("callback error: {}", result.get<sol::error>().what());
        return false;
    }
    return true;
}

} // namespace sim::scripting
