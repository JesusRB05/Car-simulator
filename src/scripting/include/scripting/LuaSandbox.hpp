// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#define SOL_ALL_SAFETIES_ON 1
#include <sol/sol.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace sim::scripting {

// A locked-down Lua state. Dangerous libraries (io, os, package, debug, ffi)
// are never opened (and the LuaJIT JIT engine is disabled so the budget hook
// always fires); the base library is opened but `load`, `loadstring`,
// `dofile`, `loadfile`, `require` and `string.dump` are removed so that
// bytecode cannot be loaded. Chunks are loaded in text-only mode to reject
// precompiled bytecode outright.
//
// An instruction budget (via a Lua debug hook) aborts runaway scripts such as
// `while true do end` without freezing the host.
class LuaSandbox {
public:
    LuaSandbox();
    ~LuaSandbox();

    LuaSandbox(const LuaSandbox&) = delete;
    LuaSandbox& operator=(const LuaSandbox&) = delete;

    sol::state_view view() const { return view_; }

    void setInstructionBudget(std::uint64_t instructions) { budget_ = instructions; }
    std::uint64_t instructionBudget() const { return budget_; }

    // Per-state memory ceiling, enforced by a custom Lua allocator. A single
    // `string.rep("x", 1e9)` is one instruction, so the instruction budget alone
    // cannot stop it; the allocator refuses the block and Lua raises "not enough
    // memory". Zero disables the limit.
    void setMemoryLimit(std::size_t bytes) { allocator_->limit = bytes; }
    std::size_t memoryLimit() const { return allocator_->limit; }
    std::size_t memoryUsed() const { return allocator_->used; }

    // Loads and runs a text chunk. Returns false and fills `error` on failure.
    bool run(std::string_view code, std::string_view chunkName, std::string& error);

    // Calls a Lua function with an optional numeric argument under the
    // instruction budget. Returns false and logs on failure.
    bool call(sol::protected_function& function);
    bool call(sol::protected_function& function, double argument);

    // Log sink used by the `sim.log` binding installed by ScriptHost.
    void setLogSink(std::function<void(std::string_view)> sink) { logSink_ = std::move(sink); }
    void log(std::string_view message) const;

private:
    struct Budget {
        std::uint64_t limit = 0;
        std::uint64_t used = 0;
        // Set once the budget is exhausted: the hook then raises on every single
        // instruction, so `pcall` cannot swallow the error (its own handler
        // instructions re-enter the hook and re-raise) and it reaches the C
        // boundary. `while true do pcall(function() while true do end end) end`
        // therefore aborts instead of spinning forever.
        bool aborted = false;
    };

    // Accounting for the custom allocator: the live byte total and the ceiling.
    struct Allocator {
        std::size_t used = 0;
        std::size_t limit = 0;
    };

    // Owns the raw Lua state and closes it on destruction. Declared before
    // `view_` so that the sol2 view (which holds registry references) is
    // destroyed first; closing the state earlier crashes on unref.
    struct StateOwner {
        lua_State* state = nullptr;
        explicit StateOwner(lua_State* s) : state(s) {}
        ~StateOwner() {
            if (state != nullptr) {
                lua_close(state);
            }
        }
        StateOwner(const StateOwner&) = delete;
        StateOwner& operator=(const StateOwner&) = delete;
    };

    static void* allocate(void* userdata, void* pointer, std::size_t oldSize, std::size_t newSize);
    static void instructionHook(lua_State* state, lua_Debug* debug);

    // Declared (and constructed) before owner_: lua_newstate calls the allocator
    // immediately, so the accounting struct must already exist. It is a smart
    // pointer so that it is destroyed *after* owner_ (which closes the state and
    // keeps calling the allocator): the ceiling must outlive the Lua state.
    std::unique_ptr<Allocator> allocator_;
    StateOwner owner_;
    lua_State* state_ = nullptr;
    sol::state_view view_;
    Budget* budgetState_ = nullptr;
    std::uint64_t budget_ = 5'000'000;
    std::function<void(std::string_view)> logSink_;
};

} // namespace sim::scripting
