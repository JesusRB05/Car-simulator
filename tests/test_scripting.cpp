// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "scripting/LuaSandbox.hpp"
#include "scripting/ScriptHost.hpp"
#include "vfs/VirtualFileSystem.hpp"

using sim::scripting::LuaSandbox;

namespace {

// Runs `code` in a fresh sandbox and returns whether it succeeded.
bool runInSandbox(const std::string& code, std::string& error) {
    LuaSandbox sandbox;
    return sandbox.run(code, "test", error);
}

void writeFile(const std::filesystem::path& path, const std::string& contents) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    out << contents;
}

} // namespace

TEST_CASE("Lua sandbox hides dangerous libraries and loaders") {
    // Every name must be unreachable both as a global and through _G/getfenv.
    const char* names[] = {"io",         "os",     "ffi",      "jit",     "debug",   "package", "load",
                           "loadstring", "dofile", "loadfile", "require", "setfenv", "getfenv", "newproxy"};
    for (const char* name : names) {
        CAPTURE(name);
        const std::string code = std::string("local n = '") + name +
                                 "'\n"
                                 "assert(_G[n] == nil, n .. ' reachable via _G')\n"
                                 "assert(getfenv == nil or getfenv(0)[n] == nil, n .. ' reachable via getfenv')\n"
                                 "assert(getfenv == nil or getfenv(1)[n] == nil, n .. ' reachable via getfenv(1)')\n";
        std::string error;
        CHECK_MESSAGE(runInSandbox(code, error), error);
    }

    std::string error;
    CHECK_MESSAGE(runInSandbox("assert(string.dump == nil)\n"
                               "assert(getmetatable('').__index.dump == nil)\n"
                               "assert(('').dump == nil)\n",
                               error),
                  error);

    // Calling a removed function must fail rather than escape.
    CHECK_FALSE(runInSandbox("io.open('x', 'w')", error));
    CHECK_FALSE(runInSandbox("os.exit(1)", error));
    CHECK_FALSE(runInSandbox("require('ffi')", error));
    CHECK_FALSE(runInSandbox("loadstring('return 1')()", error));
}

TEST_CASE("Lua sandbox rejects precompiled bytecode") {
    LuaSandbox sandbox;
    std::string error;
    // A LuaJIT bytecode header followed by junk: text-only mode must refuse it
    // before the parser ever looks at the payload.
    const std::string bytecode = std::string("\x1bLJ\x02\x02", 5) + std::string(32, '\0');
    CHECK_FALSE(sandbox.run(bytecode, "bytecode", error));
    CHECK_FALSE(error.empty());
}

TEST_CASE("Lua sandbox instruction budget interrupts infinite loops") {
    LuaSandbox sandbox;
    sandbox.setInstructionBudget(1'000'000);

    const auto start = std::chrono::steady_clock::now();
    std::string error;
    CHECK_FALSE(sandbox.run("while true do end", "loop", error));
    CHECK(error.find("instruction budget") != std::string::npos);

    // Numeric for loops are the first thing LuaJIT would compile into a trace;
    // with the JIT engine off the hook must still fire.
    error.clear();
    CHECK_FALSE(sandbox.run("for i = 1, 1e15 do end", "for", error));
    CHECK(error.find("instruction budget") != std::string::npos);

    // The sandbox stays usable after an aborted script.
    error.clear();
    CHECK_MESSAGE(sandbox.run("local x = 1 + 1", "ok", error), error);

    const auto elapsed = std::chrono::steady_clock::now() - start;
    CHECK(elapsed < std::chrono::seconds(10));
}

TEST_CASE("VFS: a higher-priority mod overrides base content") {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "sim_tests_vfs_override";
    fs::remove_all(root);
    writeFile(root / "base" / "cars" / "car.json", "base");
    writeFile(root / "base" / "only_base.txt", "untouched");
    writeFile(root / "mod" / "cars" / "car.json", "mod");

    {
        sim::vfs::VirtualFileSystem vfs;
        REQUIRE(vfs.mount(root / "base"));
        REQUIRE(vfs.mount(root / "mod")); // later mount = higher priority

        const auto car = vfs.readText("cars/car.json");
        REQUIRE(car.has_value());
        CHECK(*car == "mod");

        const auto onlyBase = vfs.readText("only_base.txt");
        REQUIRE(onlyBase.has_value());
        CHECK(*onlyBase == "untouched");

        // Unmounting the mod restores the base file.
        REQUIRE(vfs.unmount(root / "mod"));
        const auto restored = vfs.readText("cars/car.json");
        REQUIRE(restored.has_value());
        CHECK(*restored == "base");
    }

    std::error_code ignored;
    fs::remove_all(root, ignored);
}

TEST_CASE("ScriptHost exposes the documented sim API and runs an entry script") {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "sim_tests_script_host";
    fs::remove_all(root);
    // Same shape as content/base/apps/main.lua: uses every sim.* entry point.
    writeFile(root / "scripts" / "main.lua", "sim.log('loaded ' .. sim.game_version)\n"
                                             "assert(type(sim.read) == 'function')\n"
                                             "assert(sim.read('scripts/main.lua') ~= nil)\n"
                                             "loaded = 0; updates = 0\n"
                                             "sim.register_on_load(function() loaded = loaded + 1 end)\n"
                                             "sim.register_on_update(function(dt) updates = updates + dt end)\n");

    {
        sim::vfs::VirtualFileSystem vfs;
        REQUIRE(vfs.mount(root));
        sim::scripting::ScriptHost host(vfs);

        REQUIRE(host.loadScript("test", "scripts/main.lua"));
        host.callOnLoad();
        host.callOnUpdate(0.25);
        host.callOnUpdate(0.25);

        LuaSandbox* sandbox = host.sandbox("test");
        REQUIRE(sandbox != nullptr);
        std::string error;
        CHECK(sandbox->run("assert(loaded == 1 and updates == 0.5)", "check", error));
        CHECK(error.empty());
    }

    std::error_code ignored;
    fs::remove_all(root, ignored);
}

TEST_CASE("Lua sandbox budget aborts a pcall-wrapped infinite loop") {
    LuaSandbox sandbox;
    sandbox.setInstructionBudget(1'000'000);

    const auto start = std::chrono::steady_clock::now();
    std::string error;
    // `pcall` swallows the first hook error; the hook must keep raising (on every
    // instruction) so the error escapes the outer loop instead of spinning.
    CHECK_FALSE(sandbox.run("while true do pcall(function() while true do end end) end", "pcall_loop", error));
    CHECK(error.find("instruction budget") != std::string::npos);

    // The sandbox stays usable afterwards.
    error.clear();
    CHECK_MESSAGE(sandbox.run("local x = 1 + 1", "ok", error), error);

    const auto elapsed = std::chrono::steady_clock::now() - start;
    CHECK(elapsed < std::chrono::seconds(10));
}

TEST_CASE("Lua sandbox enforces a per-state memory ceiling") {
    LuaSandbox sandbox;
    sandbox.setMemoryLimit(4u * 1024u * 1024u);

    std::string error;
    // A single string.rep is one instruction, so only the allocator can stop it.
    CHECK_FALSE(sandbox.run("local s = string.rep('x', 64 * 1024 * 1024)", "big", error));
    CHECK(error.find("memory") != std::string::npos);
    CHECK(sandbox.memoryUsed() <= sandbox.memoryLimit());

    // The state survives the refused allocation and stays usable.
    error.clear();
    CHECK_MESSAGE(sandbox.run("local t = {} for i = 1, 1000 do t[i] = i * i end", "ok", error), error);

    // A tighter ceiling can also be raised back: the allocator is a live limit.
    sandbox.setMemoryLimit(64u * 1024u * 1024u);
    error.clear();
    CHECK_MESSAGE(sandbox.run("local s = string.rep('y', 8 * 1024 * 1024)", "grow", error), error);
}

TEST_CASE("ScriptHost keeps one Lua state per mod") {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "sim_tests_script_host_isolation";
    fs::remove_all(root);
    // Two mods that both define a global named `shared` with different values.
    writeFile(root / "a" / "main.lua", "shared = 'from_a'\n");
    writeFile(root / "b" / "main.lua", "assert(shared == nil, 'mod b saw mod a globals')\n"
                                       "shared = 'from_b'\n");

    {
        sim::vfs::VirtualFileSystem vfs;
        REQUIRE(vfs.mount(root));
        sim::scripting::ScriptHost host(vfs);

        REQUIRE(host.loadScript("a", "a/main.lua"));
        // Loading b would have failed if the two mods shared a state: b asserts
        // that it cannot see a's `shared`.
        REQUIRE(host.loadScript("b", "b/main.lua"));

        LuaSandbox* a = host.sandbox("a");
        LuaSandbox* b = host.sandbox("b");
        REQUIRE(a != nullptr);
        REQUIRE(b != nullptr);

        std::string error;
        CHECK(a->run("assert(shared == 'from_a')", "a1", error));
        CHECK(b->run("assert(shared == 'from_b')", "b1", error));

        // A write (including a rawset) in one state must not leak into the other.
        error.clear();
        CHECK(b->run("shared = 'mutated' rawset(_G, 'leak', 1)", "b2", error));
        CHECK(a->run("assert(shared == 'from_a') assert(leak == nil)", "a2", error));
        CHECK(b->run("assert(shared == 'mutated' and leak == 1)", "b3", error));
    }

    std::error_code ignored;
    fs::remove_all(root, ignored);
}
