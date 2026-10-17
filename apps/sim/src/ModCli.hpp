// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>

namespace sim::app {

// The windowless mod commands. Every trigger (the `--install-mod` flag, a
// `.simmod` file association, a `simmod://` link, an IPC hand-off and the
// in-game button) ends up calling the same ModInstaller, so behaviour does not
// depend on how the user got here.
struct ModCommandOptions {
    std::string install; // a `.simmod`, `.zip` or folder; empty means none
    // An index (a `file://` URL or a path) to check the installed mods against.
    std::string updatesIndex;
    // A required-content file to check the installed mods against (the check a
    // client runs before joining a session).
    std::string contentCheck;
    // Print the content fingerprint of the installed mods and stop.
    bool fingerprint = false;
    bool dryRun = false; // print the plan and stop before touching the disk
};

// Runs the requested mod command without creating a window. Returns a process
// exit code (0 ok, 1 failed, 2 bad input).
int runModCommand(const ModCommandOptions& options);

} // namespace sim::app
