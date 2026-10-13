// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "net/NetworkSystem.hpp"

#include "core/Log.hpp"

namespace sim::net {

bool NetworkSystem::start(std::uint16_t port) {
    port_ = port;
    // Networking is not implemented yet; report that plainly.
    running_ = false;
    status_ = "offline (multiplayer not implemented yet)";
    core::Log::get("net")->info("network system requested on port {} but multiplayer is not implemented yet", port);
    return false;
}

void NetworkSystem::stop() {
    running_ = false;
    status_ = "offline";
}

void NetworkSystem::update(double /*deltaSeconds*/) {
    // No-op until networking lands.
}

} // namespace sim::net
