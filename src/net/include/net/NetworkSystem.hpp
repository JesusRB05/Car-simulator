// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace sim::net {

// Placeholder for the future multiplayer layer. It exists so the module layout
// is complete and the application can already query a status string; no
// sockets are opened yet. The planned design is an authoritative fixed-step
// server with client-side prediction and rollback.
class NetworkSystem
{
public:
    bool start(std::uint16_t port);
    void stop();
    void update(double deltaSeconds);
    bool isRunning() const { return running_; }
    std::string_view status() const { return status_; }

private:
    bool running_ = false;
    std::uint16_t port_ = 0;
    std::string status_ = "offline";
};

} // namespace sim::net
