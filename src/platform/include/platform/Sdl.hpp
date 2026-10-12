// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

namespace sim::platform {

// RAII wrapper around SDL global initialisation. One instance must outlive any
// Window, InputManager or other SDL-based object.
class SdlContext
{
public:
    SdlContext();
    ~SdlContext();

    SdlContext(const SdlContext&) = delete;
    SdlContext& operator=(const SdlContext&) = delete;

    bool valid() const { return valid_; }

private:
    bool valid_ = false;
};

} // namespace sim::platform
