// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#pragma once

#include <functional>
#include <string>
#include <string_view>

namespace sim::app {

// Tells a content loader which mod owns a virtual file, so a descriptor gets its
// namespaced content reference (`<mod-id>/<content-id>`). The app wires it to the
// ModManager; a test may pass a simple map. An empty result means the owner is
// unknown and the bare content id is used as the reference.
using ContentOwner = std::function<std::string(std::string_view)>;

} // namespace sim::app
