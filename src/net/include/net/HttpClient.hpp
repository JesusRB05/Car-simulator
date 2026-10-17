// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#pragma once

#include <memory>
#include <string>
#include <string_view>

namespace sim::net {

struct HttpResponse {
    int status = 0; // 0 when nothing was ever received
    std::string body;

    std::string error; // empty on success

    bool ok() const { return error.empty() && status >= 200 && status < 300; }
};

// The only network client in the game, used by the opt-in mod catalog. It is
// deliberately tiny: GET, no cookies, no accounts, no telemetry, no background
// traffic. Nothing in the simulation links against it except the catalog.
//
// `file://` URLs and plain filesystem paths are handled by every build, which
// is what a local index, a shared folder and the tests use. `https://` needs a
// TLS implementation the build does not ship yet; such a request fails with an
// error that says so instead of silently downloading in the clear
// (docs/DECISIONS.md, "HTTP transport").
class HttpClient {
public:
    virtual ~HttpClient() = default;
    virtual HttpResponse get(std::string_view url) = 0;
};

// The client this build ships.
std::unique_ptr<HttpClient> makeHttpClient();

// Resolves `url` to a local path: `file:///C:/x`, `file://x` and a plain path
// all work. Returns false for anything with a network scheme.
bool resolveLocalUrl(std::string_view url, std::string& localPath);

} // namespace sim::net
