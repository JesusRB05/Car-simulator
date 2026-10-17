// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Car Simulator contributors
#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>

#include "net/HttpClient.hpp"

namespace sim::net {
namespace {

constexpr std::string_view kFileScheme = "file://";

class LocalHttpClient : public HttpClient {
public:
    HttpResponse get(std::string_view url) override {
        HttpResponse response;
        if (url.find("://") != std::string_view::npos && !url.starts_with(kFileScheme)) {
            response.error = "'" + std::string(url) +
                             "' needs a transport this build does not have: only file:// URLs are supported yet "
                             "(an https index requires the TLS transport described in docs/DECISIONS.md)";
            return response;
        }

        std::string path;
        if (!resolveLocalUrl(url, path)) {
            response.error = "cannot resolve '" + std::string(url) + "'";
            return response;
        }

        std::error_code ec;
        if (!std::filesystem::exists(path, ec)) {
            response.status = 404;
            response.error = "'" + path + "' does not exist";
            return response;
        }

        std::ifstream in(path, std::ios::binary);
        if (!in) {
            response.error = "cannot read '" + path + "'";
            return response;
        }
        response.status = 200;
        response.body.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        return response;
    }
};

} // namespace

bool resolveLocalUrl(std::string_view url, std::string& localPath) {
    std::string text(url);
    if (text.starts_with(kFileScheme)) {
        text.erase(0, kFileScheme.size());
        // `file:///C:/x` keeps the leading slash, which Windows does not want.
        if (text.size() > 2 && text.front() == '/' && text[2] == ':') {
            text.erase(0, 1);
        }
    } else if (text.find("://") != std::string::npos) {
        return false; // a network scheme is not a path
    }
    if (text.empty()) {
        return false;
    }
    localPath = text;
    return true;
}

std::unique_ptr<HttpClient> makeHttpClient() {
    return std::make_unique<LocalHttpClient>();
}

} // namespace sim::net
