// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace sim::core {

// Incremental SHA-256 (FIPS 180-4). Used to fingerprint native plugin
// libraries before they are loaded, so the log and the UI record exactly which
// binary the user trusted.
class Sha256 {
public:
    void update(const void* data, std::size_t size);

    // Finalises the digest and returns it as 64 lowercase hex characters. The
    // object must not be reused afterwards.
    std::string hexDigest();

private:
    void transform(const std::uint8_t* block);

    std::uint32_t state_[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                               0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
    std::uint64_t bitCount_ = 0;
    std::uint8_t buffer_[64] = {};
    std::size_t buffered_ = 0;
};

// Convenience: hashes a whole buffer and returns the lowercase hex digest.
std::string sha256Hex(const void* data, std::size_t size);

// Hashes a file in streaming chunks. On failure returns an empty string and
// fills `error`.
std::string sha256File(const std::filesystem::path& path, std::string& error);

} // namespace sim::core
