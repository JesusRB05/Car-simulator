// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "modding/PackageWriter.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <system_error>
#include <vector>

namespace sim::modding {
namespace {

namespace fs = std::filesystem;

std::uint32_t crc32(const void* data, std::size_t size) {
    static const std::array<std::uint32_t, 256> table = [] {
        std::array<std::uint32_t, 256> values{};
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t value = i;
            for (int bit = 0; bit < 8; ++bit) {
                value = (value & 1u) != 0u ? 0xEDB88320u ^ (value >> 1) : value >> 1;
            }
            values[i] = value;
        }
        return values;
    }();

    std::uint32_t crc = 0xFFFFFFFFu;
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        crc = table[(crc ^ bytes[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

void put16(std::string& out, std::uint16_t value) {
    out.push_back(static_cast<char>(value & 0xFFu));
    out.push_back(static_cast<char>((value >> 8) & 0xFFu));
}

void put32(std::string& out, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<char>((value >> (8 * i)) & 0xFFu));
    }
}

bool readWholeFile(const fs::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

struct Entry {
    std::string name; // '/'-separated, as it appears inside the package
    std::string bytes;
    std::uint32_t crc = 0;
    std::uint32_t offset = 0; // of the local header
};

// 1980-01-01 00:00, the earliest a zip can express. Fixed on purpose: the
// timestamp must not leak into the bytes, or the package would not be
// reproducible.
constexpr std::uint16_t kDosDate = 0x0021;
constexpr std::uint16_t kDosTime = 0x0000;

} // namespace

bool writePackage(const std::filesystem::path& directory, const std::filesystem::path& destination,
                  std::string& error) {
    std::error_code ec;
    if (!fs::is_directory(directory, ec)) {
        error = "'" + directory.string() + "' is not a directory";
        return false;
    }

    std::vector<fs::path> files;
    for (const fs::recursive_directory_iterator::value_type& entry : fs::recursive_directory_iterator(directory, ec)) {
        if (!entry.is_regular_file(ec)) {
            continue;
        }
        files.push_back(entry.path());
    }
    // Sorted so the archive layout does not depend on the filesystem's order.
    std::sort(files.begin(), files.end());

    std::vector<Entry> entries;
    entries.reserve(files.size());
    for (const fs::path& file : files) {
        Entry packed;
        packed.name = file.lexically_relative(directory).generic_string();
        if (!readWholeFile(file, packed.bytes)) {
            error = "cannot read '" + file.string() + "'";
            return false;
        }
        packed.crc = crc32(packed.bytes.data(), packed.bytes.size());
        entries.push_back(std::move(packed));
    }
    if (entries.empty()) {
        error = "'" + directory.string() + "' is empty";
        return false;
    }

    std::string out;
    std::string central;
    for (Entry& entry : entries) {
        entry.offset = static_cast<std::uint32_t>(out.size());
        const std::string& name = entry.name;
        const auto size = static_cast<std::uint32_t>(entry.bytes.size());

        // Local file header. Flag bit 11 = the name is UTF-8.
        put32(out, 0x04034b50u);
        put16(out, 20);     // version needed (2.0)
        put16(out, 0x0800); // UTF-8 names
        put16(out, 0);      // method: stored
        put16(out, kDosTime);
        put16(out, kDosDate);
        put32(out, entry.crc);
        put32(out, size); // compressed size
        put32(out, size); // uncompressed size
        put16(out, static_cast<std::uint16_t>(name.size()));
        put16(out, 0); // extra field length
        out += name;
        out += entry.bytes;

        put32(central, 0x02014b50u);
        put16(central, 20);     // version made by
        put16(central, 20);     // version needed
        put16(central, 0x0800); // UTF-8 names
        put16(central, 0);      // method: stored
        put16(central, kDosTime);
        put16(central, kDosDate);
        put32(central, entry.crc);
        put32(central, size);
        put32(central, size);
        put16(central, static_cast<std::uint16_t>(name.size()));
        put16(central, 0); // extra field length
        put16(central, 0); // comment length
        put16(central, 0); // disk number
        put16(central, 0); // internal attributes
        put32(central, 0); // external attributes
        put32(central, entry.offset);
        central += name;
    }

    const auto centralOffset = static_cast<std::uint32_t>(out.size());
    out += central;
    const auto centralSize = static_cast<std::uint32_t>(central.size());
    const auto count = static_cast<std::uint16_t>(entries.size());

    // End of central directory.
    put32(out, 0x06054b50u);
    put16(out, 0); // this disk
    put16(out, 0); // disk with the central directory
    put16(out, count);
    put16(out, count);
    put32(out, centralSize);
    put32(out, centralOffset);
    put16(out, 0); // comment length

    const fs::path parent = destination.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent, ec);
    }
    std::ofstream file(destination, std::ios::binary | std::ios::trunc);
    if (!file) {
        error = "cannot write '" + destination.string() + "'";
        return false;
    }
    file.write(out.data(), static_cast<std::streamsize>(out.size()));
    if (!file) {
        error = "cannot write '" + destination.string() + "'";
        return false;
    }
    return true;
}

} // namespace sim::modding
