// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::audio {

// A tiny wrapper around miniaudio's high-level engine. Sounds are decoded from
// memory, so all audio assets are read through the VFS like everything else.
class AudioEngine {
public:
    using SoundHandle = std::uint32_t;
    static constexpr SoundHandle kInvalidSound = 0;

    AudioEngine();
    ~AudioEngine();

    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    bool isRunning() const { return running_; }
    void setMasterVolume(float volume);

    // Loads a sound from the VFS. Returns kInvalidSound on failure.
    SoundHandle loadSound(vfs::VirtualFileSystem& vfs, std::string_view virtualPath);

    void play(SoundHandle handle, float volume = 1.0f, bool loop = false);
    void stop(SoundHandle handle);
    void stopAll();

    void update();

private:
    struct Sound;

    bool running_ = false;
    std::unique_ptr<struct EngineStorage> engine_;
    std::vector<std::unique_ptr<Sound>> sounds_;
};

} // namespace sim::audio
