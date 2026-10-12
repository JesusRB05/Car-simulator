// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "audio/AudioEngine.hpp"

#define MINIAUDIO_IMPLEMENTATION
#include <miniaudio.h>

#include <cstring>

#include "core/Log.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::audio {

struct EngineStorage
{
    ma_engine engine{};
};

struct AudioEngine::Sound
{
    std::vector<ma_uint8> data;
    ma_decoder decoder{};
    ma_sound sound{};
    bool decoderReady = false;
    bool soundReady = false;

    ~Sound()
    {
        if (soundReady)
        {
            ma_sound_uninit(&sound);
        }
        if (decoderReady)
        {
            ma_decoder_uninit(&decoder);
        }
    }
};

AudioEngine::AudioEngine() : engine_(std::make_unique<EngineStorage>())
{
    ma_engine_config config = ma_engine_config_init();
    if (ma_engine_init(&config, &engine_->engine) != MA_SUCCESS)
    {
        core::Log::get("audio")->warn("failed to initialise the audio engine; audio will be silent");
        running_ = false;
        return;
    }
    running_ = true;
    core::Log::get("audio")->info("audio engine ready");
}

AudioEngine::~AudioEngine()
{
    stopAll();
    sounds_.clear();
    if (running_)
    {
        ma_engine_uninit(&engine_->engine);
    }
}

void AudioEngine::setMasterVolume(float volume)
{
    if (running_)
    {
        ma_engine_set_volume(&engine_->engine, volume);
    }
}

AudioEngine::SoundHandle AudioEngine::loadSound(vfs::VirtualFileSystem& vfs, std::string_view virtualPath)
{
    const auto bytes = vfs.readBinary(virtualPath);
    if (!bytes || bytes->empty())
    {
        core::Log::get("audio")->error("sound '{}' was not found", virtualPath);
        return kInvalidSound;
    }

    auto sound = std::make_unique<Sound>();
    sound->data.resize(bytes->size());
    std::memcpy(sound->data.data(), bytes->data(), bytes->size());

    ma_decoder_config decoderConfig = ma_decoder_config_init(ma_format_f32, 0, 0);
    if (ma_decoder_init_memory(sound->data.data(), sound->data.size(), &decoderConfig, &sound->decoder) != MA_SUCCESS)
    {
        core::Log::get("audio")->error("failed to decode '{}'", virtualPath);
        return kInvalidSound;
    }
    sound->decoderReady = true;

    if (running_)
    {
        if (ma_sound_init_from_data_source(&engine_->engine, &sound->decoder, 0, nullptr, &sound->sound) != MA_SUCCESS)
        {
            ma_decoder_uninit(&sound->decoder);
            return kInvalidSound;
        }
        sound->soundReady = true;
    }

    sounds_.push_back(std::move(sound));
    return static_cast<SoundHandle>(sounds_.size());
}

void AudioEngine::play(SoundHandle handle, float volume, bool loop)
{
    if (handle == kInvalidSound || handle > sounds_.size() || !running_)
    {
        return;
    }
    Sound& sound = *sounds_[handle - 1];
    if (!sound.soundReady)
    {
        return;
    }
    ma_sound_set_volume(&sound.sound, volume);
    ma_sound_set_looping(&sound.sound, loop ? MA_TRUE : MA_FALSE);
    ma_sound_seek_to_pcm_frame(&sound.sound, 0);
    ma_sound_start(&sound.sound);
}

void AudioEngine::stop(SoundHandle handle)
{
    if (handle == kInvalidSound || handle > sounds_.size() || !running_)
    {
        return;
    }
    Sound& sound = *sounds_[handle - 1];
    if (sound.soundReady)
    {
        ma_sound_stop(&sound.sound);
    }
}

void AudioEngine::stopAll()
{
    if (!running_)
    {
        return;
    }
    for (auto& sound : sounds_)
    {
        if (sound->soundReady)
        {
            ma_sound_stop(&sound->sound);
        }
    }
}

void AudioEngine::update()
{
    // miniaudio runs on its own thread; nothing to do here yet.
}

} // namespace sim::audio
