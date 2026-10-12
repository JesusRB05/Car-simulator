/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 The Car Simulator contributors */
/*
 * Reference native plugin. It proves the C ABI end to end: it reports its ABI
 * version, receives the host API, logs a message on load and (optionally) reads
 * a text file from the VFS, then counts frames via on_update.
 */
#include "plugin_api/plugin_api.h"

#include <stdio.h>

typedef struct plugin_state
{
    const sim_plugin_api* api;
    unsigned long long frames;
    int logged_content;
} plugin_state;

static plugin_state g_state;

static void plugin_on_update(void* user_data, double delta_seconds)
{
    plugin_state* state = (plugin_state*)user_data;
    if (state == NULL)
    {
        return;
    }
    state->frames++;

    if (state->frames == 1U)
    {
        char buffer[256];
        const int64_t read = state->api->vfs_read(state->api->user_data, "/README.txt", buffer, sizeof(buffer) - 1U);
        if (read >= 0)
        {
            buffer[read] = '\0';
            char message[512];
            snprintf(message, sizeof(message), "read %lld bytes from /README.txt", (long long)read);
            state->api->log(state->api->user_data, SIM_LOG_INFO, message);
        }
        else
        {
            state->api->log(state->api->user_data, SIM_LOG_WARN, "/README.txt not found in the VFS");
        }
        state->logged_content = 1;
    }

    if (state->frames % 600ULL == 0ULL)
    {
        char message[128];
        snprintf(message, sizeof(message), "alive after %llu frames (dt=%.4f s)", state->frames, delta_seconds);
        state->api->log(state->api->user_data, SIM_LOG_INFO, message);
    }
}

SIM_PLUGIN_EXPORT uint32_t sim_plugin_abi_version(void)
{
    return SIM_PLUGIN_ABI_VERSION;
}

SIM_PLUGIN_EXPORT int sim_plugin_entry(uint32_t host_abi_version, const sim_plugin_api* api, sim_plugin_host* out_host)
{
    if (host_abi_version != SIM_PLUGIN_ABI_VERSION)
    {
        return 1;
    }
    if (api == NULL || out_host == NULL)
    {
        return 2;
    }
    if (api->struct_size < sizeof(sim_plugin_api))
    {
        return 3;
    }

    g_state.api = api;
    g_state.frames = 0;
    g_state.logged_content = 0;

    api->log(api->user_data, SIM_LOG_INFO, "example_native plugin starting");

    out_host->user_data = &g_state;
    out_host->on_update = plugin_on_update;
    out_host->on_shutdown = NULL;
    return 0;
}
