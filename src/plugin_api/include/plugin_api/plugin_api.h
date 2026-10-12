/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Copyright (C) 2026 The Car Simulator contributors */
/*
 * Car Simulator native plugin ABI, version 1.
 *
 * This header is intentionally pure C (no C++ constructs) so that plugins can
 * be written in C, C++ or any language able to produce a C ABI shared library.
 * A plugin must export exactly two symbols:
 *
 *   uint32_t sim_plugin_abi_version(void);
 *   int      sim_plugin_entry(uint32_t host_abi_version,
 *                             const sim_plugin_api* api,
 *                             sim_plugin_host* out_host);
 *
 * sim_plugin_abi_version() is read first by the host and compared against
 * SIM_PLUGIN_ABI_VERSION; sim_plugin_entry() returns 0 on success, non-zero on
 * failure. The host ABI version is passed again so the plugin can refuse a host
 * it does not understand.
 *
 * The structs only ever grow by appending fields at the end; `struct_size` lets
 * both sides ignore newer fields they do not know about.
 */
#ifndef SIM_PLUGIN_API_H
#define SIM_PLUGIN_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SIM_PLUGIN_ABI_VERSION 1u
#define SIM_PLUGIN_ABI_VERSION_SYMBOL "sim_plugin_abi_version"
#define SIM_PLUGIN_ENTRY_SYMBOL "sim_plugin_entry"

#if defined(_WIN32)
#define SIM_PLUGIN_EXPORT __declspec(dllexport)
#else
#define SIM_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

    typedef enum sim_log_level
    {
        SIM_LOG_TRACE = 0,
        SIM_LOG_DEBUG = 1,
        SIM_LOG_INFO = 2,
        SIM_LOG_WARN = 3,
        SIM_LOG_ERROR = 4,
        SIM_LOG_CRITICAL = 5
    } sim_log_level;

    /* Services provided by the game to the plugin. */
    typedef struct sim_plugin_api
    {
        uint32_t abi_version;
        size_t struct_size;
        /* Opaque context owned by the host; always passed back to callbacks. */
        void* user_data;
        const char* game_version;
        /* Writes a message to the plugin's named logger. */
        void (*log)(void* user_data, sim_log_level level, const char* message);
        /* Reads a file through the game VFS. Returns bytes written, or -1. */
        int64_t (*vfs_read)(void* user_data, const char* path, void* buffer, size_t buffer_size);
        /* Returns the host-provided name of the calling plugin. */
        const char* (*plugin_name)(void* user_data);
    } sim_plugin_api;

    /* Callbacks the plugin hands back to the game. */
    typedef struct sim_plugin_host
    {
        /* Opaque plugin-owned context, passed back into the callbacks below. */
        void* user_data;
        /* Called once per rendered frame with the frame delta in seconds. */
        void (*on_update)(void* user_data, double delta_seconds);
        /* Called once when the plugin is unloaded. May be NULL. */
        void (*on_shutdown)(void* user_data);
    } sim_plugin_host;

    typedef int (*sim_plugin_entry_fn)(uint32_t host_abi_version, const sim_plugin_api* api, sim_plugin_host* out_host);

#ifdef __cplusplus
}
#endif

#endif /* SIM_PLUGIN_API_H */
