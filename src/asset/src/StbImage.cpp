// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors

// The single translation unit that compiles the stb_image decoder, so every
// module (the glTF loader and the headless runner) shares one copy of it and the
// linker sees no duplicate symbols.
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
