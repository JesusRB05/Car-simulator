// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include "Application.hpp"
#include "HeadlessRunner.hpp"

namespace {

bool matches(const char* arg, const char* name)
{
    return std::strcmp(arg, name) == 0;
}

} // namespace

int main(int argc, char** argv)
{
    sim::app::HeadlessOptions headless;
    headless.executablePath = argc > 0 ? argv[0] : "";
    bool runHeadless = false;
    bool sceneGiven = false;

    for (int i = 1; i < argc; ++i)
    {
        const char* arg = argv[i];
        const char* next = i + 1 < argc ? argv[i + 1] : nullptr;
        if (matches(arg, "--headless"))
        {
            runHeadless = true;
        }
        else if (matches(arg, "--validation"))
        {
            headless.validation = true;
        }
        else if (matches(arg, "--frames") && next != nullptr)
        {
            headless.frames = static_cast<std::uint32_t>(std::strtoul(next, nullptr, 10));
            ++i;
        }
        else if (matches(arg, "--width") && next != nullptr)
        {
            headless.width = static_cast<std::uint32_t>(std::strtoul(next, nullptr, 10));
            ++i;
        }
        else if (matches(arg, "--height") && next != nullptr)
        {
            headless.height = static_cast<std::uint32_t>(std::strtoul(next, nullptr, 10));
            ++i;
        }
        else if (matches(arg, "--scene") && next != nullptr)
        {
            headless.scene = next;
            sceneGiven = true;
            ++i;
        }
        else if (matches(arg, "--screenshot") && next != nullptr)
        {
            headless.screenshot = next;
            ++i;
        }
        else if (matches(arg, "--resize") && next != nullptr)
        {
            headless.resizeFrame = static_cast<int>(std::strtol(next, nullptr, 10));
            ++i;
        }
        else if (matches(arg, "--golden-dir") && next != nullptr)
        {
            headless.goldenDir = next;
            ++i;
        }
        else if (matches(arg, "--verify-golden"))
        {
            headless.verifyGolden = true;
        }
        else if (matches(arg, "--update-golden"))
        {
            headless.updateGolden = true;
        }
        else if (matches(arg, "--semantic"))
        {
            headless.semantic = true;
        }
        else if (matches(arg, "--debug-draw"))
        {
            headless.debugDraw = true;
        }
        else if (matches(arg, "--contact-sheet") && next != nullptr)
        {
            headless.contactSheet = next;
            runHeadless = true;
            ++i;
        }
        else if (matches(arg, "--render-tests-sheet") && next != nullptr)
        {
            headless.renderTestsSheet = next;
            runHeadless = true;
            ++i;
        }
        else if (matches(arg, "--max-diff-percent") && next != nullptr)
        {
            headless.maxDiffPercent = std::strtod(next, nullptr);
            ++i;
        }
        else if (matches(arg, "--min-psnr") && next != nullptr)
        {
            headless.minPsnr = std::strtod(next, nullptr);
            ++i;
        }
    }

    try
    {
        if (runHeadless)
        {
            return sim::app::runHeadless(headless);
        }
        sim::app::Application application;
        if (sceneGiven)
        {
            application.setStartupScene(headless.scene);
        }
        return application.run();
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "fatal: %s\n", error.what());
        return 1;
    }
}
