// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include "core/Clock.hpp"
#include "core/Config.hpp"

using namespace sim::core;

TEST_CASE("FixedStepClock runs whole steps and reports an interpolation alpha")
{
    FixedStepClock clock(0.01, 8);
    const StepResult result = clock.advance(0.025);
    CHECK(result.steps == 2);
    CHECK(result.alpha == doctest::Approx(0.5));
    CHECK(result.simTime == doctest::Approx(0.02));
}

TEST_CASE("FixedStepClock clamps catch-up work to avoid the spiral of death")
{
    FixedStepClock clock(0.01, 4);
    const StepResult result = clock.advance(10.0);
    CHECK(result.steps == 4);
}

TEST_CASE("Config stores and retrieves nested values without touching disk")
{
    Config config("does-not-exist.toml");
    CHECK_FALSE(config.contains("a.b"));

    config.setInt("a.b", 42);
    config.setDouble("a.c", 1.5);
    config.setString("a.d", "hello");
    config.setBool("a.e", true);

    CHECK(config.contains("a.b"));
    CHECK(config.getInt("a.b") == 42);
    CHECK(config.getDouble("a.c") == doctest::Approx(1.5));
    CHECK(config.getString("a.d") == "hello");
    CHECK(config.getBool("a.e"));

    config.erase("a.b");
    CHECK_FALSE(config.contains("a.b"));
    CHECK(config.getInt("a.b", -1) == -1);
}
