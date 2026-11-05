#include <gtest/gtest.h>

#include <limits>

#include "pb_minco_smoother/stalled_warm_start_gate.hpp"

namespace {
using pb_minco::StalledWarmStartGate;

TEST(StalledWarmStartGate, DefaultsAndFalseLeaveWarmStartAlone) {
    StalledWarmStartGate gate;
    EXPECT_FALSE(gate.state(100., 1.).inhibited);
    gate.update(false, 100., 1.);
    EXPECT_FALSE(gate.state(100.1, 1.).inhibited);
    EXPECT_EQ(gate.state(100.1, 1.).epoch, 0u);
}

TEST(StalledWarmStartGate, HeartbeatsDoNotCreateNewEpisodes) {
    StalledWarmStartGate gate;
    gate.update(true, 100., 1.);
    EXPECT_TRUE(gate.state(100.1, 1.).inhibited);
    const auto epoch = gate.state(100.1, 1.).epoch;
    gate.update(true, 100.2, 1.);
    EXPECT_EQ(gate.state(100.3, 1.).epoch, epoch);
    gate.update(false, 100.4, 1.);
    EXPECT_FALSE(gate.state(100.4, 1.).inhibited);
    EXPECT_EQ(gate.state(100.4, 1.).epoch, epoch);
}

TEST(StalledWarmStartGate, TrueFalseBetweenSmoothCallsStillInvalidatesOldPva) {
    StalledWarmStartGate gate;
    const auto before = gate.state(100., 1.);
    gate.update(true, 100.1, 1.);
    gate.update(false, 100.2, 1.);
    const auto after = gate.state(100.3, 1.);
    EXPECT_FALSE(after.inhibited);
    EXPECT_GT(after.epoch, before.epoch);
}

TEST(StalledWarmStartGate, MissingSignalFailsOpenAndResumedTrueIsANewEpisode) {
    StalledWarmStartGate gate;
    gate.update(true, 100., 1.);
    const auto before = gate.state(100., 1.).epoch;
    EXPECT_FALSE(gate.state(101.01, 1.).inhibited);
    gate.update(true, 101.2, 1.);
    EXPECT_TRUE(gate.state(101.3, 1.).inhibited);
    EXPECT_GT(gate.state(101.3, 1.).epoch, before);
}

TEST(StalledWarmStartGate, InvalidTimeAndClockResetAreBounded) {
    StalledWarmStartGate gate;
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    gate.update(true, nan, 1.);
    EXPECT_EQ(gate.state(100., 1.).epoch, 0u);
    gate.update(true, 100., 1.);
    EXPECT_FALSE(gate.state(99., 1.).inhibited);
    EXPECT_FALSE(gate.state(nan, 1.).inhibited);
    const auto before = gate.state(100., 1.).epoch;
    gate.update(true, 10., 1.);
    EXPECT_GT(gate.state(10., 1.).epoch, before);
}
}  // namespace
