#include <gtest/gtest.h>

#include "unitree_interface/crc.hpp"
#include <unitree/dds_wrapper/common/crc.h>

#include <array>
#include <cstring>

// Verify our crc_32_core produces identical results to Unitree's crc32_core

TEST(Crc32CoreTest, EmptyInput) {
    uint32_t dummy = 0;
    EXPECT_EQ(
        unitree_interface::crc_32_core(&dummy, 0),
        crc32_core(&dummy, 0)
    );
}

TEST(Crc32CoreTest, SingleWord) {
    uint32_t data = 0xDEADBEEF;
    EXPECT_EQ(
        unitree_interface::crc_32_core(&data, 1),
        crc32_core(&data, 1)
    );
}

TEST(Crc32CoreTest, AllZeros) {
    std::array<uint32_t, 8> data{};
    EXPECT_EQ(
        unitree_interface::crc_32_core(data.data(), data.size()),
        crc32_core(data.data(), data.size())
    );
}

TEST(Crc32CoreTest, AllOnes) {
    std::array<uint32_t, 8> data;
    data.fill(0xFFFFFFFF);
    EXPECT_EQ(
        unitree_interface::crc_32_core(data.data(), data.size()),
        crc32_core(data.data(), data.size())
    );
}

TEST(Crc32CoreTest, IncrementingValues) {
    std::array<uint32_t, 16> data;
    for (uint32_t i = 0; i < data.size(); i++) {
        data[i] = i;
    }
    EXPECT_EQ(
        unitree_interface::crc_32_core(data.data(), data.size()),
        crc32_core(data.data(), data.size())
    );
}

TEST(Crc32CoreTest, RealisticMotorCommand) {
    // Simulate a LowCmd-sized buffer (rough approximation)
    std::array<uint32_t, 256> data{};
    // Set some fields to non-zero to mimic real commands
    data[0] = 0x0001;  // mode_pr
    data[1] = 0x0001;  // mode_machine
    data[10] = 0x3F800000;  // 1.0f as uint32
    data[11] = 0x3DCCCCCD;  // 0.1f as uint32
    EXPECT_EQ(
        unitree_interface::crc_32_core(data.data(), data.size()),
        crc32_core(data.data(), data.size())
    );
}

TEST(Crc32CoreTest, SingleBitDifference) {
    std::array<uint32_t, 4> data_a = {0x00000000, 0x00000000, 0x00000000, 0x00000000};
    std::array<uint32_t, 4> data_b = {0x00000001, 0x00000000, 0x00000000, 0x00000000};

    auto crc_a = unitree_interface::crc_32_core(data_a.data(), data_a.size());
    auto crc_b = unitree_interface::crc_32_core(data_b.data(), data_b.size());

    // CRC should differ for different inputs
    EXPECT_NE(crc_a, crc_b);

    // And both should match Unitree's
    EXPECT_EQ(crc_a, crc32_core(data_a.data(), data_a.size()));
    EXPECT_EQ(crc_b, crc32_core(data_b.data(), data_b.size()));
}
