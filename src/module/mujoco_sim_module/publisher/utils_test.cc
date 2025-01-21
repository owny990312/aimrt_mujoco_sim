// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/utils.h"
#include <gtest/gtest.h>

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

TEST(CheckFrequencyTest, InvalidFrequency) {
  double avg_interval_base = 0.0;
  EXPECT_FALSE(CheckFrequency(0, avg_interval_base));
  EXPECT_FALSE(CheckFrequency(1001, avg_interval_base));
}

TEST(CheckFrequencyTest, GetAverageInterval) {
  double avg_interval_base = 0.0;
  std::vector<uint32_t> exact_frequencies = {1, 100, 1000};

  for (const auto freq : exact_frequencies) {
    EXPECT_TRUE(CheckFrequency(freq, avg_interval_base));
    EXPECT_DOUBLE_EQ(avg_interval_base, 1000.0 / freq);
  }
}

TEST(CheckFrequencyTest, ExactError) {
  double avg_interval_base = 0.0;

  std::vector<uint32_t> valid_frequencies = {1, 100, 1000};
  for (const auto freq : valid_frequencies) {
    EXPECT_TRUE(CheckFrequency(freq, avg_interval_base));
  }

  std::vector<uint32_t> invalid_frequencies = {999};
  for (const auto freq : invalid_frequencies) {
    EXPECT_FALSE(CheckFrequency(freq, avg_interval_base));
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher