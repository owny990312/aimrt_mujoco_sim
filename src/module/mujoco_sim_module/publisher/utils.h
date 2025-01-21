// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>
#include <string>
#include "mujoco/mujoco.h"
#include "mujoco_sim_module/global.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

[[nodiscard]] inline int32_t GetValidatedSensorId(const mjModel* m,
                                                  const std::string& sensor_name) {
  if (sensor_name.empty()) {
    return -1;
  }

  int32_t sensor_idx = mj_name2id(m, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_idx < 0) {
    AIMRT_CHECK_ERROR_THROW(false, "Invalid sensor name: {}.", sensor_name);
  }
  return sensor_idx;
}

[[nodiscard]] inline bool CheckFrequency(const uint32_t channel_frq,
                                         double& avg_interval_base) noexcept {
  constexpr uint32_t MAX_SIM_FRQ = 1000;  // the maximum frequency of the simulation
  constexpr double kError = 0.05;

  if (channel_frq == 0 || channel_frq > MAX_SIM_FRQ) return false;

  const auto exact_division = (MAX_SIM_FRQ % channel_frq) == 0;
  avg_interval_base = static_cast<double>(MAX_SIM_FRQ) / channel_frq;

  if (exact_division) return true;

  const auto base = avg_interval_base;
  const auto calc_error = [base](const uint32_t interval) noexcept {
    return std::abs(static_cast<double>(interval) - base) / base;
  };

  const uint32_t intervals[] = {MAX_SIM_FRQ / channel_frq, (MAX_SIM_FRQ / channel_frq) + 1};
  return std::ranges::all_of(intervals, [&](auto i) { return calc_error(i) <= kError; });
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher