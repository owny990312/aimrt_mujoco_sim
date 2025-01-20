// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include <future>
#include <vector>

#include "imu.pb.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"

namespace aimrt_mujoco_sim::mujoco_sim_module {

class ImuSensorPublisher : public PublisherBase {
 public:
  struct Options {
    struct Imu {
      std::string name;
      std::string bind_site;
      std::string bind_framequat;
      std::string bind_gyro;
      std::string bind_framepos;
      std::string bind_velocimeter;
      std::string bind_accelerometer;
    };
    std::vector<Imu> imus;
  };

 public:
  ImuSensorPublisher() {}
  ~ImuSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  std::string_view Type() const noexcept override { return "imu_sensor"; }

  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override {
    publisher_ = publisher_handle;
  }

  void SetMj(mjModel* m, mjData* d) override {
    m_ = m;
    d_ = d;
  }

  void SetExecutor(aimrt::executor::ExecutorRef executor) override {
    executor_ = executor;
  };

  void SetFreq(uint32_t freq) override {
    channel_frq_ = freq;
  };

  void PublishSensorData() override;

 private:
  void RegisterSensorAddr();
  void CheckFrequency();

 private:
  struct SensorAddrGroup {
    int32_t framequat_addr;
    int32_t gyro_addr;
    int32_t framepos_addr;
    int32_t velocimeter_addr;
    int32_t accelerometer_addr;
  };

  struct SensorStateGroup {
    struct {
      double w, x, y, z;
    } orientation;
    struct {
      double x, y, z;
    } angular_velocity;
    struct {
      double x, y, z;
    } position;
    struct {
      double x, y, z;
    } linear_velocity;
    struct {
      double x, y, z;
    } linear_acceleration;
  };

 private:
  Options options_;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;

  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;

  size_t imu_num_ = 0;
  std::vector<SensorAddrGroup> sensor_addr_vec_;
  std::vector<std::string> name_vec_;

  uint32_t counter_ = 0;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module
