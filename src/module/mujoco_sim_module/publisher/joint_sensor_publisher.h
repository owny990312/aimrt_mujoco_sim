// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once
#include <future>
#include <vector>

#include "joint.pb.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

class JointSensorPublisher : public PublisherBase {
 public:
  struct Options {
    struct Joint {
      std::string name;
      std::string bind_joint;
      std::string bind_jointpos_sensor;
      std::string bind_jointvel_sensor;
    };
    std::vector<Joint> joints;
  };

 public:
  JointSensorPublisher() {}
  ~JointSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  void Start() override;
  void Shutdown() override;

  std::string_view Type() const noexcept override { return "joint_sensor"; }

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

 private:
  struct SensorAddrGroup {
    int32_t jointpos_addr;
    int32_t jointvel_addr;
  };

  struct SensorStateGroup {
    double jointpos_state;
    double jointvel_state;
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

  size_t joint_num_ = 0;
  std::vector<SensorAddrGroup> sensor_addr_vec_;
  std::vector<std::string> name_vec_;

  uint32_t counter_ = 0;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
