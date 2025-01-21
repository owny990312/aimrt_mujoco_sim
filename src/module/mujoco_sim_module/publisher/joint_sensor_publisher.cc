// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/joint_sensor_publisher.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/utils.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher::JointSensorPublisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher::JointSensorPublisher::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["joints"] = YAML::Node();
    for (const auto& joint : rhs.joints) {
      Node joint_node;
      joint_node["name"] = joint.name;
      joint_node["bind_joint"] = joint.bind_joint;
      joint_node["bind_jointpos_sensor"] = joint.bind_jointpos_sensor;
      joint_node["bind_jointvel_sensor"] = joint.bind_jointvel_sensor;
      node["joints"].push_back(joint_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["joints"] && node["joints"].IsSequence()) {
      for (const auto& joint_node : node["joints"]) {
        auto joint_node_options = Options::Joint{
            .name = joint_node["name"].as<std::string>(),
            .bind_joint = joint_node["bind_joint"].as<std::string>(),
            .bind_jointpos_sensor = joint_node["bind_jointpos_sensor"].as<std::string>(),
            .bind_jointvel_sensor = joint_node["bind_jointvel_sensor"].as<std::string>()};

        rhs.joints.emplace_back(std::move(joint_node_options));
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

void JointSensorPublisher::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  AIMRT_CHECK_ERROR_THROW(CheckFrequency(channel_frq_, avg_interval_base_), "Invalid frequency.");
  RegisterSensorAddr();

  options_node = options_;

  bool ret = aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::JointState>(publisher_);

  AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed.");
}

void JointSensorPublisher::Start() {
}

void JointSensorPublisher::Shutdown() {
}

void JointSensorPublisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;

  if (counter_++ < avg_interval_) return;

  std::unique_ptr<SensorStateGroup[]> state_array(new SensorStateGroup[joint_num_]);

  for (size_t i = 0; i < joint_num_; i++) {
    const auto& addr = sensor_addr_vec_[i];
    state_array[i].jointpos_state = addr.jointpos_addr >= 0 ? d_->sensordata[addr.jointpos_addr] : 0.0;
    state_array[i].jointvel_state = addr.jointvel_addr >= 0 ? d_->sensordata[addr.jointvel_addr] : 0.0;
  }

  executor_.Execute([this, state_array = std::move(state_array)]() {
    aimrt::protocols::sensor::JointState state;
    for (int i = 0; i < joint_num_; ++i) {
      auto* data = state.add_data();
      data->set_name(name_vec_[i]);
      data->set_position(state_array[i].jointpos_state);
      data->set_velocity(state_array[i].jointvel_state);
    }

    aimrt::channel::Publish(publisher_, state);
  });

  avg_interval_ += avg_interval_base_;

  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

void JointSensorPublisher::RegisterSensorAddr() {
  for (const auto& joint : options_.joints) {
    sensor_addr_vec_.emplace_back(SensorAddrGroup{
        .jointpos_addr = GetValidatedSensorId(m_, joint.bind_jointpos_sensor),
        .jointvel_addr = GetValidatedSensorId(m_, joint.bind_jointvel_sensor)});

    name_vec_.emplace_back(joint.name);
  }
  joint_num_ = sensor_addr_vec_.size();
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
