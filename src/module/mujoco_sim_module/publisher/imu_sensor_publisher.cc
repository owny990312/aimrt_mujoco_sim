// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/imu_sensor_publisher.h"
#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "mujoco_sim_module/global.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::ImuSensorPublisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::ImuSensorPublisher::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["imus"] = YAML::Node();
    for (const auto& imu : rhs.imus) {
      Node imu_node;
      imu_node["name"] = imu.name;
      imu_node["bind_site"] = imu.bind_site;
      imu_node["bind_framequat"] = imu.bind_framequat;
      imu_node["bind_gyro"] = imu.bind_gyro;
      imu_node["bind_framepos"] = imu.bind_framepos;
      imu_node["bind_velocimeter"] = imu.bind_velocimeter;
      imu_node["bind_accelerometer"] = imu.bind_accelerometer;
      node["imus"].push_back(imu_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["imus"] && node["imus"].IsSequence()) {
      for (const auto& imu_node : node["imus"]) {
        auto imu_node_options = Options::Imu{
            .name = imu_node["name"].as<std::string>(),
            .bind_site = imu_node["bind_site"].as<std::string>(),
            .bind_framequat = imu_node["bind_framequat"].as<std::string>(),
            .bind_gyro = imu_node["bind_gyro"].as<std::string>(),
            .bind_framepos = imu_node["bind_framepos"].as<std::string>(),
            .bind_velocimeter = imu_node["bind_velocimeter"].as<std::string>(),
            .bind_accelerometer = imu_node["bind_accelerometer"].as<std::string>()};

        rhs.imus.emplace_back(std::move(imu_node_options));
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module {

void ImuSensorPublisher::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  CheckFrequency();
  RegisterSensorAddr();

  options_node = options_;

  bool ret = aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::ImuState>(publisher_);

  AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed.");
}

void ImuSensorPublisher::Start() {
}

void ImuSensorPublisher::Shutdown() {
}

void ImuSensorPublisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;

  if (counter_++ < avg_interval_) return;

  std::unique_ptr<SensorStateGroup[]> state_array(new SensorStateGroup[imu_num_]);

  for (size_t i = 0; i < imu_num_; i++) {
    const auto& addr = sensor_addr_vec_[i];

    // 读取方向四元数数据
    if (addr.framequat_addr >= 0) {
      state_array[i].orientation = {
          d_->sensordata[addr.framequat_addr],
          d_->sensordata[addr.framequat_addr + 1],
          d_->sensordata[addr.framequat_addr + 2],
          d_->sensordata[addr.framequat_addr + 3]};
    }

    // 读取角速度数据
    if (addr.gyro_addr >= 0) {
      state_array[i].angular_velocity = {
          d_->sensordata[addr.gyro_addr],
          d_->sensordata[addr.gyro_addr + 1],
          d_->sensordata[addr.gyro_addr + 2]};
    }

    // 读取位置数据
    if (addr.framepos_addr >= 0) {
      state_array[i].position = {
          d_->sensordata[addr.framepos_addr],
          d_->sensordata[addr.framepos_addr + 1],
          d_->sensordata[addr.framepos_addr + 2]};
    }

    // 读取线速度数据
    if (addr.velocimeter_addr >= 0) {
      state_array[i].linear_velocity = {
          d_->sensordata[addr.velocimeter_addr],
          d_->sensordata[addr.velocimeter_addr + 1],
          d_->sensordata[addr.velocimeter_addr + 2]};
    }

    // 读取线加速度数据
    if (addr.accelerometer_addr >= 0) {
      state_array[i].linear_acceleration = {
          d_->sensordata[addr.accelerometer_addr],
          d_->sensordata[addr.accelerometer_addr + 1],
          d_->sensordata[addr.accelerometer_addr + 2]};
    }
  }

  executor_.Execute([this, state_array = std::move(state_array)]() {
    aimrt::protocols::sensor::ImuState state;
    for (int i = 0; i < imu_num_; ++i) {
      auto* data = state.add_data();
      data->set_name(name_vec_[i]);

      // 设置方向数据
      auto* orientation = data->mutable_orientation();
      orientation->set_w(state_array[i].orientation.w);
      orientation->set_x(state_array[i].orientation.x);
      orientation->set_y(state_array[i].orientation.y);
      orientation->set_z(state_array[i].orientation.z);

      // 设置角速度数据
      auto* angular_velocity = data->mutable_angular_velocity();
      angular_velocity->set_x(state_array[i].angular_velocity.x);
      angular_velocity->set_y(state_array[i].angular_velocity.y);
      angular_velocity->set_z(state_array[i].angular_velocity.z);

      // 设置位置数据
      auto* linear_position = data->mutable_linear_position();
      linear_position->set_x(state_array[i].position.x);
      linear_position->set_y(state_array[i].position.y);
      linear_position->set_z(state_array[i].position.z);

      // 设置线速度数据
      auto* linear_velocity = data->mutable_linear_velocity();
      linear_velocity->set_x(state_array[i].linear_velocity.x);
      linear_velocity->set_y(state_array[i].linear_velocity.y);
      linear_velocity->set_z(state_array[i].linear_velocity.z);

      // 设置线加速度数据
      auto* linear_acceleration = data->mutable_linear_acceleration();
      linear_acceleration->set_x(state_array[i].linear_acceleration.x);
      linear_acceleration->set_y(state_array[i].linear_acceleration.y);
      linear_acceleration->set_z(state_array[i].linear_acceleration.z);
    }

    aimrt::channel::Publish(publisher_, state);
  });

  avg_interval_ += avg_interval_base_;

  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

void ImuSensorPublisher::RegisterSensorAddr() {
  for (const auto& imu : options_.imus) {
    const int32_t framequat_idx = !imu.bind_framequat.empty()
                                      ? mj_name2id(m_, mjOBJ_SENSOR, imu.bind_framequat.c_str())
                                      : -1;
    const int32_t gyro_idx = !imu.bind_gyro.empty()
                                 ? mj_name2id(m_, mjOBJ_SENSOR, imu.bind_gyro.c_str())
                                 : -1;
    const int32_t framepos_idx = !imu.bind_framepos.empty()
                                     ? mj_name2id(m_, mjOBJ_SENSOR, imu.bind_framepos.c_str())
                                     : -1;
    const int32_t velocimeter_idx = !imu.bind_velocimeter.empty()
                                        ? mj_name2id(m_, mjOBJ_SENSOR, imu.bind_velocimeter.c_str())
                                        : -1;
    const int32_t accelerometer_idx = !imu.bind_accelerometer.empty()
                                          ? mj_name2id(m_, mjOBJ_SENSOR, imu.bind_accelerometer.c_str())
                                          : -1;

    if (!imu.bind_framequat.empty() && framequat_idx < 0) {
      AIMRT_CHECK_ERROR_THROW(false, "Invalid framequat sensor name '{}'.",
                              imu.bind_framequat);
    }
    if (!imu.bind_gyro.empty() && gyro_idx < 0) {
      AIMRT_CHECK_ERROR_THROW(false, "Invalid gyro sensor name '{}'.",
                              imu.bind_gyro);
    }
    if (!imu.bind_framepos.empty() && framepos_idx < 0) {
      AIMRT_CHECK_ERROR_THROW(false, "Invalid framepos sensor name '{}'.",
                              imu.bind_framepos);
    }
    if (!imu.bind_velocimeter.empty() && velocimeter_idx < 0) {
      AIMRT_CHECK_ERROR_THROW(false, "Invalid velocimeter sensor name '{}'.",
                              imu.bind_velocimeter);
    }
    if (!imu.bind_accelerometer.empty() && accelerometer_idx < 0) {
      AIMRT_CHECK_ERROR_THROW(false, "Invalid accelerometer sensor name '{}'.",
                              imu.bind_accelerometer);
    }

    sensor_addr_vec_.emplace_back(SensorAddrGroup{
        .framequat_addr = framequat_idx,
        .gyro_addr = gyro_idx,
        .framepos_addr = framepos_idx,
        .velocimeter_addr = velocimeter_idx,
        .accelerometer_addr = accelerometer_idx});

    name_vec_.emplace_back(imu.name);
  }
  imu_num_ = sensor_addr_vec_.size();
}

void ImuSensorPublisher::CheckFrequency() {
  constexpr static uint32_t MAX_SIM_FRQ = 1000;
  constexpr static double kError = 0.05;

  AIMRT_CHECK_ERROR_THROW(channel_frq_ <= MAX_SIM_FRQ,
                          "Invalid channel frequency {}, exceeds maximum frequency (1000 Hz)",
                          channel_frq_);
  avg_interval_base_ = static_cast<double>(MAX_SIM_FRQ) / static_cast<double>(channel_frq_);

  if (MAX_SIM_FRQ % channel_frq_ == 0) return;

  const uint32_t lower_interval = MAX_SIM_FRQ / channel_frq_;
  const uint32_t upper_interval = lower_interval + 1;

  const double lower_error = std::abs(lower_interval - avg_interval_base_) / avg_interval_base_;
  const double upper_error = std::abs(upper_interval - avg_interval_base_) / avg_interval_base_;

  AIMRT_CHECK_ERROR_THROW((lower_error <= kError && upper_error <= kError),
                          "Invalid channel frequency {}, which cauess the frequency error is more than {} ",
                          channel_frq_, kError);
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module