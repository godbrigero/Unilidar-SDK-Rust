#include "unitree_lidar_wrapper.h"
#include "unitree-lidar-l1-rust/src/bridge.rs.h"

#include <algorithm>
#include <string>

UnitreeLidarWrapper::UnitreeLidarWrapper()
{
  reader_ = std::unique_ptr<unitree_lidar_sdk::UnitreeLidarReader>(
      unitree_lidar_sdk::createUnitreeLidarReader());
}

UnitreeLidarWrapper::~UnitreeLidarWrapper() = default;

bool UnitreeLidarWrapper::initialize(uint16_t cloud_scan_num, rust::Str port,
                                     uint32_t baudrate, float rotate_yaw_bias,
                                     float range_scale, float range_bias,
                                     float range_max, float range_min)
{
  if (!reader_)
  {
    return false;
  }

  // rust::Str is not null-terminated, so convert explicitly
  return reader_->initialize(cloud_scan_num, std::string(port), baudrate,
                             rotate_yaw_bias, range_scale, range_bias,
                             range_max, range_min) == 0;
}

bool UnitreeLidarWrapper::initialize_udp(uint16_t cloud_scan_num,
                                         uint16_t lidar_port,
                                         rust::Str lidar_ip,
                                         uint16_t local_port,
                                         rust::Str local_ip,
                                         float rotate_yaw_bias,
                                         float range_scale, float range_bias,
                                         float range_max, float range_min)
{
  if (!reader_)
  {
    return false;
  }

  return reader_->initializeUDP(cloud_scan_num, lidar_port,
                                std::string(lidar_ip), local_port,
                                std::string(local_ip), rotate_yaw_bias,
                                range_scale, range_bias, range_max,
                                range_min) == 0;
}

bool UnitreeLidarWrapper::close_udp()
{
  if (!reader_)
  {
    return false;
  }

  return reader_->closeUDP();
}

MessageTypeRust UnitreeLidarWrapper::run_parse()
{
  if (!reader_)
  {
    return MessageTypeRust::None;
  }

  auto message_type = reader_->runParse();
  return static_cast<MessageTypeRust>(message_type);
}

CloudRust UnitreeLidarWrapper::get_cloud()
{
  CloudRust cloud{};

  if (!reader_)
  {
    return cloud;
  }

  const auto &sdk_cloud = reader_->getCloud();
  cloud.stamp = sdk_cloud.stamp;
  cloud.id = sdk_cloud.id;
  cloud.ring_num = sdk_cloud.ringNum;
  cloud.points.reserve(sdk_cloud.points.size());
  for (const auto &sdk_point : sdk_cloud.points)
  {
    PointUnitreeRust point;
    point.x = sdk_point.x;
    point.y = sdk_point.y;
    point.z = sdk_point.z;
    point.intensity = sdk_point.intensity;
    point.time = sdk_point.time;
    point.ring = sdk_point.ring;
    cloud.points.push_back(point);
  }

  return cloud;
}

ImuRust UnitreeLidarWrapper::get_imu()
{
  ImuRust imu{};

  if (!reader_)
  {
    return imu;
  }

  const auto &sdk_imu = reader_->getIMU();
  imu.id = sdk_imu.id;
  imu.stamp = sdk_imu.stamp;
  std::copy(sdk_imu.quaternion, sdk_imu.quaternion + 4, imu.quaternion.begin());
  std::copy(sdk_imu.angular_velocity, sdk_imu.angular_velocity + 3, imu.angular_velocity.begin());
  std::copy(sdk_imu.linear_acceleration, sdk_imu.linear_acceleration + 3, imu.linear_acceleration.begin());

  return imu;
}

void UnitreeLidarWrapper::reset()
{
  if (reader_)
  {
    reader_->reset();
  }
}

void UnitreeLidarWrapper::set_lidar_working_mode(LidarWorkingModeRust mode)
{
  if (reader_)
  {
    reader_->setLidarWorkingMode(static_cast<unitree_lidar_sdk::LidarWorkingMode>(mode));
  }
}

void UnitreeLidarWrapper::set_led_display_mode(LedDisplayModeRust mode)
{
  if (reader_)
  {
    reader_->setLEDDisplayMode(static_cast<unitree_lidar_sdk::LEDDisplayMode>(mode));
  }
}

void UnitreeLidarWrapper::set_led_table(rust::Slice<const uint8_t> table)
{
  if (!reader_)
  {
    return;
  }

  uint8_t led_table[45] = {0};
  std::copy_n(table.begin(), std::min<size_t>(table.size(), 45), led_table);
  reader_->setLEDDisplayMode(led_table);
}

rust::String UnitreeLidarWrapper::get_version_of_firmware()
{
  if (!reader_)
  {
    return rust::String("unknown");
  }

  return rust::String(reader_->getVersionOfFirmware());
}

rust::String UnitreeLidarWrapper::get_version_of_sdk()
{
  if (!reader_)
  {
    return rust::String("unknown");
  }

  return rust::String(reader_->getVersionOfSDK());
}

uint32_t UnitreeLidarWrapper::get_time_delay()
{
  if (!reader_)
  {
    return 0;
  }

  return reader_->getTimeDelay();
}

float UnitreeLidarWrapper::get_dirty_percentage()
{
  if (!reader_)
  {
    return 0.0f;
  }

  return reader_->getDirtyPercentage();
}

std::unique_ptr<UnitreeLidarWrapper> make_unitree_lidar_reader()
{
  return std::make_unique<UnitreeLidarWrapper>();
}
