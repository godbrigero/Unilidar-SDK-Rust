#pragma once

#include "rust/cxx.h"
#include "unitree_lidar_sdk.h"
#include <memory>
#include <string>
#include <vector>

struct ImuRust;
struct PointUnitreeRust;
struct CloudRust;

enum class MessageTypeRust : uint8_t;
enum class LidarWorkingModeRust : uint8_t;
enum class LedDisplayModeRust : uint8_t;

class UnitreeLidarWrapper {
public:
  UnitreeLidarWrapper();
  ~UnitreeLidarWrapper();

  bool initialize(uint16_t cloud_scan_num, rust::Str port, uint32_t baudrate,
                  float rotate_yaw_bias, float range_scale, float range_bias,
                  float range_max, float range_min);

  bool initialize_udp(uint16_t cloud_scan_num, uint16_t lidar_port,
                      rust::Str lidar_ip, uint16_t local_port,
                      rust::Str local_ip, float rotate_yaw_bias,
                      float range_scale, float range_bias, float range_max,
                      float range_min);

  bool close_udp();

  MessageTypeRust run_parse();
  CloudRust get_cloud();
  ImuRust get_imu();
  void reset();
  void set_lidar_working_mode(LidarWorkingModeRust mode);
  void set_led_display_mode(LedDisplayModeRust mode);
  void set_led_table(rust::Slice<const uint8_t> table);
  rust::String get_version_of_firmware();
  rust::String get_version_of_sdk();
  uint32_t get_time_delay();
  float get_dirty_percentage();

private:
  std::unique_ptr<unitree_lidar_sdk::UnitreeLidarReader> reader_;
};

std::unique_ptr<UnitreeLidarWrapper> make_unitree_lidar_reader();
