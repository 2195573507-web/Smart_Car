#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ydlidar_driver.h"

namespace s3_ydlidar_bridge {

class OfficialDecoder {
 public:
  OfficialDecoder();

  void setIntensities(bool enabled);

  bool decode(const uint8_t *data, size_t size,
              std::vector<::node_info> &nodes,
              std::string &error);

 private:
  ydlidar::YDlidarDriver driver_;
};

}  // namespace s3_ydlidar_bridge
