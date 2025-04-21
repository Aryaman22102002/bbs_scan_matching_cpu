#include <vector>
#include "score.hpp"
#include "transform_utils.hpp"
#include <iostream>


double computeScore(
    const VoxelMap& map,
    const std::vector<Eigen::Vector3d>& scan,
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& rpy // roll, pitch, yaw
)
{
  auto transformed_scan = transformScan(scan, translation, rpy);
  
  int occupied_count = 0;
  for (const auto& point : transformed_scan) {
        if (map.isOccupiedNearby(point, 0.10)){
           occupied_count++;
    }

  }
  std::cout << "Matching points: " << occupied_count << "/" << scan.size() << std::endl;

  return static_cast<double>(occupied_count) / scan.size();
}
