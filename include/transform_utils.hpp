// In transform_utils.hpp
#pragma once
#include <vector>
#include <Eigen/Dense>

// Transform each scan point with a 6DoF pose (roll, pitch, yaw + translation)
std::vector<Eigen::Vector3d> transformScan(
    const std::vector<Eigen::Vector3d>& scan,
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& rpy // roll, pitch, yaw
);
