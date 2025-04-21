#pragma once

#include <vector>
#include <Eigen/Dense>
#include "voxel_map.hpp"

double computeScore(
    const VoxelMap& map,
    const std::vector<Eigen::Vector3d>& scan,
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& rpy // roll, pitch, yaw
);
