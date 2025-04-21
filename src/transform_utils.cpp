#include <iostream>
#include "transform_utils.hpp"
#include <Eigen/Dense>

std::vector<Eigen::Vector3d> transformScan(
    const std::vector<Eigen::Vector3d>& scan,
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& rpy 
)
{   
    Eigen::AngleAxisd rollAngle(rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy[2], Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

    std::vector<Eigen::Vector3d> transformed_scan;
    transformed_scan.reserve(scan.size()); 

    for (const auto& point : scan) {
        transformed_scan.push_back(R * point + translation);
    }

    return transformed_scan;
}

