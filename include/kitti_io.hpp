#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

// Reads a KITTI Velodyne .bin file and returns a vector of 3D points (x, y, z)
std::vector<Eigen::Vector3d> readKittiBin(const std::string& filepath);
