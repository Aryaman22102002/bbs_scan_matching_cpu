#include "kitti_io.hpp"
#include <fstream>
#include <iostream>

std::vector<Eigen::Vector3d> readKittiBin(const std::string& filepath) {
    std::vector<Eigen::Vector3d> points;
    std::ifstream file(filepath, std::ios::binary);
    if (!file) {
        std::cerr << "Could not open KITTI .bin file: " << filepath << std::endl;
        return points;
    }

    float x, y, z, intensity;
    while (file.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
           file.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
           file.read(reinterpret_cast<char*>(&z), sizeof(float)) &&
           file.read(reinterpret_cast<char*>(&intensity), sizeof(float))) {
        points.emplace_back(x, y, z); // Ignore intensity
    }

    return points;
}

