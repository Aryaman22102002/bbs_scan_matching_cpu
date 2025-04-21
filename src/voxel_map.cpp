#include <iostream>
#include "voxel_map.hpp"
#include <cmath>

VoxelMap::VoxelMap(double voxel_size) : voxel_size_(voxel_size) {}


VoxelIndex VoxelMap::pointToIndex(const Eigen::Vector3d& pt) const {
    return {
        static_cast<int>(std::floor(pt.x() / voxel_size_)),
        static_cast<int>(std::floor(pt.y() / voxel_size_)),
        static_cast<int>(std::floor(pt.z() / voxel_size_))
    };
}

Eigen::Vector3d VoxelMap::voxelToPoint(const VoxelIndex& idx) const {
    return Eigen::Vector3d(
        (idx.x + 0.5) * voxel_size_,
        (idx.y + 0.5) * voxel_size_,
        (idx.z + 0.5) * voxel_size_
    );
}

void VoxelMap::insertPoint(const Eigen::Vector3d& point) {
    VoxelIndex idx = pointToIndex(point);
     //   std::cout << "Inserting point at voxel: ("
    //          << idx.x << ", " << idx.y << ", " << idx.z << ")" << std::endl;
    voxel_set_.insert(idx);
}

bool VoxelMap::isOccupied(const Eigen::Vector3d& point) const {
    VoxelIndex idx = pointToIndex(point);
  //  std::cout << "Lidar Scan Point After Transformation: " << point.transpose() << std::endl;
   // std::cout << "Querying voxel: ("
   //           << idx.x << ", " << idx.y << ", " << idx.z << ")" << std::endl;
    return voxel_set_.find(idx) != voxel_set_.end();
}

bool VoxelMap::isOccupiedNearby(const Eigen::Vector3d& point, double radius) const {
    VoxelIndex center = pointToIndex(point);
    int delta = std::ceil(radius / voxel_size_);

    for (int dx = -delta; dx <= delta; ++dx) {
        for (int dy = -delta; dy <= delta; ++dy) {
            for (int dz = -delta; dz <= delta; ++dz) {
                VoxelIndex neighbor = {center.x + dx, center.y + dy, center.z + dz};
                if (voxel_set_.find(neighbor) != voxel_set_.end()) {
                    return true;
                }
            }
        }
    }
    return false;
}


