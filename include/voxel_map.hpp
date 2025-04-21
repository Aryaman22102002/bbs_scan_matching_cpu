#pragma once

#include <unordered_set>
#include <Eigen/Dense>

// Define a struct to represent voxel grid indices
struct VoxelIndex {
    int x, y, z;

    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Hash function for VoxelIndex so it can be used in unordered_set
namespace std {
    template <>
    struct hash<VoxelIndex> {
        size_t operator()(const VoxelIndex& v) const {
            return ((v.x * 73856093) ^ (v.y * 19349663) ^ (v.z * 83492791));
        }
    };
}

// VoxelMap class
class VoxelMap {
public:
    VoxelMap(double voxel_size);
    void insertPoint(const Eigen::Vector3d& point);
    bool isOccupied(const Eigen::Vector3d& point) const;
    Eigen::Vector3d voxelToPoint(const VoxelIndex& idx) const;
    std::unordered_set<VoxelIndex> voxel_set_;
    bool isOccupiedNearby(const Eigen::Vector3d& point, double radius) const;


private:
    double voxel_size_;

    VoxelIndex pointToIndex(const Eigen::Vector3d& pt) const;
};

