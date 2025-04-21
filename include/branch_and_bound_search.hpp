#pragma once
#include <vector>
#include <Eigen/Dense>
#include "voxel_map.hpp"
#include "score.hpp"
#include <functional>

struct Node {
    Eigen::Vector3d translation;
    Eigen::Vector3d rpy;
    int level;
    double score;

    Node(Eigen::Vector3d t, Eigen::Vector3d r, int l, double s)
        : translation(t), rpy(r), level(l), score(s) {}
};

struct CompareNode {
    bool operator()(const Node& a, const Node& b) {
        return a.score < b.score; 
    }
};


Node runBranchAndBound(
    const VoxelMap& map,
    const std::vector<Eigen::Vector3d>& scan,
    int level_max,
    double voxel_size,
    double angle_step,
    Eigen::Vector3d search_min,
    Eigen::Vector3d search_max,
);

