#include <iostream>
#include <iomanip>
#include <sstream>
#include <map>
#include <Eigen/Dense>
#include <fstream>
#include <chrono>
#include "voxel_map.hpp"
#include "transform_utils.hpp"
#include "branch_and_bound_search.hpp"
#include "kitti_io.hpp"

void writePointCloud(const std::string& filename, const std::vector<Eigen::Vector3d>& points) {
    std::ofstream out(filename);
    for (const auto& pt : points)
        out << pt.x() << " " << pt.y() << " " << pt.z() << "\n";
    out.close();
}

std::string makeKittiPath(int frame) {
    std::ostringstream ss;
    ss << "/home/aryaman/3d_bbs/sequences/00/velodyne/"
       << std::setw(6) << std::setfill('0') << frame << ".bin";
    return ss.str();
}

int main() {
    VoxelMap map(0.5);
    std::vector<Eigen::Vector3d> map_points;

    for (int i = 0; i < 4; ++i) {
        auto pts = readKittiBin(makeKittiPath(i));
        map_points.insert(map_points.end(), pts.begin(), pts.end());
        for (const auto& pt : pts)
            map.insertPoint(pt);
    }

    int frame = 5;
    auto scan = readKittiBin(makeKittiPath(frame));
    std::cout << "Loaded scan " << frame << " with " << scan.size() << " points\n";

    int level_max = 1;
    double voxel_size = 0.5;
    double angle_step = M_PI / 6;

    Eigen::Vector3d search_min(-2.0, -2.0, -1.0);
    Eigen::Vector3d search_max(2.0, 2.0, 1.0);

    auto start_time = std::chrono::high_resolution_clock::now();

    Node result = runBranchAndBound(map, scan, level_max, voxel_size, angle_step, search_min, search_max);

    auto end_time = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double>(end_time - start_time).count();

    std::cout << "Full search result:\n";
    std::cout << "Translation: " << result.translation.transpose() << "\n";
    std::cout << "RPY: " << result.rpy.transpose() << "\n";
    std::cout << "Score: " << result.score << "\n";
    std::cout << "Time taken: " << duration << " seconds\n";

    // Save CSV
    std::ofstream traj_out("estimated_trajectory_fullsearch.csv");
    traj_out << "frame,x,y,z,roll,pitch,yaw,score,time\n";
    traj_out << frame << ","
             << result.translation.x() << ","
             << result.translation.y() << ","
             << result.translation.z() << ","
             << result.rpy.x() << ","
             << result.rpy.y() << ","
             << result.rpy.z() << ","
             << result.score << ","
             << duration << "\n";
    traj_out.close();

    // Optional overlay
    auto transformed = transformScan(scan, result.translation, result.rpy);
    std::vector<Eigen::Vector3d> combined = map_points;
    combined.insert(combined.end(), transformed.begin(), transformed.end());
    writePointCloud("overlay_scan_fullsearch.xyz", combined);

    return 0;
}
