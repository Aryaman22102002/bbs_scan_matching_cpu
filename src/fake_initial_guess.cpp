#include <iostream>
#include <iomanip>
#include <sstream>
#include <map>
#include <Eigen/Dense>
#include <fstream>
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
    writePointCloud("overlay_map.xyz", map_points);

    std::vector<Eigen::Vector3d> fake_guesses = {
        {2.5, 0.5, 0.0},   // close to GT
        {0.0, 0.0, 0.0},   // way off
        {5.0, 0.0, 0.0},   // too far ahead
        {2.5, 2.0, 0.0},   // lateral offset
        {2.5, -1.5, 0.0}   // other side
    };

    int frame = 5;
    auto scan = readKittiBin(makeKittiPath(frame));
    std::cout << "Loaded scan " << frame << " with " << scan.size() << " points\n";

    int level_max = 1;
    double voxel_size = 0.5;
    double angle_step = M_PI / 6;

    for (size_t i = 0; i < fake_guesses.size(); ++i) {
        Eigen::Vector3d guess = fake_guesses[i];
        double margin = 0.5;

        Eigen::Vector3d search_min = guess - Eigen::Vector3d(margin, margin, 0.0);
        Eigen::Vector3d search_max = guess + Eigen::Vector3d(margin, margin, 0.0);

        Node result = runBranchAndBound(map, scan, level_max, voxel_size, angle_step, search_min, search_max);
        std::cout << "[Guess " << i << "] Translation: " << result.translation.transpose()
                  << " | RPY: " << result.rpy.transpose()
                  << " | Score: " << result.score << "\n";

        std::ostringstream traj_name;
        traj_name << "estimated_trajectory_guess" << i << ".csv";
        std::ofstream traj_out(traj_name.str());
        traj_out << "frame,x,y,z,roll,pitch,yaw,score\n";
        traj_out << frame << ","
                 << result.translation.x() << ","
                 << result.translation.y() << ","
                 << result.translation.z() << ","
                 << result.rpy.x() << ","
                 << result.rpy.y() << ","
                 << result.rpy.z() << ","
                 << result.score << "\n";
        traj_out.close();

        auto transformed = transformScan(scan, result.translation, result.rpy);
        std::ostringstream overlay_name;
        overlay_name << "overlay_scan_" << std::setw(2) << std::setfill('0') << frame
                     << "_guess" << i << ".xyz";

        std::vector<Eigen::Vector3d> combined = map_points;
        combined.insert(combined.end(), transformed.begin(), transformed.end());
        writePointCloud(overlay_name.str(), combined);
    }

    return 0;
}
