#include "branch_and_bound_search.hpp"
#include "score.hpp"
#include "transform_utils.hpp"
#include <functional>
#include <queue>
#include <cmath>
#include <iostream>
#include <limits>
#include <chrono>


double computeUpperBound(
    const VoxelMap& map,
    const std::vector<Eigen::Vector3d>& scan,
    const Eigen::Vector3d& translation,
    const Eigen::Vector3d& rpy
) {
    return computeScore(map, scan, translation, rpy);
}


std::vector<Node> branchNode(const Node& parent, int next_level, double step_size) {
    std::vector<Node> children;

    double offset = step_size / 2.0;
    std::vector<int> offsets = {-1, 1};
    std::vector<double> yaw_offsets = { -M_PI / 36, M_PI / 36 }; 

    for (int dx : offsets) {
        for (int dy : offsets) {
            for (int dz : offsets) {
                for (double dyaw : yaw_offsets) {
                    Eigen::Vector3d new_translation = parent.translation + offset * Eigen::Vector3d(dx, dy, dz);
                    Eigen::Vector3d new_rpy = parent.rpy + Eigen::Vector3d(0.0, 0.0, dyaw);

                    Node child(new_translation, new_rpy, next_level, 0.0);
                    children.push_back(child);
                }
            }
        }
    }

    return children;
}


Node runBranchAndBound(
    const VoxelMap& map,
    const std::vector<Eigen::Vector3d>& scan,
    int level_max,
    double voxel_size,
    double angle_step,
    Eigen::Vector3d search_min,
    Eigen::Vector3d search_max,
) {
    using Queue = std::priority_queue<Node, std::vector<Node>, CompareNode>;
    Queue queue;

    double base_step = std::pow(2, level_max) * voxel_size;
    double best_score = -1.0;
    Node best_node(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, -1.0);


	int x_steps = static_cast<int>((search_max.x() - search_min.x()) / base_step);
	int y_steps = static_cast<int>((search_max.y() - search_min.y()) / base_step);
	int z_steps = static_cast<int>((search_max.z() - search_min.z()) / base_step);
	int yaw_steps = static_cast<int>((2 * M_PI) / angle_step);

        std::cout << "Starting initial node generation..." << std::endl;

	for (int xi = 0; xi <= x_steps; ++xi) {
	    double x = search_min.x() + xi * base_step;

	    for (int yi = 0; yi <= y_steps; ++yi) {
		double y = search_min.y() + yi * base_step;

		for (int zi = 0; zi <= z_steps; ++zi) {
		    double z = search_min.z() + zi * base_step;

		    for (int yaw_i = 0; yaw_i < yaw_steps; ++yaw_i) {
		        std::cout << "Trying pose: " << xi << ", " << yi << ", " << zi << ", yaw: " << yaw_i << std::endl;

		        double yaw = -M_PI + yaw_i * angle_step;

		        Eigen::Vector3d t(x, y, z);
		        Eigen::Vector3d rpy(0.0, 0.0, yaw);
		        double ub_score = computeUpperBound(map, scan, t, rpy);

		        Node node(t, rpy, level_max, ub_score);
		        queue.push(node);
		    }
		}
	    }
	}

    std::cout << "Initial queue size: " << queue.size() << std::endl;

    int expanded = 0;
    int max_nodes = 5000;  


    while (!queue.empty()) {
        if (expanded >= max_nodes) {
            std::cout << "Reached expansion cap: " << max_nodes << "\n";
            break;
        }

        Node current = queue.top();
        queue.pop();
        expanded++;

        if (expanded % 100 == 0) {
            std::cout << "Expanded " << expanded << " nodes...\n";
        }

        if (current.level <= 1) {
            auto t1 = std::chrono::high_resolution_clock::now();
            double score = computeScore(map, scan, current.translation, current.rpy);
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> dt = t2 - t1;

            std::cout << "[Level 0] Score: " << score 
                      << " | Pose: " << current.translation.transpose()
                      << " | Time: " << dt.count() << " sec\n";

            if (score > best_score) {
                best_score = score;
                best_node = current;
            }
        } else {
            double child_step = base_step / std::pow(2, level_max - current.level + 1);
            std::vector<Node> children = branchNode(current, current.level - 1, child_step);

            for (auto& child : children) {
                child.score = computeUpperBound(map, scan, child.translation, child.rpy);
                if (child.score > best_score) {
                    queue.push(child);
                }
            }
        }
    }

    std::cout << "Total nodes expanded: " << expanded << "\n";
    std::cout << "Best score found: " << best_score << "\n";

    return best_node;
}
