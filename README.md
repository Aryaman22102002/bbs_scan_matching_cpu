# bbs_scan_matching_cpu


### Table of Content
* [About the Project](#about-the-project)
* [File Structure](#file-structure)
* [Setup and Build Instructions](#setup-and-build-instructions)
  * [Prerequisites](#prerequisites)
  * [Build Instructions](#build-instructions)
  * [Run the Full Search Localization](#run-the-full-search-localization)
* [Usage Examples](#usage-examples)
* [Results](#results)
* [Acknowledgements](#acknowledgements)


  
### About the Project
As part of my EECE5550 Mobile Robotics course's project at Northeastern University, I reproduced a simplified version of the CPU-only implementation of the 3D-BBS (Branch-and-Bound Scan Matching) algorithm for global localization using LiDAR data. This project aligns 3D LiDAR scans to a voxelized map by exhaustively searching a 4D pose space (x, y, z, yaw) and scoring candidates using voxel occupancy overlap.

This algorithm was originally proposed in the 3D-BBS: Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound paper.
Link to the paper - https://ieeexplore.ieee.org/document/10610810

This implementation focuses on:

- Efficient voxel hashing and occupancy-based scoring.

- Hierarchical BnB search with pruning.

- Tested on real-world LiDAR data from the KITTI Odometry dataset.

Includes tools for:

- Fake initialization experiments.

- Perturbation robustness testing.

- Absolute Trajectory Error (ATE) & Angular Orientation Error (AOE) evaluation.




### File Structure
```
   ┣ 📂Fake_initial_Guess_Full_Search_Results_Generation       # Stores CSV/visual outputs for scan 5 localized from multiple fake initial positions.
   ┃ ┗ 📜00.txt                                                # Ground truth pose file for KITTI Sequence 00, used for ATE and AOE calculations.
   ┃ ┗ 📜ATE_And_AOE_Fake_Initial_Guess.py                     # Computes ATE & AOE metrics for fake-initialized scan localization results.
   ┃ ┗ 📜estimated_trajectory_guess0.csv
   ┃ ┗ 📜estimated_trajectory_guess1.csv
   ┃ ┗ 📜estimated_trajectory_guess2.csv
   ┃ ┗ 📜estimated_trajectory_guess3.csv
   ┃ ┗ 📜estimated_trajectory_guess4.csv
   ┃ ┗ 📜overlay_map.xyz
   ┃ ┗ 📜overlay_scan_05_guess0.xyz
   ┃ ┗ 📜overlay_scan_05_guess1.xyz
   ┃ ┗ 📜overlay_scan_05_guess2.xyz
   ┃ ┗ 📜overlay_scan_05_guess3.xyz
   ┃ ┗ 📜overlay_scan_05_guess4.xyz
   ┣ 📂Full_Search_Results_Generation                         # Contains output from unconstrained (wide 4D) BnB scan matching. 
   ┃ ┗ 📜ATE_And_AOE_Full_Search.py                           # Evaluates ATE & AOE for the result of a full-space scan alignment.
   ┃ ┗ 📜estimated_trajectory_fullsearch.csv
   ┃ ┗ 📜overlay_scan_fullsearch.xyz
   ┣ 📂Ground_Truth_Poses_KITTI                               # Contains KITTI-provided GT poses (00.txt to 10.txt) used for error evaluation.
   ┃ ┗ 📂poses
   ┃ ┃ ┗ 📜00.txt
   ┃ ┃ ┗ 📜01.txt
   ┃ ┃ ┗ 📜02.txt
   ┃ ┃ ┗ 📜03.txt
   ┃ ┃ ┗ 📜04.txt
   ┃ ┃ ┗ 📜05.txt
   ┃ ┃ ┗ 📜06.txt
   ┃ ┃ ┗ 📜07.txt
   ┃ ┃ ┗ 📜08.txt
   ┃ ┃ ┗ 📜09.txt
   ┃ ┃ ┗ 📜10.txt
   ┣ 📂Perturbed_Scan_Full_Search_Results_Generation         # Contains results for full-space localization from perturbed (pre-offset) scans.
   ┃ ┗ 📜00.txt                                              # Ground truth pose file for KITTI Sequence 00, used for ATE and AOE calculations on perturbed scans.
   ┃ ┗ 📜ATE_And_AOE_Perturbed_Scans.py                      # Script to compute ATE & AOE for various perturbed scan inputs.
   ┃ ┗ 📜estimated_trajectory_perturb0.csv
   ┃ ┗ 📜estimated_trajectory_perturb1.csv
   ┃ ┗ 📜estimated_trajectory_perturb2.csv
   ┃ ┗ 📜estimated_trajectory_perturb3.csv
   ┃ ┗ 📜estimated_trajectory_perturb4.csv
   ┃ ┗ 📜overlay_scan_05_perturb0.xyz
   ┃ ┗ 📜overlay_scan_05_perturb1.xyz
   ┃ ┗ 📜overlay_scan_05_perturb2.xyz
   ┃ ┗ 📜overlay_scan_05_perturb3.xyz
   ┃ ┗ 📜overlay_scan_05_perturb4.xyz
   ┣ 📂include                                               # Header files defining core components of the BnB scan matcher.
   ┃ ┗ 📜branch_and_bound_search.hpp                         # Declares the runBranchAndBound() function and Node structure.
   ┃ ┗ 📜kitti_io.hpp                                        # Functions for loading binary LiDAR data from KITTI .bin files.
   ┃ ┗ 📜score.hpp                                           # Voxel overlap-based scoring function for scan-to-map alignment.
   ┃ ┗ 📜transform_utils.hpp                                 # Utilities for transforming point clouds using RPY-based SE(3) poses.
   ┃ ┗ 📜voxel_map.hpp                                       # Voxel hashing and occupancy map representation using spatial hashing.
   ┣ 📂src                                                   # C++ source files implementing the global localization pipeline.
     ┃ ┣ 📜branch_and_bound_search.cpp                       # Core BnB logic with hierarchical branching, pruning, and scoring.
     ┃ ┣ 📜fake_initial_guess.cpp                            # BnB run with multiple fake scan poses to test robustness to initialization.
     ┃ ┣ 📜full_search_main.cpp                              # Full 4D pose space scan matching for a single frame (e.g., scan 5).
     ┃ ┣ 📜kitti_io.cpp                                      # Implements readKittiBin() to load .bin LiDAR scan files.
     ┃ ┣ 📜perturbed_scans_main.cpp                          # Tests localization accuracy for perturbed (artificially offset) scans.
     ┃ ┣ 📜score.cpp                                         # Implements the scan-to-map scoring function using voxel intersection.
     ┃ ┣ 📜transform_utils.cpp                               # Functions for applying pose transforms to 3D scans.
     ┃ ┣ 📜voxel_map.cpp                                     # Hash map-based voxelization and map point insertion logic.
   ┣ 📜LICENSE
   ┣ 📜Project_Presentation
   ┣ 📜README.md
``` 

### Setup and Build Instructions
#### Prerequisites

Make sure the following dependencies are installed:
- C++17 compiler (e.g., g++-9 or newer)
- Eigen3 (tested with Eigen 3.4)
- CMake (optional, for larger builds)
- Python 3 (for analysis scripts)
- Python packages: numpy, matplotlib, pandas, scipy, seaborn

#### Build Instructions

```bash
git clone https://github.com/yourusername/bbs_scan_matching_cpu.git
cd bbs_scan_matching_cpu
g++ -std=c++17 src/full_search_main.cpp src/voxel_map.cpp src/transform_utils.cpp src/score.cpp src/branch_and_bound_search.cpp src/kitti_io.cpp -Iinclude -I/usr/include/eigen3 -o run_test
```

#### Run the Full Search Localization:
``` ./run_test```  <br>

Make sure KITTI Sequence 00 Velodyne data is located in sequences/00/velodyne/ inside the project root.


### Usage Examples
Run full 4D scan-to-map alignment on Scan 5 using map built from Scans 0–3:<br>

```./run_test```<br>

Sample Output:
```bash
Loaded scan 5 with 123924 points
[Level 0] Score: 0.994973 | Pose: 1 0 0 | Time: 0.23 sec
Total nodes expanded: 900
Best score found: 0.994973
Translation: 1.0 0.0 0.0
RPY: 0.0 0.0 0.0
Score: 0.994973
Time taken: 286.829 seconds
```

Other Outputs:
- ```overlay_scan_fullsearch.xyz``` – Combined map + aligned scan for visualization
- ```estimated_trajectory_fullsearch.csv``` – Final estimated pose and score

Want to try fake initial guesses or scan perturbations? Compile and Run:
- ```src/fake_initial_guess.cpp```
- ```src/perturbed_scans_main.cpp```


### Results

The system was tested on real LiDAR data from the KITTI Sequence 00. The implementation successfully aligned incoming scans to a pre-built voxel map using a branch-and-bound search over a 4D pose space. Qualitative results showed progressive convergence during scan-to-map alignment, with clear visual overlap even under moderate perturbations or poor initial guesses.

- Progressive scan alignment.
- Robust to small spatial perturbations.
- Sensitive to large yaw errors.
- CPU-only full search was slow, highlighting the need for optimization.

For explanation of the algroithm, quantitative results (ATE, AOE), experimental comparisons, limitations, and future work discussion, please refer to the full project report. <br>

Below is a video that demonstrates global scan-to-map alignment using a CPU-only implementation of the 3D-BBS (Branch-and-Bound Scan Matching) algorithm. <br>

The incoming LiDAR scan (Frame 6) is aligned against a voxelized map built from Frames 0 to 4 of the KITTI Odometry Sequence 00. The visualization shows how the scan gradually aligns to the map as the branch-and-bound traversal progresses, illustrating the effectiveness of the method without relying on GPU acceleration. <br>

Red points represent the transformed scan under different pose guesses. As the Branch-and-Bound search progresses, the scan shifts from misaligned outskirts to a tightly overlapping position, maximizing alignment with the blue voxel map.


https://github.com/user-attachments/assets/cbed788c-8b98-42a7-a673-2f8840c93634


### Acknowledgements
I want to thank my course instructor, Prof. Michael Everett, as well as the teaching assistant Adarsh Salagame who helped me immensely not only while doing this project but throughout the course. They were great at teaching and managing the course and were always available and enthusiastic about solving everyone's doubts.
