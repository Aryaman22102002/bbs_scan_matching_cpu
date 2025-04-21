# bbs_scan_matching_cpu


### Table of Content
* [About the Project](#about-the-project)
* [File Structure](#file-structure)
  * [Key Features](#key-features)
  * [Image Of The Robot](#image-of-the-robot)
  * [Frame Assignment For The Robot Along With Its DH Parameters](#frame-assignment-for-the-robot-along-with-its-dh-parameters)
  * [DH Table](#dh-table)
* [Constraints Imposed On The Robot](#constraints-imposed-on-the-robot)
* [Our Approach](#our-approach)
  * [The Main Pick and Place Code](#the-main-pick-and-place-code)
  * [The Numerical Inverse Kinematics Code](#the-numerical-inverse-kinematics-code)
  * [The Trajectory Planning Code](#the-trajectory-planning-code)
  * [The Obstacle Avoidance Code](#the-obstacle-avoidance-code)
* [Some-Information-Regarding-What-Each-Code-File-Does](#some-information-regarding-what-each-code-file-does)
* [Results](#Results)
  * [Pick and Place In MATLAB Simulation](#pick-and-place-in-matlab-simulation)
  * [Pick and Place Using The Real PincherX100](#pick-and-place-using-the-real-pincherx100)
  * [Pick and Place Along With Obstacle Avoidance](#pick-and-place-along-with-obstacle-avoidance)
* [Acknowledgements](#acknowledgements)
* [Contributors](#contributors)
* [Contact](#contact)

  
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




### Acknowledgements
I want to thank my course instructor, Prof. Michael Everett, as well as the teaching assistant Adarsh Salagame who helped me immensely not only while doing this project but throughout the course. They were great at teaching and managing the course and were always available and enthusiastic about solving everyone's doubts.
