# bbs_scan_matching_cpu


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
   ┣ 📂Fake_initial_Guess_Full_Search_Results_Generation
   ┃ ┗ 📜00.txt
   ┃ ┗ 📜ATE_And_AOE_Fake_Initial_Guess.py
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
   ┣ 📂Full_Search_Results_Generation
   ┃ ┗ 📜ATE_And_AOE_Full_Search.py                      
   ┃ ┗ 📜estimated_trajectory_fullsearch.csv
   ┃ ┗ 📜overlay_scan_fullsearch.xyz
   ┣ 📂Ground_Truth_Poses_KITTI
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
   ┣ 📂Perturbed_Scan_Full_Search_Results_Generation
   ┃ ┗ 📜00.txt
   ┃ ┗ 📜ATE_And_AOE_Perturbed_Scans.py
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
   ┣ 📂include
   ┃ ┗ 📜branch_and_bound_search.hpp
   ┃ ┗ 📜kitti_io.hpp
   ┃ ┗ 📜score.hpp
   ┃ ┗ 📜transform_utils.hpp
   ┃ ┗ 📜voxel_map.hpp
   ┣ 📂src                                    
     ┃ ┣ 📜branch_and_bound_search.cpp
     ┃ ┣ 📜fake_initial_guess.cpp
     ┃ ┣ 📜full_search_main.cpp
     ┃ ┣ 📜kitti_io.cpp
     ┃ ┣ 📜perturbed_scans_main.cpp
     ┃ ┣ 📜score.cpp
     ┃ ┣ 📜transform_utils.cpp
     ┃ ┣ 📜voxel_map.cpp
   ┣ 📜LICENSE
   ┣ 📜Project_Presentation
   ┣ 📜README.md
``` 


