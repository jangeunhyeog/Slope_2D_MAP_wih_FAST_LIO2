# Slope 2D Map with Optimized FAST-LIO2

This project provides an **optimized version of FAST-LIO2** tailored for real-time mobile robot navigation on uneven terrain. It integrates a lightweight LiDAR Odometry system with a robust **2.5D Elevation Mapping** and **2D Costmap Generation** pipeline.

## 🚀 Key Features & Optimizations

### 1. Lightweight Fast-LIO2
We have significantly refactored the original Fast-LIO2 codebase to reduce computational load and memory usage for real-time deployment on mobile robots:
- **Direct Downsampling**: Removed complex feature extraction (Surf/Edge) logic. The system now uses direct downsampling, which is faster and sufficient for dense LiDARs (Livox/Velodyne).
- **Dead Code Removal**: Cleaned up unused variables, functions, and complex math related to the legacy feature matching system.
- **Memory Optimization**:
  - Removed high-bandwidth debug publishers: `cloud_effected` (internal debug points) and `Laser_map` (redundant large global pointcloud).
  - This prevents memory usage from exploding during long-duration runs.
  - **Only useful topics** are published: `/cloud_registered` (current scan), `/Odometry` (pose), and `/total_map` (global occupancy grid).

### 2. 2.5D Elevation Mapping (V-Histogram)
Instead of a simple 2D projection, we implement a **2.5D Elevation Map** to handle slopes and uneven terrain correctly.
- **V-Histogram (Vertical Histogram)**: 
  - Each grid cell (20cm x 20cm) accumulates LiDAR points in a vertical histogram.
  - This allows us to distinguish between **ground**, **obstacles**, and **ceiling/overhangs**.
- **Ceiling Filtering**: Points above a certain relative height (e.g., 2.0m) are filtered out to prevent indoor ceilings from appearing as obstacles.
- **Noise Filter**: Cells with fewer than `min_valid_points` are ignored to remove sensor noise.
- **Memory Optimization (Smart Pruning)**:
  - **Visited Check**: We employ a `visited` flag to track processed cells. 
  - **Active Window**: The heavy V-Histogram data is *only* maintained for the active area around the robot. 
  - **pruning**: Once a cell moves out of the active window, its final state (Occupied/Free + Height) is compressed and archived into a lightweight `static_map`, and the raw histogram data is removed from memory. This ensures memory usage remains stable even during large-scale mapping.

### 3. Rule-Based 2D Costmap Generation
A 2D Occupancy Grid (`/total_map`, `/costmap`) is generated from the 2.5D data using specific rules:
- **Slope Detection**:
  - We calculate the normal vector of the local terrain using PCA (Principal Component Analysis) on neighboring cells.
  - If the **slope > 30 degrees**, the cell is marked as an **Obstacle**.
- **Height Step Detection**:
  - If the vertical span of points within a cell (Max Z - Min Z) exceeds a threshold (e.g., 30cm), it is marked as an **Obstacle**.
- **Traversability**:
  - Cells that pass the slope and height checks are marked as **Free Space**.

---

## 🛠️ Architecture

### ROS2 Nodes
1.  **`fastlio_mapping` (fast_lio)**:
    *   Input: Raw LiDAR (`/points_raw`) + IMU (`/imu_raw`)
    *   Output: Odometry (`/Odometry`) + Undistorted PointCloud (`/cloud_registered`)
2.  **`elevation_mapping_node`**:
    *   Input: `/cloud_registered`
    *   Processing: Builds 2.5D V-Histogram, runs Slope/Height rules.
    *   Output: `/total_map` (Global accumulated 2D map), `/map` (Local active window).
3.  **`custom_costmap_node`**:
    *   Input: `/map` (Local map)
    *   Processing: Applies inflation and processes Region of Interest (ROI).
    *   Output: `/costmap` (Final navigation costmap).

---

## 📦 Build & Run

### Prerequisites
- **ROS 2 Humble** (Ubuntu 22.04)
- `livox_ros_driver2` (installed and sourced)
- PCL & Eigen (Standard versions)

### parameters
- All parameters are located in `src/Fast_Lio_VIL/config` and `src/Fast_Lio_VIL/launch`.
- **`avia.yaml`**: LiDAR/IMU Extrinsics and Fast-LIO settings.
- **`elevation_mapping.launch.py`**: Grid resolution, Slope threshold, robot height.

### Build
```bash
cd dev_ws
colcon build --symlink-install
source install/setup.bash
```

### Run
```bash
ros2 launch lio_nav_bringup bringup.launch.py
```
This will launch Fast-LIO, Elevation Mapping, and RViz automatically.

### Visualization (RViz)
The included `fastlio.rviz` configuration is pre-tuned:
- **Total Map** (`/total_map`): Shows the global traversed area (Occupancy Grid).
- **Local Map** (`/map`): Shows the immediate 30m surroundings.
- **Costmap** (`/costmap`): Shows the safety costs/inflation around obstacles.
- **Cloud Registered** (`/cloud_registered`): Shows raw real-time LiDAR points (Green).

---

## 📝 Credits
Based on **FAST-LIO2** (HKU-Mars) and modified for slope-aware 2D mapping applications.
Optimizations and 2.5D Mapping Logic implemented by the maintainer.
