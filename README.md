# AdaProcrustes-SLAM: Adaptive Procrustes-Based SLAM with Noise-Aware Graduated Non-Convexity

A robust LiDAR-based SLAM system that combines Procrustes alignment, adaptive Graduated Non-Convexity (GNC), and Riemannian optimization for accurate robot localization in challenging environments.

## Overview

AdaProcrustes-SLAM aligns LiDAR point clouds using Orthogonal Procrustes analysis (via SVD) to compute optimal rotation and translation from point correspondences. It introduces an adaptive GNC framework that dynamically adjusts the robustness threshold based on real-time LiDAR noise estimates (e.g., intensity, distance, or local surface variance).

This allows the system to converge reliably even under poor initialization, high outlier rates, or dynamic scenes.

## Methodology

### 1. Data Preprocessing

* Extract point clouds from LiDAR scans.
* Estimate per-point uncertainty using:

  * Distance to sensor (noise increases with range)
  * Intensity values (low reflectivity implies higher uncertainty)
  * Local surface roughness (via PCA on k-nearest neighbors)

### 2. Correspondence Matching

* Use KD-tree or voxel-based search for nearest neighbor matching between scans.
* Apply geometric consistency filtering (e.g., normal alignment constraints).

### 3. Adaptive GNC Framework

* Replace fixed annealing schedules with a noise-adaptive shape parameter (σ).
* At each iteration:

  * Compute residuals for correspondences
  * Set σ proportional to the median noise level
  * Optionally refine σ using sampling-based or B-spline approaches (e.g., SAC-GNC)

### 4. Procrustes Alignment

Solve:
min ||A − BQ||² subject to QᵀQ = I

Closed-form solution via SVD:
Q = UVᵀ, where UΣVᵀ = BᵀA

Extract rotation and translation for SE(3) pose updates.

### 5. Riemannian Optimization

* Refine pose using Riemannian SGD or Proximal Riemannian ADMM.
* Ensures updates remain on SO(3), avoiding orthogonality drift.

### 6. Matrix Splitting and Proximal Updates

* Decompose optimization using ADMM into:

  * Correspondence estimation
  * Transformation update
  * Outlier rejection
* Use proximal operators to enforce sparsity and smoothness.

## Novelty

* Integration of noise-aware adaptive GNC with Procrustes alignment in SLAM.
* Real-time sensor noise modeling improves robustness compared to fixed GNC.
* Combines closed-form alignment, adaptive robustness, and manifold optimization.
* Demonstrates improved performance over G-ICP and FGR in low-overlap and high-noise scenarios.

## Applications

* Autonomous navigation in GPS-denied environments (e.g., mines, tunnels)
* Urban SLAM with dynamic objects and repetitive structures
* Long-term mapping with reduced drift

## Requirements

* LiDAR sensor (e.g., Velodyne, Ouster)
* ROS 2 or standalone C++/Python setup
* Libraries:

  * Eigen
  * Open3D
  * PCL
  * Sophus (for SE(3) operations)

## Usage

### Installation

```bash
git clone https://github.com/yourname/AdaProcrustes-SLAM.git
cd AdaProcrustes-SLAM
pip install -r requirements.txt
```

### Configuration

Edit `config/slam_config.yaml`:

```yaml
lidar_topic: "/laser_scan"
adaptive_gnc: true
noise_model: "intensity+distance"
procrustes_enabled: true
riemannian_optimization: true
```

### Run

```bash
python3 src/main.py --bag data/kitti.bag
```

With ROS 2:

```bash
ros2 launch adaprocrustes_slam mapping.launch.py
```

### Output

* `output/trajectory.csv`: Estimated trajectory
* `output/map.pcd`: Reconstructed point cloud map

## References

* SAC-GNC (ICCV 2025): Adaptive annealing via sampling
* AGNC-PGO (arXiv 2023): B-spline-based adaptive GNC
* Proximal Riemannian ADMM (arXiv 2024): Non-convex pose graph optimization with convergence guarantees
