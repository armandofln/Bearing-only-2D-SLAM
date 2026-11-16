# Bearing-Only SLAM (2D)

University project for the **Probabilistic Robotics** course at Sapienza University of Rome.

The goal is to recover the trajectory of a robot and the positions of the landmarks it observed, starting from bearing-only measurements (which represent the direction of a landmark, without its distance) and noisy odometry.

Everything is written in **GNU Octave**.

## Approach

The pipeline in `02-BearingOnlySLAM/main.m` has three stages.

1. **Initial poses from odometry** (`odometry.m`)
   The robot poses are obtained by chaining the given transitions, starting from the first pose of the ground truth.

2. **Initial landmarks by triangulation** (`triangulation.m`)
   A single bearing cannot locate a landmark, so each landmark is initialized as the intersection of two bearing lines. The first line comes from the first observation of the landmark. The second comes from the first later observation that is "good enough": the two bearing directions must differ by at least 0.3 rad and the two poses must be at least 0.3 m apart. This avoids nearly parallel lines and unstable intersections.

3. **Least-squares optimization** (`LS_optimization.m`)
   A Gauss-Newton style optimizer with damping refines poses and landmarks together.
   - Measurement model: the predicted bearings of the landmarks are computed in the robot frame using `atan2()`. The error is defined as the difference from the measured bearing.
   - The Jacobians with respect to the robot pose and the landmark position are derived analytically.
   - Outliers are rejected by **chi-square gating**: measurements whose squared error exceeds a threshold are skipped in that iteration.
   - Poses are updated on the manifold (`SE(2)` composition) and landmarks additively.


## Results

Estimated poses and landmarks compared with the initial guess and the ground truth, after 200 iterations.

| Poses | Landmarks |
| :---: | :---: |
| ![Poses](output/plot%20poses%20200%20iterations.png) | ![Landmarks](output/plot%20landmarks%20200%20iterations.png) |
| ![Poses, separate plots](output/subplot%20poses%20200%20iterations.png) | ![Landmarks, separate plots](output/subplot%20landmarks%20200%20iterations.png) |

In the combined plots the estimate is green, the initial guess is blue and the ground truth is red.

## Repository structure

```
.  
├── 02-BearingOnlySLAM  
│   ├── geometry_helpers_global_to_local.m  
│   ├── LS_optimization.m  
│   ├── main.m  
│   ├── odometry.m  
│   ├── triangulation.m  
│   └── visualization.m  
├── datasets  
│   ├── slam2D_bearing_only_ground_truth.g2o  
│   └── slam2D_bearing_only_initial_guess.g2o  
├── output  
│   ├── plot landmarks 200 iterations.png  
│   ├── plot poses 200 iterations.png  
│   ├── subplot landmarks 200 iterations.png  
│   └── subplot poses 200 iterations.png  
└── tools/

```

where
- `slam2D_bearing_only_ground_truth.g2o` and `slam2D_bearing_only_initial_guess.g2o` are the dataset used in input to this project, respectively the ground truth and the measurements from the odometer and the bearing sensor
- `main.m ` is the entry point to the project
- `odometry.m` computes the initial poses from the odometry sensor
- `triangulation.m` computes the initial landmark positions from bearing lines and initial poses
- `LS_optimization.m` computes the errors and the Jacobians, and applies the least-squares loop
- `geometry_helpers_global_to_local.m` helper script to transform homogeneous transformation matrix in SE(2) into local Lie vector, and viceversa
- `visualization.m` plotting tool
- `tools/` a folder of utility script that was provided by the University

## How to run

Requirements: GNU Octave.

```bash
cd 02-BearingOnlySLAM
octave main.m
```

`main.m` loads `../datasets/slam2D_bearing_only_initial_guess.g2o` and `../datasets/slam2D_bearing_only_ground_truth.g2o`, runs the optimization and opens the plots.
