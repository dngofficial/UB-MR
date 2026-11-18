# Lincoln LiDAR Alignment — Investigation Progress

## Project Objective

Develop a one-time ROS2-based alignment process between the
Lincoln vehicle's LiDAR observations and the UB-MR Digital Twin.

The alignment should estimate the initial vehicle position and
orientation before the vehicle starts moving.

## Oakley's Requirements

- Estimate both translation and rotation.
- Determine an approximate initial vehicle location.
- Implement the alignment process as a ROS2 node outside Unity.
- Use point cloud data near the vehicle rather than the entire map.
- Perform alignment once before vehicle movement.

Oakley also suggested investigating existing localization
capabilities instead of implementing registration from scratch.

## Completed Investigation

### 1. Lincoln ROS2 Bag

- Inspected the Lincoln ROS2 bag and relevant sensor topics.
- Identified stationary vehicle intervals.
- Extracted a LiDAR scan at approximately 10 seconds.
- Cropped the scan to a 30-meter radius.
- Saved approximately 31,110 points.

File:
`current_alignment/lincoln_lidar_10s_30m.csv`

### 2. Reference Point Cloud Maps

Inspected the Autoware and Unity reference PCD maps.

- Autoware map: approximately 12.43 million points.
- Unity map: approximately 6.99 million points.
- Generated a smaller Unity point cloud sample.

File:
`current_alignment/pointcloud_uniform_50k.pcd`

The exact transformation between the map coordinate frames
has not yet been verified.

### 3. Sensor Calibration and Coordinate Frames

Investigated:

- base_link
- sensor_kit_base_link
- velodyne_top
- GNSS odometry coordinates
- Autoware map coordinates

The complete transformation from GNSS coordinates to the
local point cloud map frame remains unresolved.

### 4. Point Cloud Visualization

Generated a point cloud comparison visualization.

File:
`current_alignment/pointcloud_bev_comparison.png`

This visualization does not represent a validated
Lincoln-to-Unity registration result.

## Current Direction

Investigate the existing Lincoln Autoware localization pipeline.

The priority is to determine:

1. How the existing system obtains an approximate initial pose.
2. How that pose is expressed in the reference map frame.
3. Whether existing NDT/localization components can refine it.
4. How to integrate the resulting alignment into a ROS2 node.

## Not Yet Implemented

- Verified automatic initial pose estimation
- Validated Lincoln-to-Unity point cloud registration
- Final translation and rotation estimation
- One-time ROS2 alignment node
- Unity integration and end-to-end testing

## Collaboration

The initial LiDAR extraction, map inspection, and coordinate-frame
investigation have been performed locally.

The next phase involves investigating existing localization
components and coordinating ROS2 integration.

## Notes

The full Lincoln ROS2 bag and complete UB-MR Unity project
are not included in this package.

This archive contains investigation artifacts, not a completed
alignment implementation.
