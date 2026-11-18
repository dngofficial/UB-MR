# Lincoln point cloud alignment

This integrates Jake's uploaded investigation into UB-MR. It initializes **existing Autoware localization**, captures a localized LiDAR scan through TF, refines a nearby **Unity-map → Autoware-map** transform, and lets Unity apply that correction to the shared environment/cloud parent once before driving.

The implementation and synthetic checks do not establish a physical Lincoln alignment. The Lincoln bag, candidate maps/projectors and an installed image's sensor configuration have now been located; the recording's exact map/calibration and approximate map-to-map relationship still need confirmation. See [the local discovery report](reports/LOCAL_DISCOVERY.md) for verified paths, the exact source scan, frame findings and the installer failure. The working Unity project remains this checkout, not the UB-MR inside `~/UB-DigitalTwin`.

[The follow-up using Jake's calibration](reports/JAKE_FOLLOWUP.md) adds candidate comparison, complete URDF-chain validation, a guarded `robot_state_publisher` replay launch, and a diagnostic geographic projection. It also identifies a recorded **VLP-32-C** versus installed-description **VLS-128** mismatch; no recording calibration is selected automatically.

Original notes and data are preserved in `JAKE_NOTES.md`, `INVESTIGATION_README.md`, `current_alignment/` and the original reports. See [the implementation validation report](reports/IMPLEMENTATION_CHECKS.md) for the earlier implementation checks.

## Reviewing the prototype

This branch includes the ROS2 package, Unity integration, small investigation samples, calibration candidates and validation reports. The full Oakley bag, dense external maps, Python environment and generated build outputs are excluded. Offline and synthetic ROS checks use the included samples/fixtures; physical replay requires your own bag/map inputs.

Absolute paths in the discovery reports describe the machine where the investigation ran. For another checkout, pass its bag directory to `tools/inspect_local_bag.py --bag-dir /your/extracted/bag`; use the path arguments shown by the geographic audit's `--help` for your map and OpenDRIVE files. The bundled configuration remains unconfirmed for the recording. Follow the setup instructions below before testing.

## What Unity actually uses

`Assets/UB_MR/Modules/UB-Service-Center-Loop/UB-Service-Center-Loop.unity` has a `NearbyPointCloudSelector` on `PointCloudRuntime`. Its assigned MeshFilter belongs to **PointCloud_REFERENCE_MANUAL** and uses **pointcloud_uniform_50k.asset**, containing 49,950 points. Its vertex data matches the supplied PCD with importer conversion `(x,y,z) → (x,z,y)` within 0.000005 m. This is a serialized Unity mesh; Unity is not loading Autoware's `pointcloud_map.pcd` automatically.

The selected cloud is parented to `ServiceCenterLoopMR`, alongside the environment geometry. Its existing local placement is position `(2.858025, -2.7407467, 0)` and quaternion `(0.0074047437, -0.707068, -0.707068, 0.0074047437)`. That manual placement is preserved. The open `pointcloud_uniform_50k 2.asset` is a **RawXYZ** variant and is not the selected reference. Do not substitute it without reviewing axes/handedness.

Reproduce this finding from the repo root:

```bash
python3 Integrations/LincolnAlignment/tools/audit_unity_scene.py
```

The audit writes `reports/unity_scene_audit.json`. It verifies mesh identity/import mode, not the physical correctness of the manual placement or a relationship to the missing Autoware map.

## Data and frame contract

| Item | Meaning |
|---|---|
| `/map/pointcloud_map` | Autoware reference map, PointCloud2 in `map`; change topic if your stack differs |
| `/localization/kinematic_state` | Existing Autoware body pose/twist/covariance, parent `map`, child `base_link` |
| `/api/localization/initialization_state` | Existing Autoware AD API initialization state; required by default |
| `/unity/reference_cloud_ros` | Frozen nearby points from the assigned Unity mesh in `unity_map_ros` |
| `/map_alignment/align_once` | `std_srvs/srv/Trigger`; attempt alignment while stationary |
| `/map_alignment/candidate_transform` | Candidate `T_A_U`, mapping Unity map ROS coordinates into Autoware map coordinates; **not a TF broadcast** |
| `/map_alignment/aligned_unity_cloud` | Local reference transformed into Autoware `map`, for RViz comparison |
| `/map_alignment/report` | JSON numerical result and provenance; optionally also saved to disk |

Unity publishes `mesh local → all scene transforms → Unity world → (world.z, -world.x, world.y)`. This includes the imported cloud's actual scale/rotation/translation and its map parent. **LiDAR-local `NearbyRosPoints` remain available for debugging, but are not the map-alignment input.** The selector can use an explicit `Lidar Override` when several agents exist; otherwise it finds runtime `LiDAR_Top`.

There are two distinct starting estimates:

1. A body pose **in Autoware's map frame**, or a GNSS path whose projection/origin is verified, to initialize Autoware's NDT/localization pipeline.
2. An approximate **map-to-map transform** to seed local ICP. Autoware localization does not determine the relationship between arbitrary Unity and map origins. Obtain it from known import/georeferencing information or at least three well-spread corresponding static landmarks. Reserve additional landmarks for independent validation.

The bundled CSV has 31,110 already-cropped points, of which 18,653 are within 20 m. Its exact source scan has now been recovered from the Oakley bag: `velodyne_top`, stamp `1757032460.631107072`. Three recorded cloud topics have identical XYZ, so the extraction topic remains ambiguous. Reuse the bag's original message/stamp for runtime checks; the XYZ CSV alone cannot establish stationarity. The 50k map sample may be too sparse for accurate local alignment; dense candidate maps are listed in the local discovery report.

## Set up and run checks now

Run these commands from the **repository root**. Use ROS Humble/Python 3.10 to match this project. The system Python in this checkout had NumPy 2 with incompatible compiled SciPy dependencies; use an isolated environment instead of changing global packages.

```bash
source /opt/ros/humble/setup.bash
export LINCOLN_ROOT="$PWD/Integrations/LincolnAlignment"
python3 -m venv --system-site-packages "$LINCOLN_ROOT/.venv-ros"
source "$LINCOLN_ROOT/.venv-ros/bin/activate"
python -m pip install -r "$LINCOLN_ROOT/requirements-ros.txt"
export LINCOLN_PYTHON="$LINCOLN_ROOT/.venv-ros/bin/python"

# Uploaded data, mesh/axis audit, transforms, stationary gate, PointCloud2 and ICP
bash "$LINCOLN_ROOT/tools/check.sh" offline

# Build the included ROS package with the same Python interpreter
bash "$LINCOLN_ROOT/tools/check.sh" build
source "$LINCOLN_ROOT/ros2_ws/install/setup.bash"

# Real DDS/services and TF, using SYNTHETIC fixtures in isolated ROS domain 83
bash "$LINCOLN_ROOT/tools/check.sh" ros
```

The ROS check changes its own process's domain, not your terminal's domain. Set `LINCOLN_TEST_DOMAIN_ID` if 83 is occupied. Keep these synthetic checks separate from a running vehicle. `require_initialized=false` is used only by the synthetic fixture because it does not launch Autoware.

In Unity 6000.0.36f1, run **UB-MR → Lincoln Alignment → Run Offline Unity Checks**. It checks basis conversion, composing the correction with an already-placed map parent, subtracting a large display origin in double precision, spherical selection, fixed world coordinates when the simulated LiDAR turns, and little-endian XYZ payload encoding. It does not start a ROS node or change the saved scene.

For a terminal check with the Editor closed:

```bash
"$UNITY_EDITOR" -batchmode -nographics -projectPath "$PWD" \
  -executeMethod CAVAS.UB_MR.PointCloud.Editor.LincolnAlignmentChecks.Run \
  -logFile /tmp/lincoln-unity-checks.log -quit
```

Set `UNITY_EDITOR` to the installed Unity executable. Look for `LINCOLN_ALIGNMENT_CHECKS_PASSED` in the log. The normal Unity licensing/client access and project dependencies are required.

Optional file inspection and ROS preview:

```bash
python "$LINCOLN_ROOT/tools/inspect_inputs.py"
ros2 run rotation_integration file_preview --ros-args \
  -p scan_file:="$LINCOLN_ROOT/current_alignment/lincoln_lidar_10s_30m.csv" \
  -p reference_file:="$LINCOLN_ROOT/current_alignment/pointcloud_uniform_50k.pcd"
```

Preview frames `extracted_scan_unverified` and `unity_pcd_unverified` are separate and have publication-time stamps. Select each separately as RViz's fixed frame; these topics are for inspection only.

## Run against the Lincoln bag

### 1. Verify the installed Autoware setup

Source the matching Autoware workspace as well as this package in every ROS terminal. Launch the team's Lincoln map, sensor model, preprocessing and localization configuration. This repository does not contain enough information to supply that launch command. Match ROS_DOMAIN_ID and RMW_IMPLEMENTATION with Unity; the repository's `Scripts/host_ros2_env.bash` describes its local CycloneDDS setup. Start Unity from that ROS environment so native libraries and DDS settings are inherited.

Inspect the bag before replaying:

```bash
ros2 bag info /path/to/oakley_test_bag
ros2 topic list -t
ros2 service type /api/localization/initialize
ros2 interface show autoware_adapi_v1_msgs/srv/InitializeLocalization
ros2 interface show autoware_adapi_v1_msgs/msg/LocalizationInitializationState
```

Replay the stationary segment with `/clock` and `use_sim_time=true` on Autoware, alignment and snapshot nodes. For example, after confirming the stationary offset:

```bash
ros2 bag play /path/to/oakley_test_bag --clock --start-offset 10
```

Ten seconds is Jake's approximate extraction offset, not a verified exact timestamp or stationary-window boundary. Use an explicit topic allowlist when recomputing localization from recorded sensors, so recorded localization/map-to-body TF outputs do not compete with the running localizer. The allowlist depends on the bag's actual LiDAR, IMU, GNSS and vehicle-state topics. Start subscribers before replay and keep the vehicle stopped through validation/application. The implementation does not control vehicle motion or keep the bag inside its stationary interval automatically.

### 2. Initialize the existing localizer

If the GNSS projection, map origin and sensor extrinsics are verified:

```bash
ros2 run rotation_integration initialize_once --use-gnss \
  --ros-args -p use_sim_time:=true
```

Otherwise copy `config/seed_pose.template.json` to `config/seed_pose.local.json`, fill a measured **base_link pose in Autoware map coordinates** with covariance, and set `confirmed=true`. Then:

```bash
ros2 run rotation_integration initialize_once \
  --seed "$LINCOLN_ROOT/config/seed_pose.local.json" \
  --ros-args -p use_sim_time:=true
```

The client calls the public AD API once. An omitted pose asks Autoware to use its configured GNSS path, as described in [Autoware's initialization API](https://docs.autoware.org/main/design/autoware-architecture-v1/interfaces/ad-api/list/api/localization/initialize/). It does not assume raw GNSS odometry is map coordinates. On timeout the server outcome is unknown; inspect initialization state before retrying.

```bash
ros2 topic echo --once /api/localization/initialization_state
ros2 topic echo --once /localization/kinematic_state
ros2 topic echo --once /actual/lidar/topic --field header
ros2 run tf2_ros tf2_echo map actual_scan_frame
```

The default gate expects Autoware's [INITIALIZED state](https://docs.autoware.org/main/design/autoware-architecture-v1/interfaces/ad-api/types/autoware_adapi_v1_msgs/msg/LocalizationInitializationState/). Verify NDT/EKF diagnostics and actual scan overlap as well. For older incompatible Autoware APIs, adapt to that version's supported initialization interface; do not substitute an arbitrary internal service name.

### 3. Check the real LiDAR and mounting calibration

```bash
ros2 launch rotation_integration snapshot.launch.py \
  cloud_topic:=/actual/lidar/topic expected_scan_frame:=actual_scan_frame \
  map_frame:=map use_sim_time:=true

# In another sourced terminal, with RViz already subscribing:
ros2 service call /localization_snapshot/capture std_srvs/srv/Trigger '{}'
```

Use RViz Fixed Frame `map` and compare `/localization_snapshot/map_cloud` with the Autoware map. The snapshot uses `map ← message.header.frame_id` at the **exact scan timestamp**, including the full TF chain through base_link and the LiDAR mount. It does not add a second manually entered mounting offset. It rejects zero/stale/future stamps and unavailable exact-time TF; it does not substitute the latest pose. For a pre-transformed/concatenated cloud, use its actual header frame. The default crop radius is 20 m about that frame's origin, which may be a body origin rather than the physical LiDAR. This helper does not deskew scans or itself verify stationarity. Outputs are volatile, so start RViz/recorders first.

### 4. Prepare Unity's reference and a map-to-map seed

Open the Service Center Loop scene. `PointCloudRuntime` already has **UnityMapAlignmentBridge** referencing the existing selector and Module. Place the simulated sensor approximately in the corresponding Unity map region **before starting Play**, using the agent's starting pose or selector `Lidar Override`. Selection is frozen on startup; the sensor only chooses the nearby region, not the reference coordinate basis.

The old `PointCloudRoot` test loader still refers to Jake's external absolute path. That separate test object and the unused duplicate cloud are disabled in the saved scene to avoid missing-file errors and ambiguous overlays. The alignment bridge uses the explicitly assigned serialized mesh and needs no external PCD path.

In Play mode, the bridge publishes the frozen nearby cloud once per second. Verify:

```bash
ros2 topic info -v /unity/reference_cloud_ros
ros2 topic echo --once /unity/reference_cloud_ros --field header
ros2 topic info -v /map/pointcloud_map
```

Use **Export Unity Map ROS CSV** on the selector to save `unity_map_ros.csv` under Unity's `Application.persistentDataPath` (printed in the Console). These are the fixed-world coordinates appropriate for map-to-map inspection. The older **Export Nearby Points CSV** writes LiDAR-local debug coordinates and is not interchangeable.

Fill `config/landmarks.template.json` with corresponding points already expressed in `unity_map_ros` and Autoware `map`, save it as `config/landmarks.local.json`, then:

```bash
python "$LINCOLN_ROOT/tools/seed_from_landmarks.py" \
  --landmarks "$LINCOLN_ROOT/config/landmarks.local.json" \
  --output "$LINCOLN_ROOT/config/map_alignment.local.json"
```

Check residuals, coordinate units/handedness, held-out landmarks and transform direction; only then set `confirmed=true` in the result. A known importer/georeferencing transform can instead populate `initial_autoware_from_unity_ros` directly. The shipped template intentionally has no numeric seed. Local ICP cannot recover an arbitrary initial position/origin by itself.

### 5. Align once, inspect and apply

```bash
ros2 launch rotation_integration alignment.launch.py \
  config_file:="$LINCOLN_ROOT/config/map_alignment.local.json" \
  use_sim_time:=true \
  output_report:="$LINCOLN_ROOT/reports/local_alignment.json"

ros2 service call /map_alignment/align_once std_srvs/srv/Trigger '{}'
ros2 topic echo --once --qos-durability transient_local /map_alignment/report
```

The service requires both cloud frame IDs, initialized localization, a fresh body pose with valid covariance, and a continuous two-second stationary window. Default limits are linear speed 0.1 m/s, angular speed 0.05 rad/s, pose gap 0.5 s, drift 0.2 m / 2 degrees, position standard deviation 2 m, yaw standard deviation 15 degrees. These are configurable engineering starting values, not site-calibrated quality guarantees. A clock reset, motion, bad covariance or localization gap restarts the window.

It crops nearby geometry around the localized vehicle and the inverse-seeded Unity location, runs the uploaded Open3D multiscale rigid ICP, and checks overlap/RMSE/refinement limits and nonplanar geometry. It estimates translation and rotation. It caches incoming reference messages, so a full 12.4-million-point map can consume substantial memory; prefer a dense local reference topic/crop if necessary. For Autoware map loaders offering only differential loading services, expose the matching local map cloud first; this node does not implement those loader services.

The Autoware map subscription requests reliable/transient-local QoS by default. If its publisher is volatile, launch with `autoware_transient_local:=false` and republish after subscription. Unity publishes reliable/volatile, repeated at 1 Hz; the service accepts that stream. Output topics are reliable/transient-local; set RViz PointCloud2 durability accordingly. Compare `/map_alignment/aligned_unity_cloud` against `/map/pointcloud_map` in fixed frame `map`, including independent building corners and ground height. Good ICP metrics can still describe an incorrect local minimum.

Before applying in Unity, set **Autoware Origin X/Y/Z** on UnityMapAlignmentBridge to the exact map-coordinate origin subtracted by the vehicle's `/ub_mr/localization` bridge, then enable **Display Origin Confirmed**. Use `(0,0,0)` only if that vehicle output already uses unmodified Autoware map coordinates. The mr_pkg submodule is empty in this checkout, so its current offsets/z handling cannot be verified here; obtain that configuration or use a bridge preserving the full pose with a known origin. Flattened vehicle Z cannot be repaired by map registration alone.

With the vehicle still stopped, use **Apply Candidate Alignment** on the bridge component. Auto-apply is available but off by default. The world correction is `C · Translate(-display_origin) · T_A_U · C^-1`, composed with the map parent's pose at capture. Origin subtraction happens before conversion to Unity float. `Module.SetMapPoseFromAlignment` updates the map controls so the next UI frame preserves the result. The FBX and parented cloud move together; the vehicle stays outside that parent.

The candidate stamp identifies the frozen Unity snapshot; it is intentionally not a measurement-time TF stamp. Unity checks it and both frame IDs, rejects stale-session candidates and changed references, and applies once. Accepted ROS alignment also runs once per process. To repeat after changing a map, seed, crop, or scene pose, restart the ROS node **and** Unity Play mode. Stop Play mode to return to the saved scene placement. Numerical reports are saved when requested; applying/saving a reusable calibration preset is a separate follow-up.

## What remains to confirm

The bag, candidate maps, installed-image sensor calibration and reference mr_pkg bridge have now been found locally; see [the local discovery report](reports/LOCAL_DISCOVERY.md). The original investigation ZIP supplied afterward has identical CSV/PCD data, so it does not replace the integrated implementation.

- Confirm which physical map/version was used when recording. The MGRS `~/host_data/ub_hdmap` candidate is distinct from the Local proving-ground/CARLA maps.
- Confirm that the installed image's calibration matches the recording, including GNSS lever-arm/frame conventions. The bag contains no recorded TF or Autoware localization.
- Establish a valid Autoware-localized initial pose and inspect diagnostics/scan overlap; raw GNSS odometry is not a map-frame body pose.
- Establish the selected Unity map's relationship to the physical map from provenance/georeferencing or corresponding landmarks, with held-out features for validation.
- Verify the deployed display bridge and target origin/Z treatment. The inspected reference bridge flattens Z and assumes modulo-100km MGRS coordinates.
- Resolve the separately reproduced v1.1.0 installer map-release failure if a fresh simulator install is needed. Existing local inputs/images suffice for the current read-only checks.

Suggested team split from the conversation: Jake verifies map provenance, landmarks and initial localization; Devan verifies ROS topics/TF/QoS and bag replay. Together inspect the held-out physical features before accepting a real transform. No real alignment matrix is fabricated from the sketch or the standalone XYZ files.
