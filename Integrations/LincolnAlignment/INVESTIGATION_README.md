# Lincoln alignment — combined investigation and ROS 2 integration

Jake's original investigation is preserved in `JAKE_NOTES.md`, and all three files in `current_alignment/` are unchanged. This bundle adds an input audit, separate-frame ROS preview, an Autoware initialization client, the timestamped scan snapshot node, and a one-shot **map-to-map alignment service** matching the sketch.

**Status:** real input parsing and offline tests work. Autoware, ROS 2 middleware and Unity have not been run in this environment. No validated real Lincoln-to-Unity transform has been computed. The archive does not include the Autoware point-cloud map, original ROS bag, sensor calibration, or Unity project.

## What the sketch requires

There are two separate transforms, and two different approximate starting estimates:

1. **Vehicle in the Autoware map:** initialize the existing Autoware localization pipeline, using its correctly configured GNSS path or an approximate `base_link` pose in the Autoware `map` frame. Autoware performs the localization. The `initialize_once` client calls its public AD API rather than replacing NDT or the pose initializer.
2. **Unity map into the Autoware map:** align corresponding nearby map geometry. An approximate map-to-map transform is still needed for local registration. A simple starting point is three or more well-spread corresponding static landmarks visible in both maps, such as building corners. Use more landmarks and hold out additional ones for validation. The provided landmark helper uses Open3D to estimate the starting transform; the ROS node refines it with Open3D ICP.

The vehicle pose determines which local map regions to crop; it does **not** determine the relationship between two arbitrary map origins. Autoware localization does not automatically register an unrelated Unity map. If a verified importer/georeferencing transform already exists, use it and check residuals rather than estimating it again. A fixed relationship between unchanged map assets can be saved and reused; this node performs one accepted alignment per process, before driving.

The service estimates both **translation and rotation**. Despite the old name “Rotation Service”, a yaw-only change is generally insufficient. It does not perform global registration with no initial guess, publish a new localization TF authority, move Unity, or initialize the vehicle by matching this extracted CSV.

## What was verified in the uploaded data

| Measurement | Lincoln scan | Unity sample |
|---|---:|---:|
| XYZ rows | 31,110 | 49,950 |
| Finite rows | 31,110 | 49,950 |
| Duplicate XYZ rows | 9 | 0 |
| Within 20 m of each file's own origin | 18,653 | 626 |
| Within 30 m of each file's own origin | 31,110 | 1,220 |
| Median nearest-neighbour distance | 0.061 m | 0.736 m |

Those two file origins are **not** known to be the same physical location. The Unity sample's origin crop is not a verified vehicle neighbourhood. Jake's older PNG labels its reference crop as 1,132 points; the supplied PCD produces 1,220 points in a 30 m sphere about zero. The original plotting script/filter is absent, so the difference is not diagnosed as an error. The nine repeated scan points were left intact.

See `reports/input_audit.json` for bounds, hashes, and exact statistics; `reports/native_frame_overview.png` shows the complete files in separate native frames. `config/input_metadata.json` records unknown information as null rather than assigning guessed values.

## 1. Run the offline inspection now

From the extracted `lincoln_alignment_progress` directory:

```bash
python3 -m venv .venv-offline
source .venv-offline/bin/activate
python -m pip install -r requirements-offline.txt
python tools/inspect_inputs.py
```

This needs no ROS, Unity, or bag. It regenerates the audit JSON and plot. To crop a map after choosing a centre **in that map's native frame**:

```bash
python tools/crop_reference.py \
  --reference current_alignment/pointcloud_uniform_50k.pcd \
  --center X Y Z --radius 30 --output reports/reference_crop.csv
```

Replace X/Y/Z with measured coordinates; no default centre is assumed. This only crops. It does not identify the vehicle or align the data. The bundled reader handles the actual headered XYZ CSV and ASCII PCD. It deliberately rejects binary PCD; use a suitable PCD reader/converter for a later binary map.

## 2. Put the ROS package in a workspace

The archive includes `ros2_ws/src/rotation_integration`, so you can build the included workspace directly. Alternatively, copy only `rotation_integration` into an existing workspace's `src/`. Do not build two copies in the same workspace. This replaces the earlier version of that package.

Use the Python and ROS distribution matching the running Autoware stack. The inspected UB-MR setup uses Humble. The offline environment above is separate; deactivate it before using system ROS Python.

```bash
deactivate
source /opt/ros/humble/setup.bash
cd /path/to/lincoln_alignment_progress
export LINCOLN_PROJECT="$PWD"
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rotation_integration
source install/setup.bash
```

Standard ROS dependencies include sensor_msgs_py, tf2_ros, nav_msgs, std_srvs, NumPy and SciPy. The `initialize_once` executable additionally needs `autoware_adapi_v1_msgs` from your sourced Autoware workspace. The `align_maps` executable needs Open3D, installed into the **same Python interpreter used to build/run the package**; `requirements-registration.txt` pins the tested Open3D version. Do not install it into an unrelated venv and expect a previously built console script to find it.

One option for registration on Humble is a system-Python virtual environment with `--system-site-packages`, created outside `ros2_ws` (to keep colcon from scanning it). Activate it after sourcing ROS/Autoware, install `requirements-registration.txt`, then use that interpreter to run `python -m colcon build --symlink-install --packages-select rotation_integration`. Check `python -c 'import rclpy, open3d, scipy'` in that environment before building. If switching interpreter after an earlier build, clean this package's generated build/install directories or use a fresh workspace.

## 3. Preview Jake's files in ROS now

This publishes the original file coordinates once per second on isolated preview topics, with deliberately unverified frame names:

```bash
ros2 run rotation_integration file_preview --ros-args \
  -p scan_file:="$LINCOLN_PROJECT/current_alignment/lincoln_lidar_10s_30m.csv" \
  -p reference_file:="$LINCOLN_PROJECT/current_alignment/pointcloud_uniform_50k.pcd"
```

Open RViz:

* To inspect the scan, Fixed Frame = `extracted_scan_unverified`, PointCloud2 topic = `/lincoln_file_preview/scan`.
* To inspect the reference, switch Fixed Frame to `unity_pcd_unverified`, topic = `/lincoln_file_preview/reference`.

They cannot both appear aligned in one fixed frame because no verified transform connects them. **Do not add an identity TF to hide this.** Publication timestamps are current preview time, not the missing bag measurement time. Do not remap these preview topics into Autoware's localization inputs. The preview is not a reconstruction of the original sensor stream.

## 4. Initialize Autoware when the real bag/config are available

Use the existing vehicle Autoware launch and correct map, sensor transforms and localization inputs. This bundle cannot provide that launch configuration from the available files. Avoid simultaneously replaying old map-to-base TF/localization outputs and running a new localizer as competing authorities. For recorded data, use a single `/clock` source and set `use_sim_time` consistently. Obtain the exact scan topic, frame, and bag timestamps from the original bag:

```bash
ros2 bag info /path/to/lincoln_bag
ros2 topic list -t
ros2 topic echo --once /actual/lidar/topic --field header
ros2 service type /api/localization/initialize
ros2 interface show autoware_adapi_v1_msgs/srv/InitializeLocalization
```

The client expects the public `autoware_adapi_v1_msgs/srv/InitializeLocalization` interface, with a bounded `pose` array and response `status`. If the installed stack exposes a different interface, use its own adapter/initializer; do not rename an internal NDT service to force it to match.

If the configured GNSS pose has been verified to use the correct map projection/origin and required extrinsics, ask the existing initializer to use it:

```bash
ros2 run rotation_integration initialize_once --use-gnss \
  --ros-args -p use_sim_time:=true
```

Otherwise copy `config/seed_pose.template.json` to a separate seed file, fill its position, quaternion and covariance with a **base_link pose in Autoware map coordinates**, and set `confirmed: true` after checking the frame and units. A LiDAR pose is not interchangeable with a base_link pose. Covariance ordering is x,y,z,roll,pitch,yaw; values are variances in m²/rad². The template deliberately contains nulls and refuses to run as-is.

```bash
ros2 run rotation_integration initialize_once \
  --seed "$LINCOLN_PROJECT/config/your_seed.json" \
  --ros-args -p use_sim_time:=true
```

This sends one request, reports success/error and exits. On timeout the server outcome is unknown; inspect Autoware's state before retrying. A successful API response is not independent proof of correct localization. Check initialization state, diagnostics, kinematic state and map overlap. Confirm the vehicle is stationary before proceeding.

## 5. Validate the localized LiDAR scan

Start the snapshot node with the actual localized sensor stream/frame:

```bash
ros2 launch rotation_integration snapshot.launch.py \
  cloud_topic:=/actual/lidar/topic expected_scan_frame:=actual_scan_frame \
  map_frame:=map use_sim_time:=true
```

Start RViz/recorders before capturing because snapshot outputs are volatile. RViz Fixed Frame = `map`, cloud topic `/localization_snapshot/map_cloud`. Compare against the real Autoware map. Call:

```bash
ros2 service call /localization_snapshot/capture std_srvs/srv/Trigger '{}'
```

It resolves `map <- actual_scan_frame` at the measurement time and publishes XYZ plus `/localization_snapshot/sensor_pose`. It refuses zero/future/stale stamps, wrong frames, and missing exact-time TF. It does not fall back to latest TF, deskew scans, or determine stationarity. The default 20 m crop is relative to the message frame origin. The optional existing UB-MR bridge's `/ub_mr/localization` output should not be substituted for map TF without checking its origin adjustments.

## 6. One-time map-to-map refinement (after missing inputs arrive)

Inputs expected by `align_maps`:

| Input | Default | Contract |
|---|---|---|
| Autoware reference map | `/map/pointcloud_map` | PointCloud2 in configured Autoware frame; static map, not raw LiDAR |
| Unity reference map | `/unity/reference_cloud_ros` | PointCloud2 in a confirmed right-handed ROS-basis Unity-map frame, in metres |
| Vehicle localization | `/localization/kinematic_state` | Odometry, parent Autoware map, child configured body_frame |
| Approximate map transform | JSON config | `initial_autoware_from_unity_ros`, 4x4 rigid, no scale/reflection |

The map topic defaults must be checked in your launch. Dynamic Autoware map loaders may expose only loading services or local partitions; in that case expose the corresponding local reference cloud to this node. It does not implement Autoware's differential-map service protocol. The node caches entire incoming messages and crops on the service call; publish nearby map subsets if memory is limited. It does not reject map timestamps based on age because reference maps are static; replace/restart when map assets change.

**Axes matter:** the repository's ROS-to-Unity position mapping is `(x,y,z) -> (-y,z,x)`. Actual Unity world XYZ can be converted back to a right-handed ROS basis with `(ux,uy,uz) -> (uz,-ux,uy)`. This conversion is NOT automatically appropriate for a raw PCD; the file may already be ROS-style. Confirm its original axes and the Unity import scale/transform first. Do not apply the conversion twice. Both registration clouds must have compatible units/handedness before ICP.

For an approximate starting transform from landmarks, fill `config/landmarks.template.json` with three or more pairs in matching order. Both sets must already be in the respective confirmed right-handed map bases. Run with Open3D installed:

```bash
python tools/seed_from_landmarks.py \
  --landmarks config/your_landmarks.json \
  --output config/map_alignment.json
```

Inspect the residuals, frame labels and map orientation. Add held-out landmarks to check correctness; a fit to the same three points is not independent validation. Then set `confirmed: true` in the output config. Alternatively fill the 4x4 matrix from a verified importer/georeferencing relationship. The tool does not find corresponding landmarks automatically.

Start the node in a sourced workspace that can import Open3D:

```bash
ros2 run rotation_integration align_maps --ros-args \
  -p config_file:="$LINCOLN_PROJECT/config/map_alignment.json" \
  -p use_sim_time:=true
```

By default it requests transient-local reliable QoS for the Autoware map and volatile best-effort for Unity's stream. For a volatile Autoware publisher, add `-p autoware_transient_local:=false` and republish the map after subscription. For a latched Unity publisher, add `-p unity_transient_local:=true`. Start the appropriate publishers and inspect QoS with `ros2 topic info -v`.

After localization is valid and the vehicle is stationary:

```bash
ros2 service call /map_alignment/align_once std_srvs/srv/Trigger '{}'
```

It checks frame names and current reported linear/angular speeds, crops around the localized vehicle in Autoware and the inverse-seeded corresponding location in Unity, then uses Open3D multi-scale point-to-point ICP. It estimates all six rigid degrees of freedom, rejects insufficient/near-planar local geometry and filters by fitness, RMSE and change from the seed. Thresholds are starting values, not site-validated guarantees. Speed checks are instantaneous; there is no dwell-time or localization-quality monitor. Ensure the vehicle remains stopped and check Autoware diagnostics externally.

Outputs:

* `/map_alignment/candidate_transform` — TransformStamped with parent Autoware map and child Unity ROS-basis map, mapping **Unity points into Autoware**, `p_A = T_A_U p_U`. This is a candidate on its own topic, **not a TF broadcast**.
* `/map_alignment/aligned_unity_cloud` — local Unity cloud transformed into Autoware map coordinates for RViz comparison.
* `/map_alignment/report` — JSON numerical metrics and the 4x4 transform. Successful outputs are transient-local, so configure later subscribers accordingly.

Once accepted, the node refuses another alignment until restarted; it does not update the map every scan. A rejected attempt can be retried after correcting the inputs. Inspect independent buildings/corners and ground height; ICP can produce a wrong local minimum with seemingly good fitness, especially with repeated/ground-dominated features. The source-direction fitness is not a global alignment certificate.

Unity application remains pending the actual project and parent/import conventions. If source U is *current Unity world coordinates converted to ROS basis*, and the desired target display is simply Autoware coordinates converted with matrix C, the Unity world correction would be `C T_A_U C^-1`. A separate target origin/anchor adds another transform. Do not apply this formula to a PCD-local frame without including its importer transform. Apply the eventual correction to the common environment/cloud parent, not the vehicle.

## Existing bridge finding

Inspected `ub-cavas/mr_pkg` at commit `f5ba57c3a08519dd2fb82757ad29f8fc4e37c446`: `autoware_localization.py` consumes kinematic_state, subtracts MGRS-style origin offsets, sets z=0 and labels the result `odom` on `/ub_mr/localization`. That implementation is not a verified full 6-DOF map registration and cannot by itself resolve the relationship between these uploaded PCD coordinates. No existing bridge code has been changed. The local UB-DigitalTwin checkout inspected is `c1acf308d5715f266a0f9328df86c8a801cb2f76`; this is a source observation, not a claim about all future revisions.

## What to get from Jake / Unity next

1. The actual Autoware map (or a dense local crop), map projector info, map/version provenance and how it relates to Unity's PCD.
2. Original scan topic, exact header frame and timestamp, and the extraction/cropping script; “10 seconds” alone is not a ROS stamp.
3. URDF/TF sensor calibration through `base_link`, `sensor_kit_base_link`, and `velodyne_top`, plus the full bag inputs needed by localization.
4. Autoware version/launch, initialization service and GNSS/map-origin configuration, and a valid approximate vehicle pose if GNSS is unsuitable.
5. Unity cloud importer axes, scale, original transform and environment parent. Also identify the same stable features in both map frames for an initial map-to-map estimate.

Suggested split: Jake verifies initialization/map provenance; Devan runs the ROS input/TF checks and service integration; together validate the map transform and Unity parent application. The uploaded sketch is a design, not numerical calibration data.

## Validation and references

Run `PYTHONPATH=ros2_ws/src/rotation_integration python -m unittest discover -s ros2_ws/src/rotation_integration/test -v`. Tests include the uploaded files, frame/seed rejection, transforms and Open3D synthetic map registration (Open3D tests skip if it is absent). See `reports/TEST_REPORT.md` for the executed checks and untested boundaries.

Primary documentation used:

* https://docs.autoware.org/pr-874/design/autoware-architecture-v1/interfaces/components/localization/
* https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html

Use your installed Autoware interface definitions as authoritative for its version.
