# Follow-up using Jake's calibration — 2026-11-18

Jake's reported mount and the Local projector are now represented explicitly in the integration. This update adds reproducible calibration/geographic audits and a guarded sensor-frame replay launch. No recording calibration, map-frame vehicle pose or physical map alignment is automatically accepted.

## The complete frame chain matters

Jake supplied:

```text
base_link -> sensor_kit_base_link
  xyz = (0.9, 0.0, 2.0) m
  rpy = (-0.001, 0.015, -0.0364) rad
sensor_kit_base_link -> velodyne_top_base_link
  identity
velodyne_top_base_link -> velodyne_top
  not yet verified
```

`config/calibration_candidates.json` preserves this as `jake_reported`, with `confirmed_for_bag=false` and an incomplete scan-frame chain. Both installed images (`latest` and `20260303.0`) were inspected using temporary stopped containers. Their native xacro was expanded with the real ROS xacro processor. Both installed descriptions use:

```text
base_link -> sensor_kit_base_link: (1.24, 0, 1.93), zero RPY
sensor_kit_base_link -> velodyne_top_base_link: identity
velodyne_top_base_link -> velodyne_top: (0, 0, 0.06611), zero RPY
```

The final 6.611 cm joint comes from `vls_description/urdf/VLS-128.urdf.xacro`. The previously reported `(1.24,0,1.93)` stopped at the mounting frame. The full installed-description `base_link <- velodyne_top` translation is **(1.24, 0, 1.99611)**. The native fixed-chain reference is saved in `config/reference/installed_lidar_chain.urdf`, and image IDs/source hashes are recorded in the candidate config.

Applying the installed terminal joint to Jake's rotated mount gives the explicitly mixed-source profile `jake_with_installed_scan_joint_hypothesis`. Its LiDAR origin in base_link is approximately:

```text
(0.9009933613, 0.0000299795, 2.0661025297) m
```

The small X/Y changes arise because the final sensor offset is composed through Jake's rotation, rather than added directly in vehicle axes. This is a computation for comparison, not a confirmed recording transform.

On the actual 31,110-point CSV, the two complete candidates differ by:

| Measurement | Result |
|---|---:|
| Sensor-origin displacement | 0.3462 m |
| Relative rotation | 2.2560 degrees |
| Median transformed-point displacement | 0.8494 m |
| 95th percentile displacement | 1.3193 m |
| Maximum displacement | 1.4635 m |

These are calibration sensitivity measurements, not registration errors or proof that either candidate is correct. See `calibration_comparison.json`.

## A sensor-model conflict was recovered from the bag

The recorded `/diagnostics` hardware model is **VLP-32-C**. The point-cloud channel/ring values cover 0 through 31. The installed driver configuration also selects VLP32, but its sensor URDF references a **VLS-128** geometry macro. The installed Velodyne description package contains HDL-32E and VLP-16 URDFs; no VLP-32-C description was found there.

Consequently, the VLS-128 final joint must not be silently adopted as the recorded VLP-32-C joint. Jake's actual description/frame convention and the recording configuration still determine the correct terminal transform. All shipped candidates stay unconfirmed; no runtime profile is selected.

The bag has zero `/parameter_events` messages, so its recording-time robot description cannot be recovered there. Its `/rosout` does record repeated NovAtel `GPSFix position source= INSPVA` switches. Those logs explain why the raw GNSS input deserves scrutiny when low speed coexists with pose jumps; they do not independently certify a stable localized body pose. The extended `inspect_local_bag.py` records the model, channel IDs and source-switch logs in `oakley_bag_audit.json`.

## What the Local projector does and does not establish

The actual proving-ground projector file says `projector_type: local`. Its Lanelet map contains 1,583 local-coordinate nodes and **zero populated geographic anchors**: latitude/longitude attributes are empty. The projector file supplies no geographic origin, heading or vertical relationship. Those files alone cannot establish `map <- UTM`.

The current Unity RoadRunner OpenDRIVE file supplies a geographic hypothesis:

```text
+proj=tmerc +lat_0=42.9926180599693 +lon_0=-78.7924945927909
+k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs
```

Using the installed PROJ command-line tools, the bag's GNSS fix projects into this OpenDRIVE XY system at approximately **(-43.2466, 69.4416) m**. Projecting the corresponding UTM odometry to the same system agrees within about 1.4 nanometres, confirming consistency of those two coordinate representations in this numerical check. It does not prove that this OpenDRIVE XY system is the actual PCD map frame.

The Transverse Mercator origin parameters define the geographic projection, as described by [PROJ's documentation](https://proj.org/en/stable/operations/projections/tmerc.html). Their presence in a separate OpenDRIVE file does not establish the PCD's origin/axes or a local vertical datum. The audit deliberately leaves `map_z`, `body_pose` and the confirmed UTM-to-map relationship unset. The marker is a **GNSS sensor position hypothesis**, not a localized base_link pose or initializer seed.

![Geographic diagnostic only](local_georeference_audit.png)

`local_georeference_audit.json` and the PNG make this hypothesis reviewable against the Local Lanelet geometry. The marker's placement does not establish registration or a driveable initial position. Verify the PCD/OpenDRIVE provenance and held-out static/geographic features before adopting the projection.

## Implementation added

- `frame_chain.py`: composes fixed-joint URDF transforms with the correct RPY order, supports reverse lookup, and rejects missing links, moving-joint gaps, ambiguous parents, cycles and unexpanded xacro.
- `compare_calibrations.py`: reports each candidate's complete/incomplete status and numerical sensitivity on the real extracted scan; publishes no ROS data.
- `georeference.py` / `audit_local_georeference.py`: audit geographic anchors and use local PROJ for an explicitly unconfirmed XY projection. A nonzero OpenDRIVE offset is rejected rather than ignored.
- `recorded_sensor_frames.launch.py`: reuses standard ROS `robot_state_publisher` for a verified sensor-only fixed chain. It requires an explicit profile confirmed for the recording, rejects an incomplete chain even if its flag is changed, and refuses a localization transform above the body frame. It never provides map-to-body localization.
- Current input metadata now references both calibration candidates, the recorded sensor model and the unresolved geographic relationship. The earlier incomplete mount-only extrinsic is no longer presented as a selected full LiDAR extrinsic.

## Run the new checks

The compatible environment is now in the project at `.venv-ros`, rather than the earlier temporary `/tmp` environment.

From the working repository root:

```bash
source /opt/ros/humble/setup.bash
export LINCOLN_ROOT="$PWD/Integrations/LincolnAlignment"
export LINCOLN_PYTHON="$LINCOLN_ROOT/.venv-ros/bin/python"

"$LINCOLN_PYTHON" "$LINCOLN_ROOT/tools/compare_calibrations.py"
"$LINCOLN_PYTHON" "$LINCOLN_ROOT/tools/audit_local_georeference.py" --plot
"$LINCOLN_PYTHON" "$LINCOLN_ROOT/tools/inspect_local_bag.py"
bash "$LINCOLN_ROOT/tools/check.sh" offline
bash "$LINCOLN_ROOT/tools/check.sh" build
source "$LINCOLN_ROOT/ros2_ws/install/setup.bash"
bash "$LINCOLN_ROOT/tools/check.sh" ros
```

These audits require the local files described in `LOCAL_DISCOVERY.md`. Geographic projection additionally needs PROJ `cs2cs` (normally the `proj-bin` package). The plot is a standalone static artifact. The ROS checks use synthetic fixtures in isolated domain 83.

After verifying the recording's full VLP-32-C chain, put that verified profile in a separate local candidate file, preserve its provenance and set `confirmed_for_bag=true`. Only then use:

```bash
ros2 launch rotation_integration recorded_sensor_frames.launch.py \
  calibration_file:=/absolute/path/to/verified_calibration.local.json \
  profile:=your_verified_profile use_sim_time:=true
```

This launch supplies only the fixed LiDAR sensor chain represented by that profile. It does not provide GNSS/IMU extrinsics unless those are also included and verified. When a full Autoware robot description already publishes these joints, use that existing authority instead of starting another publisher for the same frames.

Then replay the stopped interval and run the existing snapshot with `cloud_topic:=/sensing/lidar/top/aw_points`, `expected_scan_frame:=velodyne_top`, and `map_frame:=map`. Autoware must first provide valid map-to-body localization. The reported geographic XY is insufficient to initialize a full map-frame body pose; the Local map's origin/axes, vertical relationship and GNSS lever arm/orientation still need resolution.

## Checks executed

- 33 offline tests passed, including twelve new URDF-chain/geographic checks, the real extracted data and synthetic ICP recovery.
- Three ROS integration tests passed. The new test starts native `robot_state_publisher` with an isolated synthetic fixture using Jake's RPY and the nominal terminal joint, checks the resulting TF, and captures a point through the existing snapshot node. This validates software composition, not the recording's physical extrinsic.
- ROS package build passed. Installed launch arguments were checked; launching a shipped unconfirmed profile was verified to fail before starting a publisher.
- Native xacro expansion succeeded for both installed images; the two frame chains agree.
- Calibration and geographic audits ran on the real local inputs. The diagnostic plot was inspected.

See `jake_followup_offline_checks.txt` and `jake_followup_ros_checks.txt`. The test environment uses Python 3.10, NumPy 1.24.4 and the installed ROS Humble/Open3D/SciPy packages; no global Python packages were changed. Full Lincoln Autoware localization, physical map registration and Unity application remain unvalidated.
