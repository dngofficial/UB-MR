# Local source discovery — 2026-11-17

**Follow-up correction (2026-11-18):** the mounting values below stop short of the final scan frame. Native xacro expansion also gives `velodyne_top_base_link → velodyne_top = (0,0,0.06611)` in the installed VLS-128 description. The bag's diagnostics identify VLP-32-C, so that terminal joint is not confirmed for the recording. See `JAKE_FOLLOWUP.md` and `calibration_comparison.json`; no recording extrinsic is selected in current metadata.

The working Unity project is `/home/devanng/Downloads/UB-MR-2/UB-MR-2`. `/home/devanng/UB-DigitalTwin` was inspected as a reference. Its embedded UB-MR project was not substituted for this project or modified.

## Files now found

| Input | Location | Finding |
|---|---|---|
| Oakley recording | `/home/devanng/Downloads/oakley_test_bag-selected.zip` | 15.11 GB SQLite bag plus metadata; 271.9235 s, 856,159 messages, recorded ROS schema Humble |
| Extracted recording | `Integrations/LincolnAlignment/.local-data/oakley_test_bag` | Extracted here with ZIP CRC checks; `.local-data/` is ignored by Git; roughly 15.1 GB on disk |
| Original investigation ZIP | `/home/devanng/Downloads/lincoln_alignment_progress.zip` | Its CSV and PCD match the already-integrated data exactly; it is the original investigation, not an implementation update |
| Dense Local map | `/home/devanng/host_data/maps/ub_autonomous_proving_grounds/pointcloud_map.pcd` | 12,425,186 points; binary XYZ/intensity/normals; adjacent projector file literally says `projector_type: local` |
| Physical-map candidate | `/home/devanng/host_data/ub_hdmap/pointcloud_map.pcd` | 8,026,472 ASCII XYZ points; projector `MGRS`, grid `17TPH`, vertical datum `WGS84` |
| Sensor calibration | Installed image `ubcavas/autoware-lincoln:20260303.0` | Sensor-kit/vehicle descriptions and Lincoln launch files extracted for inspection from a temporary stopped container, then the temporary container was removed |
| Existing Unity localization bridge | `~/UB-DigitalTwin/UB-MR/submodules/mr_pkg/mr_pkg/autoware_localization.py` | Applies modulo-100km UTM origin subtraction, forces Z=0, labels output `odom`; its assumptions must match the chosen map |

The home Digital Twin checkout documents a versioned v1.1.0 PCD under `Autoware/host_data/maps/.../v1.1.0`, but that file is absent locally. Its tracked remapping report describes a 21,349,776-point simulated map; that is a different artifact from the 12.4M Local map above. Do not treat a README's installation path as evidence that the map is installed.

Full path/header/hash inventory: `local_source_inventory.json`. Bag observations: `oakley_bag_audit.json`. The copied calibration YAMLs in `../config/reference/` are review references, not confirmation of the September recording's calibration.

Reference checkout revisions inspected:

- UB-DigitalTwin: `23e232ba8fb2c52e07f2e90f91819443e5238e7d`
- Its ub-lincoln-docker: `7ad1c192d294bad5fb95abf563262c99abfce2fd`
- Its mr_pkg: `f5ba57c3a08519dd2fb82757ad29f8fc4e37c446`

## Exact scan recovered

The first LiDAR scan recorded after a 10-second bag offset has:

- Header frame: `velodyne_top`
- Header stamp: seconds `1757032460`, nanoseconds `631107072`
- Full finite points: **45,273**
- 20 m spherical crop: **18,653**
- 30 m spherical crop: **31,110**, exactly equal to every uploaded CSV XYZ value and row order

That same XYZ appears on `/sensing/lidar/top/aw_points`, `/sensing/lidar/top/aw_points_ex` and `/sensing/lidar/top/pointcloud_raw_ex`. Additional field layouts differ, so the single XYZ CSV cannot identify which topic Jake exported. Use `/sensing/lidar/top/aw_points` as a recorded-message candidate for checks and retain the original stamp/frame.

The bag contains no `/tf`, `/tf_static` or `/localization/*` topics. `/sensing/gnss/novatel/oem7/odom` is GNSS odometry, not Autoware `kinematic_state`. Reconstruct the recording's sensor TF from its verified robot description and run the existing localization pipeline. Changing a frame label or adding an identity transform would not establish localization.

GNSS-reported speed is very low near the extraction offset (maximum about 0.00464 m/s in the 8–12 s window); its first reported norm above 0.1 m/s occurs at bag offset **199.225 s**. However, this same window contains GNSS position jumps spanning about 0.50 m in X/Y and 0.82 m in Z. These are observations of that topic, not proof of physical stationarity or localized-pose quality. Cross-check vehicle velocity/wheel data and consecutive LiDAR scans before choosing a runtime alignment window. Do not use this raw odometry as the alignment service's localization input.

## Coordinate-frame finding

At roughly 10 seconds, GNSS NavSatFix is:

- latitude `42.993243137132964`, longitude `-78.79302490093541`, altitude `143.4469912117347`
- header frame `gnss_link`

Projecting that WGS84 fix with installed PROJ `cs2cs` to UTM zone 17N reproduces the recorded odometry position: approximately `(679909.802034, 4762427.873072, 143.446991)`. Although the odometry labels its child `base_link`, its position equals the GNSS fix; do not assume a calibrated body pose solely from the label.

The separate MGRS map has bounds approximately:

- X: `79478.477` to `80008.820`
- Y: `62269.801` to `62575.379`
- Z: `112.05682` to `163.39424`

Under the UTM-to-MGRS-grid hypothesis, the GNSS location corresponds to approximately `(79909.802, 62427.873, 143.447)` within that map. There are 443,845 map points within 35 m of that candidate location. This supports investigating this physical-map candidate with Autoware's existing GNSS poser/projector and sensor extrinsics. It does **not** confirm the exact recording map, vertical convention, base_link lever arm or localized orientation; no confirmed seed/TF was generated. `physical_map_candidate.json` records the hypothesis explicitly as unconfirmed.

The 12.4M Local map instead has bounds near `[-506,-83,-2]` to `[93,249,19]`. The sampled Unity PCD is not an exact subsample of that map in its current raw file coordinates: none of its 49,950 points matched a map point within 0.1 mm; nearest-neighbor median was 0.121 m, 95th percentile 1.466 m, maximum 21.462 m. This does not rule out a geometric relationship, but the filename and similar site coverage do not establish one. Establish its relationship to the physical map and the selected Unity environment before registration.

## Mounting calibration found

The installed image's YAML/xacro chain gives:

- `base_link → sensor_kit_base_link`: translation `(1.24, 0, 1.93)` m, zero roll/pitch/yaw
- `sensor_kit_base_link → velodyne_top_base_link`: identity, followed by the installed description's additional scan joint
- `sensor_kit_base_link → gnss_link` and `imu_link`: translation `(-1.07, 0, -1.33)` m, zero roll/pitch/yaw

These mounting YAML values give base-to-GNSS `(0.17,0,0.60)`. The complete installed-description base-to-`velodyne_top` transform is `(1.24,0,1.99611)` after its additional 0.06611 m scan joint, as established in the 2026-11-18 follow-up. This is a VLS-128 description, whereas the bag diagnostics report VLP-32-C. Verify the recording's complete calibration rather than adopting either mount-only or installed-description values. Let the normal URDF/TF and GNSS poser apply the appropriate offsets once. The extracted AD API service definition also matches the initializer client's bounded pose-array request/status-response interface.

The reference Dockerfile selects Autoware `0.45.1`; the installed image has tag `20260303.0`, whereas the reference compose file requests `20251015.0`. These observations do not establish the recording's exact software revision or prove that either remote image tag is unavailable.

## Reproduced installer failure

Read-only check executed:

```bash
python3 /home/devanng/UB-DigitalTwin/scripts/install_ub_carla.py --version v1.1.0 --check
```

It found `UB-CARLA-v1.1.0.zip` but failed with:

```text
Autoware maps v1.1.0 not found in public Drive folder. Expected v1.1.0.
```

The installer requires a `v1.1.0` folder containing `lanelet2_map.osm`, `pointcloud_map.pcd` and `map_projector_info.yaml` in its configured public maps folder. It resolves both CARLA and map releases before downloading either. `Autoware/setup_autoware.sh` invokes the same map installer first; this failure happens before its Docker pull. For this reproduced path, the missing public map release/sharing/layout needs correction; changing the Docker image tag would not fix the earlier map check. No builds were downloaded, existing image tags changed, or reference-checkout files edited.

There is also an empty legacy `CARLA/setup_carla.sh` in that checkout. The documented installer is `scripts/install_ub_carla.sh`. The exact user's failing command/output is still useful to distinguish other failures such as a separate Docker pull error.

The existing image/calibration and local bag remain usable for read-only analysis, so the alignment investigation can continue without a fresh simulator install.

## Run the real-data audit again

From the working UB-MR project root:

```bash
source /opt/ros/humble/setup.bash
python3 Integrations/LincolnAlignment/tools/inspect_local_bag.py
```

The tool opens the extracted SQLite bag read-only, reads CDR using installed standard ROS message types, compares the scan crop with the original CSV and writes `reports/oakley_bag_audit.json`. It does not launch/replay ROS nodes, alter the source ZIP/database, or publish TF.

For a future initialized Autoware replay, the snapshot arguments are now concrete:

```bash
ros2 launch rotation_integration snapshot.launch.py \
  cloud_topic:=/sensing/lidar/top/aw_points \
  expected_scan_frame:=velodyne_top map_frame:=map use_sim_time:=true
```

That command still requires verified TF and Autoware localization; bag playback alone cannot provide them. Follow the main guide's initialization and overlay checks. Next confirm the physical map/version and recording calibration, run localization on the stationary interval, then establish the physical-map-to-Unity seed and validate held-out features before applying the map-parent correction.
