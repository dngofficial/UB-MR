# Implementation checks — 2026-11-16

The original `TEST_REPORT.md` and `test_output.txt` describe the uploaded bundle's earlier tests. This report covers the implementation in this UB-MR checkout.

| Check | Result |
|---|---|
| Uploaded data preservation | Original CSV, PCD, comparison PNG and JAKE_NOTES.md verified byte-for-byte against ZIP |
| Unity scene/mesh audit | Selected `pointcloud_uniform_50k.asset`: 49,950 points, CurrentXZY, maximum coordinate difference from ZIP PCD 0.000005 m |
| Offline Python suite | 21 passed; 2 opt-in ROS tests skipped in this invocation |
| ROS2 integration suite | 2 passed, actual DDS/service/TF communication, synthetic fixtures, isolated domain 83 |
| ROS package | `colcon build --symlink-install --packages-select rotation_integration` passed |
| Installed CLI/launch | Initialization client `--help` and alignment launch `--show-args` passed |
| Unity compilation | Full runtime assembly compiled; headless Editor also compiled and executed the checks |
| Unity checks | Passed basis conversion, correction/parent composition, large-origin subtraction, spherical selection, world-frame invariance and XYZ byte encoding |

Logs: `implementation_offline_checks.txt`, `implementation_ros_checks.txt`, `implementation_unity_checks.txt`. The offline and ROS suites together exercise 23 distinct Python tests.

The offline tests include real supplied-data parsing/radius counts, map registration with rotation/translation at a large coordinate origin, rejection of planar/insufficient geometry and excessive refinement, invalid rigid transforms/seeds/covariances, continuous stationary dwell, motion/drift/stale/gapped/reset localization, and padded/reordered/big-endian PointCloud2 layouts without modifying the incoming buffer.

The ROS alignment fixture publishes a transient-local reference map before subscription and a repeated volatile Unity reference. It verifies rejection while moving and for wrong cloud frames, recovery of a known nonidentity map transform, numerical report persistence, candidate/frame/snapshot identity, aligned output coordinates, late-subscriber delivery and refusal of a second accepted alignment. The fixture explicitly disables the Autoware initialization-state requirement because no Autoware stack is running; it does not fabricate a real localization state.

The snapshot fixture uses actual ROS TF transport for a two-link chain `map → body → LiDAR`, with a rotated body and nonzero mounting translation. It checks the expected output points and original scan timestamp, rejects a zero scan stamp, and rejects an older scan when only a newer dynamic transform is available. No latest-TF fallback or second mounting-offset application is used.

Environment: Ubuntu 22.04, ROS Humble/Python 3.10, Open3D 0.19.0, NumPy 1.24.4, installed SciPy 1.8.0 and Unity 6000.0.36f1. The test interpreter is `/tmp/ubmr-lincoln-venv/bin/python`, created with system site packages; only NumPy was overridden there. The host's default NumPy 2.2.6 is incompatible with its compiled SciPy stack. `requirements-ros.txt` supplies a reproducible project environment, including SciPy 1.10.1; the full pinned environment was not separately installed during these checks. Open3D registration ran on CPU.

Reproduce from the repository root after preparing the environment in `../README.md`:

```bash
bash Integrations/LincolnAlignment/tools/check.sh offline
bash Integrations/LincolnAlignment/tools/check.sh build
bash Integrations/LincolnAlignment/tools/check.sh ros
```

In Unity: **UB-MR → Lincoln Alignment → Run Offline Unity Checks**. Headless execution of `CAVAS.UB_MR.PointCloud.Editor.LincolnAlignmentChecks.Run` passed with exit code zero. These Unity checks deliberately do not initialize ROS or exercise native message publication; ROS communication was tested independently in the Python fixtures.

## Remaining physical/integration validation

- No original Lincoln bag, actual Autoware point cloud/projector configuration, deployed sensor URDF/calibration or active Autoware stack was available.
- The real initialization service and initialization-state subscription could not be exercised; `autoware_adapi_v1_msgs` is absent from this local ROS installation. The documented public AD API was checked against official Autoware interface documentation; the installed vehicle version still needs confirmation.
- No real map transform, automatic initial vehicle pose, physical registration accuracy or stationary interval was established from the uploaded XYZ alone.
- Unity's live reference publisher/candidate receiver and the full scene with vehicle playback were compiled but not exercised together with an Autoware process. The Unity checks validate coordinate math, selection and byte encoding independently.
- The mr_pkg submodule is empty here, so its deployed origin subtraction, vehicle Z handling and TF ownership require inspection before enabling Unity's display-origin confirmation.
- Numerical ICP/covariance thresholds are defaults, not calibrated Lincoln acceptance limits. Inspect held-out physical features, localization diagnostics and scan/map overlap before applying a candidate.

The saved scene keeps `displayOriginConfirmed=false` and `applyAutomatically=false`. Applying requires the target display frame to be configured. The unrelated legacy external-file loader and duplicate cloud are disabled, while the selected reference stays active. Applying updates the shared map parent and map UI only for the current Play session; a saved calibration/preset workflow is not included.
