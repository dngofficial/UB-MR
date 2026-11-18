# Validation report — 2026-11-14

## Executed

- Parsed the actual uploaded headered CSV (31,110 rows) and ASCII PCD (49,950 rows), checked finite values, bounds, duplicates and spherical-radius counts.
- Regenerated `input_audit.json` and `native_frame_overview.png`; inspected the plot. It is a native-frame overview, not registration evidence.
- Ran 13 unit tests, all passed, including the real input files, centred cropping, malformed PCD rejection, unconfirmed/wrong-frame seed rejection, invalid covariance/quaternion rejection, and rigid transform math.
- Tested actual Open3D 0.19.0 registration on synthetic maps with a known rotation/translation, deliberately perturbed starting rotation and translation, and a large 500,000 / 4,200,000 coordinate origin. The recovered matrix met the test tolerance (0.015 per element; mean transformed-point error below 0.02 m).
- Verified rejection of insufficient local overlap, nearly planar geometry, reflected coordinate transforms, and excessive refinement.
- Independently checked the landmark helper's known rigid transform recovery and collinear-landmark rejection.
- Compiled all Python sources for syntax validation.
- Preserved Jake's original data/image bytes and original README as `JAKE_NOTES.md`.

`test_output.txt` contains the test run. Open3D emitted a missing-CUDA warning; these tests used CPU registration, and no GPU was required. Runtime here: Python 3.12 with Open3D 0.19.0. This is not the ROS Humble Python 3.10 environment.

## Not executed / not available

- No installed ROS 2 runtime: no colcon/ament build, DDS/QoS, PointCloud2 publication, TF, service discovery, or live callback test was performed.
- No running Autoware stack or original Lincoln bag: initialization API and localization quality/TF tests were not performed.
- No Autoware map in the upload: real map-to-map registration was not attempted.
- No Unity project/import configuration: no reference-cloud publisher or Unity parent-transform application was tested or included.
- Stationarity and original sensor identity/timestamps cannot be independently verified from a single XYZ CSV.

The tests support the offline math/data handling and library integration only. They do not establish a valid physical pose or map alignment. The generated templates intentionally remain unconfirmed and contain missing values; numerical screening in the map service is not an independent correctness guarantee.
