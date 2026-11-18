# rotation_integration

ROS Humble package for one-shot Lincoln/Unity map alignment. Autoware supplies localization and the reference map; this package does not replace the vehicle localizer.

- `initialize_once`: request initialization through Autoware's public AD API, using its configured GNSS path or a verified map-frame body pose.
- `snapshot`: resolve the complete map-to-scan TF chain at the measurement timestamp and publish a nearby localized scan for inspection.
- `align_maps`: require initialized, fresh, continuously stationary localization; refine a seeded nearby Unity-map → Autoware-map transform; publish a candidate, overlay and JSON report once.
- `file_preview`: publish the uploaded XYZ files in separate, explicitly unverified frames.

The included `alignment.launch.py` requires a confirmed map-seed JSON. Autoware and Unity run separately. `autoware_adapi_v1_msgs` must be available from the sourced Autoware workspace for real initialization/state checks; Open3D is required for registration. Standard ROS dependencies are declared in package.xml. The standalone synthetic fixtures can run without the Autoware messages.

`recorded_sensor_frames.launch.py` reuses `robot_state_publisher` for a complete sensor-only fixed-joint chain. It requires an explicit profile with `confirmed_for_bag=true`, refuses incomplete chains, and supplies no map-to-body localization. The shipped profiles remain unconfirmed. See [Jake's calibration follow-up](../../../reports/JAKE_FOLLOWUP.md) for the recorded VLP-32-C versus installed VLS-128 description mismatch.

See [the integration guide](../../../README.md) for environment setup, Unity scene wiring, frame/seed requirements, bag replay, service calls and checks. From the UB-MR repository root:

```bash
bash Integrations/LincolnAlignment/tools/check.sh offline
bash Integrations/LincolnAlignment/tools/check.sh build
bash Integrations/LincolnAlignment/tools/check.sh ros
```

Set `LINCOLN_PYTHON` to the prepared Python environment. The ROS checks use synthetic data in isolated domain 83. The real bag/map/calibration and deployed display origin still need validation; no physical transform is supplied as a default.
