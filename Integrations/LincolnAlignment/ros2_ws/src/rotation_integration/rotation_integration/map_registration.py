"""Local Unity-map -> Autoware-map refinement using Open3D's ICP."""
import numpy as np
from .data_io import crop_xyz


def rigid(matrix):
    a = np.asarray(matrix, dtype=float)
    if a.shape != (4, 4) or not np.isfinite(a).all():
        raise ValueError('Transform must be a finite 4x4 matrix')
    r = a[:3, :3]
    if not (np.allclose(a[3], [0, 0, 0, 1]) and
            np.allclose(r.T @ r, np.eye(3), atol=1e-5) and
            np.isclose(np.linalg.det(r), 1, atol=1e-5)):
        raise ValueError('Transform must be rigid; convert axes/handedness before registration')
    return a


def transform(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def align_maps(unity_ros_xyz, autoware_xyz, initial, center_autoware,
               radius=30., min_fitness=.5, max_rmse=.5,
               max_refinement_translation=5., max_refinement_angle_deg=20.):
    import open3d as o3d
    initial = rigid(initial)
    settings = [radius, min_fitness, max_rmse, max_refinement_translation, max_refinement_angle_deg]
    if not np.isfinite(settings).all() or min(settings) <= 0 or min_fitness > 1:
        raise ValueError('Registration thresholds must be finite and positive; fitness <= 1')
    center = np.asarray(center_autoware, dtype=float)
    center_source = transform(center.reshape(1, 3), np.linalg.inv(initial))[0]
    source = crop_xyz(np.asarray(unity_ros_xyz), center_source, radius)
    target = crop_xyz(np.asarray(autoware_xyz), center, radius + max_refinement_translation)
    if min(len(source), len(target)) < 100:
        raise ValueError('Need at least 100 local points in each map; check seed and map density')
    # Work near zero to avoid numerical conditioning problems with large map origins.
    source_local = transform(source, initial) - center
    target_local = target - center
    for cloud in (source_local, target_local):
        spread = np.linalg.svd(cloud - cloud.mean(axis=0), compute_uv=False)
        if spread[0] <= 0 or spread[-1] / spread[0] < 1e-3:
            raise ValueError('Local cloud is nearly planar/linear; include stable 3-D features')
    def pc(points):
        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(points)
        return out
    src, dst = pc(source_local), pc(target_local)
    delta = np.eye(4)
    registration = o3d.pipelines.registration
    for voxel, max_distance in ((1., 3.), (.5, 1.5), (.25, .75)):
        s, t = src.voxel_down_sample(voxel), dst.voxel_down_sample(voxel)
        if min(len(s.points), len(t.points)) < 30:
            raise ValueError('Too few distinct local points after voxel downsampling')
        result = registration.registration_icp(
            s, t, max_distance, delta,
            registration.TransformationEstimationPointToPoint(False),
            registration.ICPConvergenceCriteria(max_iteration=60))
        if len(result.correspondence_set) < 30:
            raise ValueError('Insufficient correspondences: approximate map transform is required')
        delta = rigid(result.transformation)
    evaluation = registration.evaluate_registration(s, t, .75, delta)
    # Undo recentering. T_A_U maps source Unity ROS-basis map points into Autoware.
    recenter = np.eye(4)
    recenter[:3, 3] = center
    estimate = rigid(recenter @ delta @ np.linalg.inv(recenter) @ initial)
    translation = float(np.linalg.norm(delta[:3, 3]))
    angle = float(np.degrees(np.arccos(np.clip((np.trace(delta[:3, :3])-1)/2, -1, 1))))
    fitness, rmse = float(evaluation.fitness), float(evaluation.inlier_rmse)
    passed = bool(fitness >= min_fitness and rmse <= max_rmse and
                  translation <= max_refinement_translation and angle <= max_refinement_angle_deg)
    report = {'accepted': passed, 'autoware_from_unity_ros': estimate.tolist(),
              'fitness': fitness, 'rmse_m': rmse,
              'refinement_translation_at_crop_center_m': translation,
              'refinement_rotation_deg': angle,
              'source_local_points': len(source), 'target_local_points': len(target),
              'center_autoware': center.tolist(), 'radius_m': radius,
              'meaning': 'Numerical screening only; independently inspect feature alignment before use.'}
    return report, transform(source, estimate)
