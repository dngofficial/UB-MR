using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Globalization;
using UnityEngine;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.PointCloud
{
    public class NearbyPointCloudSelector : MonoBehaviour
    {
        [Header("Point Cloud")]
        [SerializeField]
        private MeshFilter pointCloudMeshFilter;

        [Header("Runtime LiDAR")]
        [SerializeField]
        private string lidarObjectName = "LiDAR_Top";

        [Tooltip("Optional explicit sensor/selection centre; useful when several agents exist.")]
        [SerializeField] private Transform lidarOverride;

        [Header("Selection")]
        [SerializeField]
        private float radius = 30.0f;


        /*
         * Nearby points expressed in Unity WORLD coordinates.
         *
         * These are useful for visualization/debugging.
         */
        private readonly List<Vector3> nearbyWorldPoints =
            new List<Vector3>();


        /*
         * Nearby points expressed in Unity LiDAR-local coordinates.
         *
         * LiDAR_Top is treated as the origin.
         */
        private readonly List<Vector3> nearbyLidarLocalPoints =
            new List<Vector3>();


        /*
         * Nearby points expressed using ROS coordinate conventions,
         * relative to LiDAR_Top.
         *
         * These are the points that we will eventually package into
         * sensor_msgs/PointCloud2.
         */
        private readonly List<Vector3> nearbyRosPoints =
            new List<Vector3>();

        // Fixed Unity WORLD frame expressed in the ROS basis. Unlike nearbyRosPoints,
        // these coordinates do not change with the simulated LiDAR orientation.
        private readonly List<Vector3> nearbyRosWorldPoints = new List<Vector3>();


        private Transform lidarTransform;


        public IReadOnlyList<Vector3> NearbyWorldPoints =>
            nearbyWorldPoints;


        public IReadOnlyList<Vector3> NearbyLidarLocalPoints =>
            nearbyLidarLocalPoints;


        public IReadOnlyList<Vector3> NearbyRosPoints =>
            nearbyRosPoints;

        public IReadOnlyList<Vector3> NearbyRosWorldPoints => nearbyRosWorldPoints;
        public MeshFilter PointCloudMeshFilter => pointCloudMeshFilter;
        public int SelectionVersion { get; private set; }


        public Transform LidarTransform =>
            lidarOverride != null ? lidarOverride : lidarTransform;


        private IEnumerator Start()
        {
            if (pointCloudMeshFilter == null)
            {
                Debug.LogError(
                    "NearbyPointCloudSelector: Point cloud MeshFilter is not assigned."
                );

                yield break;
            }


            /*
             * CARLA Agent and LiDAR_Top are created at runtime
             * by Module.cs / Agent.cs.
             *
             * Therefore we wait until LiDAR_Top appears.
             */
            while (LidarTransform == null)
            {
                GameObject lidarObject =
                    GameObject.Find(lidarObjectName);

                if (lidarObject != null)
                {
                    lidarTransform =
                        lidarObject.transform;

                    break;
                }

                yield return null;
            }


            Debug.Log(
                $"NearbyPointCloudSelector: Found runtime LiDAR " +
                $"'{LidarTransform.name}' at world position " +
                $"{LidarTransform.position}."
            );


            SelectNearbyPoints();
        }


        [ContextMenu("Select Nearby Points")]
        public void SelectNearbyPoints()
        {
            nearbyWorldPoints.Clear();
            nearbyLidarLocalPoints.Clear();
            nearbyRosPoints.Clear();
            nearbyRosWorldPoints.Clear();
            SelectionVersion++;


            if (pointCloudMeshFilter == null)
            {
                Debug.LogError(
                    "NearbyPointCloudSelector: Point cloud MeshFilter is not assigned."
                );

                return;
            }


            if (LidarTransform == null)
            {
                Debug.LogWarning(
                    "NearbyPointCloudSelector: Runtime LiDAR has not been found yet."
                );

                return;
            }

            if (float.IsNaN(radius) || float.IsInfinity(radius) || radius <= 0f)
            {
                Debug.LogError("NearbyPointCloudSelector: Radius must be positive and finite.");
                return;
            }


            Mesh mesh =
                pointCloudMeshFilter.sharedMesh;


            if (mesh == null)
            {
                Debug.LogError(
                    "NearbyPointCloudSelector: MeshFilter has no mesh."
                );

                return;
            }


            Vector3[] localPoints =
                mesh.vertices;


            Transform cloudTransform =
                pointCloudMeshFilter.transform;


            Vector3 lidarWorldPosition =
                LidarTransform.position;


            float radiusSquared =
                radius * radius;


            /*
             * 1. Imported PCD local point
             *      ↓
             * 2. Unity world point
             *      ↓
             * 3. LiDAR local point
             *      ↓
             * 4. ROS-coordinate LiDAR point
             */
            for (int i = 0; i < localPoints.Length; i++)
            {
                /*
                 * Convert imported point-cloud local coordinate
                 * into Unity world coordinates.
                 */
                Vector3 worldPoint =
                    cloudTransform.TransformPoint(
                        localPoints[i]
                    );

                if (!IsFinite(worldPoint)) continue;


                float distanceSquared =
                    (
                        worldPoint -
                        lidarWorldPosition
                    ).sqrMagnitude;


                if (distanceSquared > radiusSquared)
                    continue;


                /*
                 * Store world-space version.
                 */
                nearbyWorldPoints.Add(
                    worldPoint
                );


                /*
                 * Express the same point relative to LiDAR_Top.
                 *
                 * LiDAR_Top becomes:
                 *
                 *     (0, 0, 0)
                 */
                Vector3 lidarLocalPoint =
                    LidarTransform.InverseTransformPoint(
                        worldPoint
                    );


                nearbyLidarLocalPoints.Add(
                    lidarLocalPoint
                );


                /*
                 * Convert Unity coordinates to ROS coordinates.
                 *
                 * UB-MR Ros2Utility:
                 *
                 * Unity (x, y, z)
                 *       ↓
                 * ROS   (z, -x, y)
                 */
                Vector3 rosPoint =
                    Ros2Utility.UnityToRos2Position(
                        lidarLocalPoint
                    );


                nearbyRosPoints.Add(
                    rosPoint
                );
                nearbyRosWorldPoints.Add(Ros2Utility.UnityToRos2Position(worldPoint));
            }


            Debug.Log(
                $"NearbyPointCloudSelector: " +
                $"Total points = {localPoints.Length:N0}, " +
                $"Nearby points within {radius:F1} m = " +
                $"{nearbyWorldPoints.Count:N0}"
            );


            Debug.Log(
                $"NearbyPointCloudSelector: Generated " +
                $"{nearbyLidarLocalPoints.Count:N0} LiDAR-local points " +
                $"and {nearbyRosPoints.Count:N0} ROS-frame points."
            );


            /*
             * Print a few example points so we can verify
             * the coordinate conversion without flooding
             * the Unity Console.
             */
            int sampleCount =
                Mathf.Min(
                    5,
                    nearbyRosPoints.Count
                );


            for (int i = 0; i < sampleCount; i++)
            {
                Debug.Log(
                    $"NearbyPointCloudSelector Sample [{i}]: " +
                    $"World={nearbyWorldPoints[i]}, " +
                    $"LiDARLocal={nearbyLidarLocalPoints[i]}, " +
                    $"ROS={nearbyRosPoints[i]}"
                );
            }
        }

        private static bool IsFinite(Vector3 p) =>
            !float.IsNaN(p.x) && !float.IsInfinity(p.x) &&
            !float.IsNaN(p.y) && !float.IsInfinity(p.y) &&
            !float.IsNaN(p.z) && !float.IsInfinity(p.z);

        [ContextMenu("Export Unity Map ROS CSV")]
        public void ExportUnityMapRosCsv()
        {
            if (nearbyRosWorldPoints.Count == 0)
            {
                Debug.LogWarning("Select nearby points before exporting the Unity map.");
                return;
            }
            string path = Path.Combine(Application.persistentDataPath, "unity_map_ros.csv");
            using (var writer = new StreamWriter(path))
            {
                writer.WriteLine("x,y,z");
                foreach (Vector3 p in nearbyRosWorldPoints)
                    writer.WriteLine(string.Join(",", p.x.ToString("R", CultureInfo.InvariantCulture),
                        p.y.ToString("R", CultureInfo.InvariantCulture), p.z.ToString("R", CultureInfo.InvariantCulture)));
            }
            Debug.Log($"Exported {nearbyRosWorldPoints.Count:N0} points in unity_map_ros to {path}");
        }



        // =====================================================
        // Export selected nearby reference points to CSV
        // =====================================================

        [ContextMenu("Export Nearby Points CSV")]
        public void ExportNearbyPointsCsv()
        {
            if (nearbyRosPoints.Count == 0)
            {
                Debug.LogWarning(
                    "NearbyPointCloudSelector: No nearby points to export."
                );
                return;
            }

            if (nearbyWorldPoints.Count != nearbyRosPoints.Count ||
                nearbyLidarLocalPoints.Count != nearbyRosPoints.Count)
            {
                Debug.LogError(
                    "NearbyPointCloudSelector: Point list counts do not match."
                );
                return;
            }

            string path = Path.Combine(
                Application.persistentDataPath,
                "nearby_reference_points.csv"
            );

            using (StreamWriter writer = new StreamWriter(path))
            {
                writer.WriteLine(
                    "world_x,world_y,world_z," +
                    "lidar_x,lidar_y,lidar_z," +
                    "ros_x,ros_y,ros_z"
                );

                for (int i = 0; i < nearbyRosPoints.Count; i++)
                {
                    Vector3 world = nearbyWorldPoints[i];
                    Vector3 lidar = nearbyLidarLocalPoints[i];
                    Vector3 ros = nearbyRosPoints[i];

                    writer.WriteLine(string.Join(",", new string[]
                    {
                        world.x.ToString("R", CultureInfo.InvariantCulture),
                        world.y.ToString("R", CultureInfo.InvariantCulture),
                        world.z.ToString("R", CultureInfo.InvariantCulture),

                        lidar.x.ToString("R", CultureInfo.InvariantCulture),
                        lidar.y.ToString("R", CultureInfo.InvariantCulture),
                        lidar.z.ToString("R", CultureInfo.InvariantCulture),

                        ros.x.ToString("R", CultureInfo.InvariantCulture),
                        ros.y.ToString("R", CultureInfo.InvariantCulture),
                        ros.z.ToString("R", CultureInfo.InvariantCulture)
                    }));
                }
            }

            Debug.Log(
                $"NearbyPointCloudSelector: Exported " +
                $"{nearbyRosPoints.Count:N0} points to {path}"
            );
        }

    } // NearbyPointCloudSelector class
}     // namespace
