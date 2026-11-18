using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.PointCloud.Editor
{
    public static class LincolnAlignmentChecks
    {
        [MenuItem("UB-MR/Lincoln Alignment/Run Offline Unity Checks")]
        public static void Run()
        {
            CheckCorrectionComposition();
            CheckWorldFrameSelectionAndPayload();
            Debug.Log("LINCOLN_ALIGNMENT_CHECKS_PASSED: frame conversion, parent composition, large display origin, world selection and PointCloud2 payload.");
        }

        private static void CheckCorrectionComposition()
        {
            Vector3 oldPosition = new Vector3(23, 5, -12);
            Quaternion oldRotation = Quaternion.Euler(12, 25, -9);
            Quaternion rosRotation = Quaternion.Euler(7, -11, 18);
            double ox = 500000.125, oy = 4200000.25, oz = 100.5;
            Vector3 offset = new Vector3(.03125f, -.0625f, .125f);
            MapAlignmentMath.Correction(ox + offset.x, oy + offset.y, oz + offset.z,
                rosRotation.x, rosRotation.y, rosRotation.z, rosRotation.w, ox, oy, oz,
                out Vector3 translation, out Quaternion rotation);
            Close(translation, Ros2Utility.Ros2ToUnityPosition(offset));
            // Include a pre-existing map rotation and translation; the candidate is a correction
            // in world coordinates, not a replacement for the parent's local transform.
            Vector3 newPosition = rotation * oldPosition + translation;
            Quaternion newRotation = rotation * oldRotation;
            foreach (Vector3 local in new[] { Vector3.zero, Vector3.right, Vector3.up, new Vector3(-3, 8, 11) })
            {
                Vector3 before = oldRotation * local + oldPosition;
                Vector3 expected = Ros2Utility.Ros2ToUnityPosition(
                    rosRotation * Ros2Utility.UnityToRos2Position(before) + offset);
                Close(newRotation * local + newPosition, expected);
            }
            bool rejected = false;
            try
            {
                MapAlignmentMath.Correction(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, out _, out _);
            }
            catch (ArgumentException) { rejected = true; }
            if (!rejected) throw new Exception("Zero quaternion was accepted.");
        }

        private static void CheckWorldFrameSelectionAndPayload()
        {
            var root = new GameObject("Lincoln alignment test map");
            var cloud = new GameObject("Lincoln alignment test cloud");
            var lidar = new GameObject("Lincoln alignment test sensor");
            var mesh = new Mesh();
            try
            {
                root.transform.SetPositionAndRotation(new Vector3(10, 2, -4), Quaternion.Euler(0, 31, 0));
                cloud.transform.SetParent(root.transform, false);
                cloud.transform.localPosition = new Vector3(1, 2, 3);
                mesh.vertices = new[] { new Vector3(1, 2, 3), new Vector3(1000, 0, 0) };
                var filter = cloud.AddComponent<MeshFilter>();
                filter.sharedMesh = mesh;
                lidar.transform.position = cloud.transform.TransformPoint(mesh.vertices[0]);
                var selector = cloud.AddComponent<NearbyPointCloudSelector>();
                var settings = new SerializedObject(selector);
                settings.FindProperty("pointCloudMeshFilter").objectReferenceValue = filter;
                settings.FindProperty("lidarOverride").objectReferenceValue = lidar.transform;
                settings.FindProperty("radius").floatValue = 20f;
                settings.ApplyModifiedPropertiesWithoutUndo();
                selector.SelectNearbyPoints();
                if (selector.NearbyRosWorldPoints.Count != 1) throw new Exception("Incorrect spherical crop.");
                Vector3 worldRos = selector.NearbyRosWorldPoints[0];
                Close(worldRos, Ros2Utility.UnityToRos2Position(lidar.transform.position));
                // Turning the simulated LiDAR must not rotate the map coordinates sent to ROS.
                lidar.transform.rotation = Quaternion.Euler(10, 80, 3);
                selector.SelectNearbyPoints();
                Close(selector.NearbyRosWorldPoints[0], worldRos);
                byte[] payload = UnityMapAlignmentBridge.EncodeXyz(selector.NearbyRosWorldPoints);
                if (payload.Length != 12) throw new Exception("Invalid XYZ payload size.");
                using (var stream = new MemoryStream(payload))
                using (var reader = new BinaryReader(stream))
                    Close(new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()), worldRos);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(root);
                UnityEngine.Object.DestroyImmediate(lidar);
                UnityEngine.Object.DestroyImmediate(mesh);
            }
        }

        private static void Close(Vector3 actual, Vector3 expected)
        {
            if ((actual - expected).magnitude > .0002f)
                throw new Exception($"Alignment check failed: actual={actual:R}, expected={expected:R}");
        }
    }
}
