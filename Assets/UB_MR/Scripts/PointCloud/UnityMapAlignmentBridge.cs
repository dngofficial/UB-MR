using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;
using TransformStamped = geometry_msgs.msg.TransformStamped;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.PointCloud
{
    // Publishes one frozen reference snapshot repeatedly for volatile ROS subscribers.
    // ROS callbacks copy scalar values only; Unity transforms are accessed on the main thread.
    public class UnityMapAlignmentBridge : MonoBehaviour
    {
        [SerializeField] private NearbyPointCloudSelector selector;
        [SerializeField] private Module module;
        [SerializeField] private string referenceTopic = "/unity/reference_cloud_ros";
        [SerializeField] private string candidateTopic = "/map_alignment/candidate_transform";
        [SerializeField] private string unityMapFrame = "unity_map_ros";
        [SerializeField] private string autowareMapFrame = "map";
        [SerializeField, Min(0.1f)] private float publishIntervalSeconds = 1f;

        [Header("Autoware display origin (must match vehicle localization bridge)")]
        [SerializeField] private double autowareOriginX;
        [SerializeField] private double autowareOriginY;
        [SerializeField] private double autowareOriginZ;
        [Tooltip("Enable after checking the ROS map origin used by /ub_mr/localization.")]
        [SerializeField] private bool displayOriginConfirmed;
        [Tooltip("Apply a numerically accepted candidate immediately. Otherwise use Apply Candidate Alignment after inspecting RViz.")]
        [SerializeField] private bool applyAutomatically;

        private ROS2Node node;
        private IPublisher<PointCloud2> publisher;
        private ISubscription<TransformStamped> subscription;
        private PointCloud2 snapshot;
        private Matrix4x4 rootAtCapture, cloudAtCapture;
        private Vector3 rootPosition;
        private Quaternion rootRotation;
        private int selectionVersion;
        private bool applied;
        private readonly object candidateLock = new object();
        private Candidate pending, candidate;
        private bool hasPending, hasCandidate;

        [Serializable]
        private struct Candidate
        {
            public string parent, child;
            public int sec;
            public uint nanosec;
            public double tx, ty, tz, qx, qy, qz, qw;
        }

        private IEnumerator Start()
        {
            if (selector == null || module == null || module.MapRoot == null ||
                string.IsNullOrWhiteSpace(unityMapFrame) || string.IsNullOrWhiteSpace(autowareMapFrame) ||
                unityMapFrame == autowareMapFrame || float.IsNaN(publishIntervalSeconds) ||
                float.IsInfinity(publishIntervalSeconds) || publishIntervalSeconds < 0.1f)
            {
                Debug.LogError("UnityMapAlignmentBridge: Assign selector/module and distinct map frames; check publish interval.");
                yield break;
            }
            while (selector.SelectionVersion == 0) yield return null;
            while (!ROS2_Bridge.ROS_CORE.Ok()) yield return null;
            // ROS2ForUnity sets the native message-library path during initialization.
            if (!CaptureReference()) yield break;
            node = ROS2_Bridge.ROS_CORE.CreateNode("unity_map_alignment_" + Math.Abs((long)GetInstanceID()));
            var qos = new QualityOfServiceProfile();
            qos.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 1);
            publisher = node.CreatePublisher<PointCloud2>(referenceTopic, qos);
            var candidateQos = new QualityOfServiceProfile();
            candidateQos.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 1);
            candidateQos.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            subscription = node.CreateSubscription<TransformStamped>(candidateTopic, ReceiveCandidate, candidateQos);
            // This stamp is a snapshot identity, not LiDAR measurement time. Keep it fixed.
            node.clock.UpdateROSClockTime(snapshot.Header.Stamp);
            Debug.Log($"Unity alignment: publishing {snapshot.Width:N0} frozen WORLD/ROS points on {referenceTopic}, frame={unityMapFrame}. " +
                "Use /map_alignment/align_once once Autoware localization is valid and stationary.");
            var wait = new WaitForSecondsRealtime(publishIntervalSeconds);
            while (true)
            {
                publisher.Publish(snapshot);
                yield return wait;
            }
        }

        private bool CaptureReference()
        {
            selector.SelectNearbyPoints();
            Transform root = module.MapRoot;
            MeshFilter cloud = selector.PointCloudMeshFilter;
            if (cloud == null || !cloud.transform.IsChildOf(root) || selector.NearbyRosWorldPoints.Count < 100)
            {
                Debug.LogError("Unity alignment: selected mesh must be under Module.MapRoot and contain at least 100 nearby points.");
                return false;
            }
            if (selector.LidarTransform == null || selector.LidarTransform.IsChildOf(root))
            {
                Debug.LogError("Unity alignment: the vehicle/LiDAR must be outside the environment parent.");
                return false;
            }
            // A scaled sensor would make LiDAR-local coordinates nonmetric; reject it explicitly.
            if ((selector.LidarTransform.lossyScale - Vector3.one).sqrMagnitude > 1e-8f ||
                root.localToWorldMatrix.determinant <= 0f ||
                (root.parent != null && (root.parent.lossyScale - Vector3.one).sqrMagnitude > 1e-8f))
            {
                Debug.LogError("Unity alignment: use a unit-scale sensor, unit-scale ancestors of MapRoot, and no root reflection.");
                return false;
            }
            rootAtCapture = root.localToWorldMatrix;
            cloudAtCapture = cloud.transform.localToWorldMatrix;
            rootPosition = root.position;
            rootRotation = root.rotation;
            selectionVersion = selector.SelectionVersion;
            snapshot = MakeCloud(selector.NearbyRosWorldPoints, unityMapFrame);
            return true;
        }

        public static PointCloud2 MakeCloud(IReadOnlyList<Vector3> rosWorldPoints, string frame)
        {
            var msg = new PointCloud2();
            msg.Header.Frame_id = frame;
            msg.Height = 1;
            msg.Width = (uint)rosWorldPoints.Count;
            msg.Point_step = 12;
            msg.Row_step = checked(msg.Width * msg.Point_step);
            msg.Is_dense = true;
            msg.Fields = new[]
            {
                new PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 },
                new PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 },
                new PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 }
            };
            msg.Is_bigendian = false;
            msg.Data = EncodeXyz(rosWorldPoints);
            return msg;
        }

        public static byte[] EncodeXyz(IReadOnlyList<Vector3> rosWorldPoints)
        {
            // BinaryWriter produces little-endian floats on every platform.
            using (var stream = new MemoryStream(checked(rosWorldPoints.Count * 12)))
            using (var writer = new BinaryWriter(stream))
            {
                foreach (Vector3 point in rosWorldPoints)
                {
                    if (!MapAlignmentMath.IsFinite(point)) throw new ArgumentException("Reference points must be finite.");
                    writer.Write(point.x);
                    writer.Write(point.y);
                    writer.Write(point.z);
                }
                return stream.ToArray();
            }
        }

        private void ReceiveCandidate(TransformStamped msg)
        {
            var t = msg.Transform.Translation;
            var q = msg.Transform.Rotation;
            var copy = new Candidate { parent = msg.Header.Frame_id, child = msg.Child_frame_id,
                sec = msg.Header.Stamp.Sec, nanosec = msg.Header.Stamp.Nanosec,
                tx = t.X, ty = t.Y, tz = t.Z, qx = q.X, qy = q.Y, qz = q.Z, qw = q.W };
            lock (candidateLock) { pending = copy; hasPending = true; }
        }

        private void Update()
        {
            Candidate received;
            lock (candidateLock)
            {
                if (!hasPending) return;
                received = pending;
                hasPending = false;
            }
            if (snapshot == null || received.parent != autowareMapFrame || received.child != unityMapFrame ||
                received.sec != snapshot.Header.Stamp.Sec || received.nanosec != snapshot.Header.Stamp.Nanosec)
            {
                Debug.LogWarning("Unity alignment: ignored candidate for another frame or reference snapshot.");
                return;
            }
            if (applied) return;
            candidate = received;
            hasCandidate = true;
            Debug.Log("Unity alignment candidate received. Inspect /map_alignment/report and RViz, then use Apply Candidate Alignment on this component.");
            if (applyAutomatically) ApplyCandidateAlignment();
        }

        [ContextMenu("Apply Candidate Alignment")]
        public void ApplyCandidateAlignment()
        {
            if (!Application.isPlaying || !hasCandidate || applied || !displayOriginConfirmed)
            {
                Debug.LogWarning("Unity alignment: requires Play mode, a candidate for this snapshot, and a confirmed display origin; applies once per session.");
                return;
            }
            if (selector.SelectionVersion != selectionVersion ||
                !MapAlignmentMath.SameMatrix(rootAtCapture, module.MapRoot.localToWorldMatrix) ||
                !MapAlignmentMath.SameMatrix(cloudAtCapture, selector.PointCloudMeshFilter.transform.localToWorldMatrix))
            {
                Debug.LogError("Unity alignment: reference changed after capture. Restart Play mode and the ROS alignment node before retrying.");
                return;
            }
            try
            {
                MapAlignmentMath.Correction(candidate.tx, candidate.ty, candidate.tz,
                    candidate.qx, candidate.qy, candidate.qz, candidate.qw,
                    autowareOriginX, autowareOriginY, autowareOriginZ, out Vector3 t, out Quaternion r);
                module.SetMapPoseFromAlignment(r * rootPosition + t, r * rootRotation);
                applied = true;
                Debug.Log("Applied alignment to Module.MapRoot (environment and parented cloud). Restart Play mode to restore the saved scene pose.");
            }
            catch (ArgumentException exc) { Debug.LogError("Unity alignment: " + exc.Message); }
        }

        private void OnDestroy()
        {
            if (node == null) return;
            if (ROS2_Bridge.ROS_CORE.Ok())
            {
                if (subscription != null) node.RemoveSubscription<TransformStamped>(subscription);
                if (publisher != null) node.RemovePublisher<PointCloud2>(publisher);
                ROS2_Bridge.ROS_CORE.RemoveNode(node);
            }
            node = null;
        }
    }
}
