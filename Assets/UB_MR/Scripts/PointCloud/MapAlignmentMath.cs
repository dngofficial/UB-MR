using System;
using UnityEngine;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.PointCloud
{
    public static class MapAlignmentMath
    {
        // C T_A_U C^-1, after subtracting the chosen Autoware display origin.
        // The quaternion conversion accounts for the change of handedness.
        public static void Correction(double tx, double ty, double tz,
            double qx, double qy, double qz, double qw,
            double originX, double originY, double originZ,
            out Vector3 translation, out Quaternion rotation)
        {
            foreach (double value in new[] { tx, ty, tz, qx, qy, qz, qw, originX, originY, originZ })
                if (double.IsNaN(value) || double.IsInfinity(value))
                    throw new ArgumentException("Alignment contains a non-finite value.");
            double norm = Math.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            if (Math.Abs(norm - 1.0) > 0.001)
                throw new ArgumentException("Alignment quaternion must be normalized.");
            // Subtract in double precision before converting large map coordinates to float.
            translation = Ros2Utility.Ros2ToUnityPosition(
                new Vector3((float)(tx - originX), (float)(ty - originY), (float)(tz - originZ)));
            if (!IsFinite(translation)) throw new ArgumentException("Display translation exceeds float range.");
            rotation = Ros2Utility.Ros2ToUnityQuaternion(
                new Quaternion((float)(qx / norm), (float)(qy / norm), (float)(qz / norm), (float)(qw / norm)));
        }

        public static bool IsFinite(Vector3 p) =>
            !float.IsNaN(p.x) && !float.IsInfinity(p.x) &&
            !float.IsNaN(p.y) && !float.IsInfinity(p.y) &&
            !float.IsNaN(p.z) && !float.IsInfinity(p.z);

        public static bool SameMatrix(Matrix4x4 a, Matrix4x4 b, float tolerance = 0.0001f)
        {
            for (int i = 0; i < 16; i++)
                if (Mathf.Abs(a[i] - b[i]) > tolerance) return false;
            return true;
        }
    }
}
