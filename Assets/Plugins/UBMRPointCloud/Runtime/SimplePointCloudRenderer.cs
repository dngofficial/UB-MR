using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

[RequireComponent(typeof(SimplePcdLoader))]
[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class SimplePointCloudRenderer : MonoBehaviour
{
    [SerializeField] private Material pointMaterial;

    private Mesh pointMesh;

    private void Start()
    {
        SimplePcdLoader loader = GetComponent<SimplePcdLoader>();

        if (loader.Points.Count == 0)
        {
            loader.LoadPcd();
        }

        BuildPointMesh(loader.Points);
    }

    private void BuildPointMesh(IReadOnlyList<Vector3> points)
    {
        if (points == null || points.Count == 0)
        {
            Debug.LogError("No point-cloud data is available.");
            return;
        }

        Vector3[] vertices = new Vector3[points.Count];
        int[] indices = new int[points.Count];

        for (int i = 0; i < points.Count; i++)
        {
            vertices[i] = points[i];
            indices[i] = i;
        }

        pointMesh = new Mesh
        {
            name = "PointCloudMesh",
            indexFormat = points.Count > 65535
                ? IndexFormat.UInt32
                : IndexFormat.UInt16
        };

        pointMesh.vertices = vertices;
        pointMesh.SetIndices(indices, MeshTopology.Points, 0);
        pointMesh.RecalculateBounds();

        GetComponent<MeshFilter>().sharedMesh = pointMesh;

        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();

        if (pointMaterial != null)
        {
            meshRenderer.sharedMaterial = pointMaterial;
        }
        else
        {
            Debug.LogWarning(
                "Point material is not assigned. Create and assign a material using the UBMR/PointCloud shader.");
        }

        Debug.Log($"Rendered {points.Count} point-cloud points.");
    }

    private void OnDestroy()
    {
        if (pointMesh != null)
        {
            Destroy(pointMesh);
        }
    }
}
