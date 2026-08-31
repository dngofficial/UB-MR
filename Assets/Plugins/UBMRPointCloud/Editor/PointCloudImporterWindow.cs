using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

public class PointCloudImporterWindow : EditorWindow
{
    private string pcdFilePath = "";
    private Material pointMaterial;
    private enum CoordinateMode
    {
        CurrentXZY,
        RawXYZ,
        XZNegY,
        NegXZY

    }
    private CoordinateMode coordinateMode = CoordinateMode.CurrentXZY;

    private enum AlignmentMode
    {
        None,
        ServiceCenterLoopManual
    }

    private AlignmentMode alignmentMode = AlignmentMode.None;


    [MenuItem("UB-MR/Point Cloud Importer")]
    public static void ShowWindow()
    {
        GetWindow<PointCloudImporterWindow>("Point Cloud Importer");
    }

    private void OnGUI()
    {
        GUILayout.Label(
            "UB-MR Point Cloud Importer",
            EditorStyles.boldLabel
        );

        EditorGUILayout.Space();

        EditorGUILayout.LabelField("PCD File");

        EditorGUILayout.BeginHorizontal();

        pcdFilePath = EditorGUILayout.TextField(pcdFilePath);

        if (GUILayout.Button("Browse", GUILayout.Width(80)))
        {
            string selectedPath = EditorUtility.OpenFilePanel(
                "Select PCD File",
                "",
                "pcd"
            );

            if (!string.IsNullOrEmpty(selectedPath))
            {
                pcdFilePath = selectedPath;
            }
        }

        EditorGUILayout.EndHorizontal();

        pointMaterial = (Material)EditorGUILayout.ObjectField(
            "Point Material",
            pointMaterial,
            typeof(Material),
            false
        );

        coordinateMode = (CoordinateMode)EditorGUILayout.EnumPopup(
            "Coordinate Mode",
            coordinateMode
        );

        alignmentMode = (AlignmentMode)EditorGUILayout.EnumPopup(
            "Alignment",
            alignmentMode
        );

        EditorGUILayout.Space();

        GUI.enabled =
            !string.IsNullOrEmpty(pcdFilePath) &&
            pointMaterial != null;

        if (GUILayout.Button("Import Point Cloud", GUILayout.Height(40)))
        {
            ImportPointCloud();
        }

        GUI.enabled = true;

        EditorGUILayout.Space();

        EditorGUILayout.HelpBox(
            "ASCII PCD test importer. Creates a persistent Unity Mesh asset and places it in the current Scene.",
            MessageType.Info
        );
    }

    private void ImportPointCloud()
    {
        if (!File.Exists(pcdFilePath))
        {
            EditorUtility.DisplayDialog(
                "Import Error",
                "PCD file was not found.",
                "OK"
            );
            return;
        }

        List<Vector3> points = new List<Vector3>();
        bool dataSectionFound = false;

        try
        {
            using (StreamReader reader = new StreamReader(pcdFilePath))
            {
                while (!reader.EndOfStream)
                {
                    string line = reader.ReadLine();

                    if (string.IsNullOrWhiteSpace(line))
                        continue;

                    if (!dataSectionFound)
                    {
                        if (line.StartsWith(
                            "DATA ascii",
                            StringComparison.OrdinalIgnoreCase))
                        {
                            dataSectionFound = true;
                        }

                        continue;
                    }

                    string[] values = line.Split(
                        (char[])null,
                        StringSplitOptions.RemoveEmptyEntries
                    );

                    if (values.Length < 3)
                        continue;

                    if (
                        float.TryParse(
                            values[0],
                            NumberStyles.Float,
                            CultureInfo.InvariantCulture,
                            out float x
                        ) &&
                        float.TryParse(
                            values[1],
                            NumberStyles.Float,
                            CultureInfo.InvariantCulture,
                            out float y
                        ) &&
                        float.TryParse(
                            values[2],
                            NumberStyles.Float,
                            CultureInfo.InvariantCulture,
                            out float z
                        )
                    )
                    {
                        // PCD -> Unity coordinate conversion
                        // points.Add(new Vector3(x, z, y));

                        Vector3 point;
                        switch (coordinateMode)
                        {
                            case CoordinateMode.RawXYZ:
                                point = new Vector3(x, y, z);
                                break;
                            
                            case CoordinateMode.XZNegY:
                                point = new Vector3(x, z, -y);
                                break;

                            case CoordinateMode.NegXZY:
                                point = new Vector3(-x, z, y);
                                break;

                            case CoordinateMode.CurrentXZY:
                            default:
                                point = new Vector3(x, z, y);
                                break;
                        }
                        points.Add(point);
                        }
                    }
                }
            

            if (!dataSectionFound)
            {
                EditorUtility.DisplayDialog(
                    "Import Error",
                    "This test importer currently supports DATA ascii PCD files only.",
                    "OK"
                );
                return;
            }

            if (points.Count == 0)
            {
                EditorUtility.DisplayDialog(
                    "Import Error",
                    "No points were found in the PCD file.",
                    "OK"
                );
                return;
            }

            CreateUnityAsset(points);
        }
        catch (Exception exception)
        {
            Debug.LogException(exception);

            EditorUtility.DisplayDialog(
                "Import Error",
                exception.Message,
                "OK"
            );
        }
    }

    private void CreateUnityAsset(List<Vector3> points)
    {
        const string rootFolder = "Assets/UB_MR/PointClouds";
        const string testFolder = "Assets/UB_MR/PointClouds/Imported";

        if (!AssetDatabase.IsValidFolder(rootFolder))
        {
            AssetDatabase.CreateFolder(
                "Assets/UB_MR",
                "PointClouds"
            );
        }

        if (!AssetDatabase.IsValidFolder(testFolder))
        {
            AssetDatabase.CreateFolder(
                rootFolder,
                "Imported"
            );
        }

        string cloudName =
            Path.GetFileNameWithoutExtension(pcdFilePath);

        string assetPath =
            $"{testFolder}/{cloudName}.asset";

        assetPath =
            AssetDatabase.GenerateUniqueAssetPath(assetPath);

        Mesh mesh = new Mesh();

        mesh.name = cloudName;
        mesh.indexFormat = IndexFormat.UInt32;

        Vector3[] vertices = points.ToArray();
        int[] indices = new int[vertices.Length];

        for (int i = 0; i < indices.Length; i++)
        {
            indices[i] = i;
        }

        mesh.vertices = vertices;

        mesh.SetIndices(
            indices,
            MeshTopology.Points,
            0,
            false
        );

        mesh.RecalculateBounds();

        AssetDatabase.CreateAsset(mesh, assetPath);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        GameObject pointCloudObject =
            new GameObject("PointCloud_" + cloudName);

        MeshFilter meshFilter =
            pointCloudObject.AddComponent<MeshFilter>();

        MeshRenderer meshRenderer =
            pointCloudObject.AddComponent<MeshRenderer>();

        meshFilter.sharedMesh = mesh;
        meshRenderer.sharedMaterial = pointMaterial;

        if (alignmentMode == AlignmentMode.ServiceCenterLoopManual)
        {
            if (coordinateMode != CoordinateMode.CurrentXZY)
            {
                Debug.LogWarning(
                    "ServiceCenterLoopManual alignment was calibrated for CurrentXZY coordinates."
                );
            }

            GameObject mapRootObject = GameObject.Find("ServiceCenterLoopMR");
            if (mapRootObject == null)
            {
                Debug.LogWarning(
                    "ServiceCenterLoopMR was not found. " +
                    "Point cloud was created without alignment."
                );
            }
            else
            {
                Transform pointCloudTransform = pointCloudObject.transform;
                // Parent the point cloud under the RoadRunner map.
                pointCloudTransform.SetParent(mapRootObject.transform, false);

                // Manual registration transform established for
                // pointcloud_uniform_50k using CurrentXZY coordinates.
                pointCloudTransform.localPosition =
                    new Vector3(2.858025f, -2.7407467f, 0f);
                pointCloudTransform.localRotation =
                    new Quaternion(
                        0.0074047437f,
                        -0.707068f,
                        -0.707068f,
                        0.0074047437f
                    );

                pointCloudTransform.localScale = Vector3.one;
            }
        }

        Undo.RegisterCreatedObjectUndo(
            pointCloudObject,
            "Import Point Cloud"
        );

        Selection.activeGameObject = pointCloudObject;

        EditorSceneManager.MarkSceneDirty(
            EditorSceneManager.GetActiveScene()
        );

        Debug.Log(
            $"Imported {points.Count:N0} points. Unity asset created at: {assetPath}"
        );

        EditorUtility.DisplayDialog(
            "Point Cloud Import Complete",
            $"Imported {points.Count:N0} points.\n\n" +
            $"Unity asset:\n{assetPath}\n\n" +
            "The point cloud should now be visible in Scene View without entering Play mode.",
            "OK"
        );
    }
}
