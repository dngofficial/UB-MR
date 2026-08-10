using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public class SimplePcdLoader : MonoBehaviour
{
    [Header("PCD File")]
    [SerializeField] private string pcdFilePath;

    [Header("Loading")]
    [SerializeField] private int maxPoints = 50000;
    [SerializeField] private bool loadOnStart = true;

    public IReadOnlyList<Vector3> Points => points;

    private readonly List<Vector3> points = new();

    private void Start()
    {
        if (loadOnStart)
        {
            LoadPcd();
        }
    }

    [ContextMenu("Load PCD")]
    public void LoadPcd()
    {
        points.Clear();

        if (string.IsNullOrWhiteSpace(pcdFilePath))
        {
            Debug.LogError("PCD file path is empty.");
            return;
        }

        if (!File.Exists(pcdFilePath))
        {
            Debug.LogError($"PCD file not found: {pcdFilePath}");
            return;
        }

        bool dataSectionFound = false;

        try
        {
            using StreamReader reader = new(pcdFilePath);

            while (!reader.EndOfStream && points.Count < maxPoints)
            {
                string line = reader.ReadLine();

                if (string.IsNullOrWhiteSpace(line))
                {
                    continue;
                }

                if (!dataSectionFound)
                {
                    if (line.StartsWith("DATA ascii", StringComparison.OrdinalIgnoreCase))
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
                {
                    continue;
                }

                if (
                    float.TryParse(values[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float x) &&
                    float.TryParse(values[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float y) &&
                    float.TryParse(values[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float z)
                )
                {
                    points.Add(new Vector3(x, z, y));
                }
            }

            if (!dataSectionFound)
            {
                Debug.LogError("Only ASCII PCD files are supported by this test loader.");
                return;
            }

            Debug.Log($"Loaded {points.Count} points from {pcdFilePath}");
        }
        catch (Exception exception)
        {
            Debug.LogException(exception);
        }
    }
}
