using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MsdEcs : MonoBehaviour
{

    public float ClusterSize = 1;
    public List<Cluster> clusters = new List<Cluster>();
    public void Awake()
    {

        var collider = GetComponent<MeshCollider>();
        var min = collider.sharedMesh.bounds.min;
        var max = collider.sharedMesh.bounds.max;
        var halfC = ClusterSize / 2;
        var parent = new GameObject("Clusters");
        parent.transform.SetParent(this.transform, false);
        var verts = collider.sharedMesh.vertices;

        for (float x = min.x + halfC; x < max.x + halfC; x += ClusterSize)
        {
            for (float y = min.y + halfC; y < max.y + halfC; y += ClusterSize)
            {
                for (float z = min.z + halfC; z < max.z + halfC; z += ClusterSize)
                {
                    var indexes = CubeContains(new Vector3(x, y, z), ClusterSize, verts);
                    if (indexes.Length <= 0) continue;

                    var center = new Vector3(x, y, z);
                    var o = new GameObject("Cluster");
                    o.AddComponent<BoxCollider>();
                    o.transform.SetParent(parent.transform, false);
                    o.transform.localPosition = center;
                    o.transform.localScale = Vector3.one * ClusterSize;
                    var cluster = o.AddComponent<Cluster>();
                    cluster.VertIndexes = indexes;
                    clusters.Add(cluster);
                }
            }
        }
    }

    private static int[] CubeContains(Vector3 center, float sideLength, Vector3[] point)
    {
        var halfv = new Vector3(sideLength, sideLength, sideLength) * .5f;
        var min = center - halfv;
        var max = center + halfv;
        List<int> indexes = new List<int>();
        for (int i = 0; i < point.Length; i++)
        {

            if (point[i].GreaterThanEq(min) && point[i].LessThanEq(max)) indexes.Add(i);
        }
        return indexes.ToArray();
    }

    public void OnDrawGizmosSelected()
    {
        var collider = GetComponent<MeshCollider>();
        var min = collider.sharedMesh.bounds.min;
        var oldMax = collider.sharedMesh.bounds.max;
        var diff = (oldMax - min) / ClusterSize;
        var ceil = new Vector3(Mathf.Ceil(diff.x), Mathf.Ceil(diff.y), Mathf.Ceil(diff.z)) * ClusterSize;
        var max = ceil + min;


        for (float x = min.x; x <= max.x + ClusterSize / 2; x += ClusterSize)
        {
            for (float y = min.y; y <= max.y + ClusterSize / 2; y += ClusterSize)
            {
                Debug.DrawLine(new Vector3(x, y, min.z).ToWorld(this.transform), new Vector3(x, y, max.z).ToWorld(this.transform));
            }
            for (float z = min.z; z <= max.z + ClusterSize / 2; z += ClusterSize)
            {
                Debug.DrawLine(new Vector3(x, min.y, z).ToWorld(this.transform), new Vector3(x, max.y, z).ToWorld(this.transform));
            }
        }
        for (float y = min.y; y <= max.y + ClusterSize / 2; y += ClusterSize)
        {
            for (float z = min.z; z <= max.z + ClusterSize / 2; z += ClusterSize)
            {
                Debug.DrawLine(new Vector3(min.x, y, z).ToWorld(this.transform), new Vector3(max.x, y, z).ToWorld(this.transform));
            }
        }



    }
}

public static class VectorExtensions
{
    public static Vector3 ToWorld(this Vector3 localPt, Transform t) => t.TransformPoint(localPt);
    public static bool LessThanEq(this Vector3 a, Vector3 b) => a.x <= b.x && a.y <= b.y && a.z <= b.z;
    public static bool GreaterThanEq(this Vector3 a, Vector3 b) => a.x >= b.x && a.y >= b.y && a.z >= b.z;
}

public class Cluster : MonoBehaviour
{
    public int[] VertIndexes;
}
