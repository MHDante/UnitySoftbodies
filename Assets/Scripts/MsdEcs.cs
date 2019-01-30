using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MsdEcs : MonoBehaviour
{

    [Range(0.1f, 10)]
    public float ClusterSize = 1;

    public bool lines;
    public List<Cluster> clusters = new List<Cluster>();
    private Vector3[] verts;
    private MeshFilter meshFilter;

    public void Awake()
    {

        var meshCollider = GetComponent<MeshCollider>();
        var min = meshCollider.sharedMesh.bounds.min;
        var max = meshCollider.sharedMesh.bounds.max;
        var halfC = ClusterSize / 2;
        var parent = new GameObject("Clusters");
        parent.transform.SetParent(this.transform, false);
        verts = meshCollider.sharedMesh.vertices;
        var zero = transform.InverseTransformPoint(Vector3.zero);
        var right = transform.InverseTransformPoint(Vector3.right);
        var up = transform.InverseTransformPoint(Vector3.up);
        var forward = transform.InverseTransformPoint(Vector3.forward);

        var worldExtents = new Vector3(
            Vector3.Distance(zero, right),
            Vector3.Distance(zero, up),
            Vector3.Distance(zero, forward)
        );
        var halfextents = worldExtents * .5f * ClusterSize;

        for (float x = min.x + halfC; x < max.x + halfC; x += ClusterSize)
        {
            for (float y = min.y + halfC; y < max.y + halfC; y += ClusterSize)
            {
                for (float z = min.z + halfC; z < max.z + halfC; z += ClusterSize)
                {
                    var center = new Vector3(x, y, z);
                    var cols = Physics.OverlapBox(center.ToWorld(transform), halfextents, transform.rotation);
                    if (!cols.Contains(meshCollider)) continue;

                    var cluster = new GameObject("Cluster").AddComponent<Cluster>();
                    cluster.Sphere = cluster.gameObject.AddComponent<SphereCollider>();
                    cluster.RigidBody = cluster.gameObject.AddComponent<Rigidbody>();
                    cluster.transform.SetParent(parent.transform, false);
                    cluster.transform.localPosition = center;
                    cluster.transform.localScale = Vector3.one * ClusterSize;
                    CubeContains(new Vector3(x, y, z), ClusterSize, verts, cluster);

                    foreach (var c2 in clusters)
                    {
                        Physics.IgnoreCollision(cluster.Sphere, c2.Sphere, true);
                    }
                    clusters.Add(cluster);
                }
            }
        }

        Destroy(meshCollider);
        Destroy(GetComponent<Rigidbody>());

        meshFilter = GetComponent<MeshFilter>();
        verts = meshFilter.mesh.vertices;
    }

    public void FixedUpdate()
    {
        foreach (var cluster in clusters)
        {
            for (var i = 0; i < cluster.Offsets.Length; i++)
            {
                var index = cluster.VertIndexes[i];
                var offset = cluster.Offsets[i];

                verts[index] = cluster.transform.localPosition + offset;
            }
        }
        
        meshFilter.mesh.vertices = verts;
    }

    private static void CubeContains(Vector3 center, float sideLength, Vector3[] point, Cluster c)
    {
        var halfv = new Vector3(sideLength, sideLength, sideLength) * .5f;
        var min = center - halfv;
        var max = center + halfv;
        List<int> indexes = new List<int>();
        List<Vector3> offsets = new List<Vector3>();
        for (int i = 0; i < point.Length; i++)
        {

            if (point[i].GreaterThanEq(min) && point[i].LessThanEq(max))
            {
                indexes.Add(i);
                offsets.Add(point[i] - center);
            }

        }

        c.Offsets = offsets.ToArray();
        c.VertIndexes = indexes.ToArray();
    }


    public void OnDrawGizmosSelected()
    {
        if (!lines) return;

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
    public SphereCollider Sphere;
    public Rigidbody RigidBody;
    public Vector3[] Offsets;
}
