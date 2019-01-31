using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using UnityEngine;
using Unity.Mathematics;
using UnityEditor;

public class MsdEcs : MonoBehaviour
{

    [Range(0.1f, 10)]
    public float ClusterSize = 1;

    public bool Lines;
    public float alpha;
    public List<Cluster> Clusters = new List<Cluster>();
    private Vector3[] _verts;
    private float3[] q;
    private float[] m;
    private float3[] p;
    private float3[] x;
    private quaternion? r;
    private MeshFilter _meshFilter;

    public void Awake()
    {

        var meshCollider = GetComponent<MeshCollider>();
        var min = meshCollider.sharedMesh.bounds.min;
        var max = meshCollider.sharedMesh.bounds.max;
        var halfC = ClusterSize / 2;
        var parent = new GameObject("Clusters");
        parent.transform.SetParent(transform, false);
        _verts = meshCollider.sharedMesh.vertices;
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

        for (var x = min.x + halfC; x < max.x + halfC; x += ClusterSize)
        {
            for (var y = min.y + halfC; y < max.y + halfC; y += ClusterSize)
            {
                for (var z = min.z + halfC; z < max.z + halfC; z += ClusterSize)
                {
                    var center = new Vector3(x, y, z);
                    var cols = Physics.OverlapBox(center.ToWorld(transform), halfextents, transform.rotation);
                    if (!cols.Contains(meshCollider)) continue;

                    var cluster = new GameObject("Cluster").AddComponent<Cluster>();
                    cluster.Sphere = cluster.gameObject.AddComponent<SphereCollider>();
                    cluster.RigidBody = cluster.gameObject.AddComponent<Rigidbody>();
                    cluster.RigidBody.useGravity = false;
                    cluster.RigidBody.drag = 0.1f;
                    cluster.RigidBody.velocity = Vector3.zero;
                    cluster.RigidBody.freezeRotation = true;
                    cluster.transform.SetParent(parent.transform, false);
                    cluster.transform.localPosition = center;
                    cluster.transform.localScale = Vector3.one * ClusterSize / 2;
                    CubeContains(new Vector3(x, y, z), ClusterSize, _verts, cluster);

                    foreach (var c2 in Clusters)
                    {
                        Physics.IgnoreCollision(cluster.Sphere, c2.Sphere, true);
                    }

                    Clusters.Add(cluster);
                }
            }
        }

        var x0 = Clusters.Select(c => (float3)c.transform.position).ToArray();
        p = new float3[x0.Length];
        m = new float[x0.Length];
        Fill(m,1);
        x = new float3[x0.Length];
        var x0cm = Sum(x0, m) / Sum(m);
        q = x0.Select(x0i => x0i - x0cm).ToArray();

        Array.Copy(q, p, q.Length);
        Destroy(meshCollider);
        Destroy(GetComponent<Rigidbody>());

        _meshFilter = GetComponent<MeshFilter>();
    }

    private void AddClusters()
    {

    }


    public static void Fill<T>(T[] array, T value)
    {
        for (int i = 0; i < array.Length; i++)
        {
            array[i] = value;
        }
    }

    public void FixedUpdate()
    {
        for (int i = 0; i < p.Length; i++)
        {
            x[i] = Clusters[i].RigidBody.position;
        }

        var xcm = Sum(x, m)/Sum(m);
        for (int i = 0; i < p.Length; i++)
        {
            p[i] = x[i]-xcm;
        }

        float3x3 Apq = float3x3.zero;
        for (int i = 0; i < p.Length; i++)
        {
            Apq +=  m[i]* p[i].MulTranspose(q[i]);
        }

        var rq = r ?? math.normalize(math.quaternion(Apq));
        ExtractRotation(ref Apq, ref rq);
        r = rq;
        
        for (int i = 0; i < p.Length; i++)
        {
            var gi = math.mul(r.Value , q[i]) + xcm;
            var dif = alpha * (gi - x[i]) / Time.fixedDeltaTime;
            Clusters[i].RigidBody.velocity += (Vector3)dif;
        }
        
        UpdateClusters();
    }

    public void UpdateClusters()
    {


        foreach (var cluster in Clusters)
        {
            for (var i = 0; i < cluster.Offsets.Length; i++)
            {
                var index = cluster.VertIndexes[i];
                var offset = cluster.Offsets[i];

                _verts[index] = cluster.transform.localPosition + offset;
            }
        }

        _meshFilter.mesh.vertices = _verts;
    }

    private static void CubeContains(Vector3 center, float sideLength, Vector3[] point, Cluster c)
    {
        var halfv = new Vector3(sideLength, sideLength, sideLength) * .5f;
        var min = center - halfv;
        var max = center + halfv;
        var indexes = new List<int>();
        var offsets = new List<Vector3>();
        for (var i = 0; i < point.Length; i++)
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

    public float3 Sum(float3[] value, float[] weight = null)
    {
        float3 r = 0;
        for (var i = 0; i < value.Length; i++)
        {
            r += weight?[i] * value[i] ?? value[i];
        }

        return r;
    }

    
    public float3 Sum(float[] value, float[] weight = null)
    {
        float3 r = 0;
        for (var i = 0; i < value.Length; i++)
        {
            r += weight?[i] * value[i] ?? value[i];
        }

        return r;
    }

    // From: https://animation.rwth-aachen.de/media/papers/2016-MIG-StableRotation.pdf
    void ExtractRotation(ref float3x3 a, ref quaternion q, int maxIter = 9)
    {

        var r = new float3x3(q);
        for (var i = 0; i < maxIter; i++)
        {
            var omega = (math.cross(r[0], a[0]) + math.cross(r[1], a[1]) + math.cross(r[2], a[2])) *
                        (1f / math.abs(math.dot(r[0], a[0]) + math.dot(r[1], a[1]) + math.dot(r[2], a[2]) + 0.000000001f));
            var w = math.length(omega);
            if (w < 1.0e-9) break;
            q = math.mul(quaternion.AxisAngle(omega / w, w * Mathf.Rad2Deg), q);
            q = math.normalize(q);
        }
    }

    public void OnDrawGizmosSelected()
    {
        if (!Lines) return;

        var collider = GetComponent<MeshCollider>();
        var min = collider.sharedMesh.bounds.min;
        var oldMax = collider.sharedMesh.bounds.max;
        var diff = (oldMax - min) / ClusterSize;
        var ceil = new Vector3(Mathf.Ceil(diff.x), Mathf.Ceil(diff.y), Mathf.Ceil(diff.z)) * ClusterSize;
        var max = ceil + min;


        for (var x = min.x; x <= max.x + ClusterSize / 2; x += ClusterSize)
        {
            for (var y = min.y; y <= max.y + ClusterSize / 2; y += ClusterSize)
            {
                Debug.DrawLine(new Vector3(x, y, min.z).ToWorld(this.transform), new Vector3(x, y, max.z).ToWorld(this.transform));
            }
            for (var z = min.z; z <= max.z + ClusterSize / 2; z += ClusterSize)
            {
                Debug.DrawLine(new Vector3(x, min.y, z).ToWorld(this.transform), new Vector3(x, max.y, z).ToWorld(this.transform));
            }
        }
        for (var y = min.y; y <= max.y + ClusterSize / 2; y += ClusterSize)
        {
            for (var z = min.z; z <= max.z + ClusterSize / 2; z += ClusterSize)
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

    
    /// <summary>Returns the float3x3 matrix result of a matrix multiplication between a float3x2 matrix and a float2x3 matrix.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float3x3 MulTranspose(this float3 a, float3 b)
    {
        return math.float3x3(a*b.x, a*b.y,a*b.z);
    }
}

public class Cluster : MonoBehaviour
{
    public int[] VertIndexes;
    public SphereCollider Sphere;
    public Rigidbody RigidBody;
    public Vector3[] Offsets;
}
