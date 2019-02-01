using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using UnityEngine;
using Unity.Mathematics;

public class MsdEcs : MonoBehaviour
{
    
    public float Alpha = 1;
    [SerializeField] private float Dampening;
    [SerializeField] private float Drag;

    private Vector3[] vertices;
    private float3[] q;
    private float3[] x;
    private float[] m;
    private MeshFilter meshFilter;
    private Rigidbody[] bodies;
    private float3 x0Cm;
    private quaternion rq = quaternion.identity;

    public void Awake()
    {

        //var oldCollider = GetComponent<Collider>();
        meshFilter = GetComponent<MeshFilter>();

        vertices = meshFilter.mesh.vertices;
        bodies = new Rigidbody[vertices.Length];
        var colliders = new List<Collider>();

        var parent = new GameObject("Vertices").transform;
        parent.SetParent(transform,false);

        for (var i = 0; i < vertices.Length; i++)
        {
            var o = new GameObject("Vertex " + i);
            o.transform.SetParent(parent,false);
            o.transform.localPosition = vertices[i];
            var c = o.AddComponent<SphereCollider>();
            foreach (var cc in colliders)
            {
                Physics.IgnoreCollision(c, cc, true);
            }
            colliders.Add(c);
            c.radius = 0.1f;
            var rb = o.AddComponent<Rigidbody>();
            rb.velocity = Vector3.zero;
            rb.freezeRotation = true;
            rb.useGravity = true;
            rb.isKinematic = false;
            rb.drag = Drag;
            rb.constraints = RigidbodyConstraints.FreezeRotation;
            bodies[i] = rb;
        }
        
        x = new float3[bodies.Length];
        m = new float[bodies.Length];
        GetPositionsAndMasses(bodies,x,m,false);
        x0Cm = CenterOfMass(x,m);
        q = x.Select(x0i => x0i - x0Cm).ToArray();
    }

    public void FixedUpdate()
    {
        
        GetPositionsAndMasses(bodies,x,m,false);
        var xcm = CenterOfMass(x,m);
        
        float3x3 Apq = float3x3.zero;
        for (int i = 0; i < x.Length; i++)
        {
            var pi = x[i] - xcm;
            var mat = pi.MulTranspose(q[i]);
            Apq += bodies[i].mass * mat;
        }
        
        ExtractRotation(ref Apq, ref rq);
        var A = new Vector3[]{Apq[0],Apq[1],Apq[2]};
        rq = math.normalize(rq);
        var r = ExtractRotation(ref Apq);
        for (int i = 0; i < x.Length; i++)
        {
            var rot1 = math.mul(rq, q[i]);
            var rot2 = math.mul(r, q[i]);
            var gi = rot1 + xcm;
            var diff = gi - x[i];
            var accel = (Alpha/Time.fixedDeltaTime) * diff;
            //bodies[i].velocity = accel;
            //bodies[i].MovePosition(gi);
            bodies[i].velocity *= Dampening;
            bodies[i].velocity += (Vector3)accel;
            //bodies[i].AddForce(accel, ForceMode.VelocityChange);
        }
    }

    void DrawPt(Vector3 gi, Color? color = null)
    {;
        var c = color ?? Color.white;
        Debug.DrawLine(gi,gi+new Vector3(0,0,.1f),c);
        Debug.DrawLine(gi,gi+new Vector3(0,.1f,0),c);
        Debug.DrawLine(gi,gi+new Vector3(.1f,0,0),c);

    }
    private void Update()
    {
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = this.transform.InverseTransformPoint(bodies[i].position);
        }

        meshFilter.mesh.vertices = vertices;
    }

    private void GetPositionsAndMasses(Rigidbody[] rs, float3[] ps, float[] ms, bool includeVelocity)
    {
        for (int i = 0; i < rs.Length; i++)
        {
            ps[i] = includeVelocity? rs[i].position + rs[i].velocity*Time.fixedDeltaTime : rs[i].position;
            ms[i] = rs[i].mass;
        }
    }

    public static float3 CenterOfMass(float3[] ps, float[] ms)
    {
        float3 r = float3.zero;
        float m = 0;
        for (var i = 0; i < ps.Length; i++)
        {
            r += (ps[i]) * ms[i];
            m += ms[i];
        }

        return r / m;
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
    void ExtractRotation(ref float3x3 a, ref quaternion r, int maxIterations = 9)
    {

        
        for (var i = 0; i < maxIterations; i++)
        {
            var rmat = new float3x3(r);
            var omega = (math.cross(rmat[0], a[0]) + math.cross(rmat[1], a[1]) + math.cross(rmat[2], a[2])) *
                        (1f / math.abs(math.dot(rmat[0], a[0]) + math.dot(rmat[1], a[1]) + math.dot(rmat[2], a[2]) + 0.000000001f));
            var w = math.length(omega);
            if (w < 0.00001)
            {
                break;
            }
            r = math.mul(quaternion.AxisAngle(omega / w, w), r);
            r = math.normalize(r);
        }
    }
    float3x3 ExtractRotation(ref float3x3 a)
    {
        var s = (a * math.transpose(a)).Sqrt();
        var sinv = math.inverse(s);
        return a * sinv;
    }

    

}


public static class VectorExtensions
{
    public static Vector3 ToWorld(this Vector3 localPt, Transform t) => t.TransformPoint(localPt);
    public static bool LessThanEq(this Vector3 a, Vector3 b) => a.x <= b.x && a.y <= b.y && a.z <= b.z;
    public static bool GreaterThanEq(this Vector3 a, Vector3 b) => a.x >= b.x && a.y >= b.y && a.z >= b.z;
    
    public static void ToMatrix(this quaternion q, ref float3x3 m) {
        m[0] = math.mul(q , new float3(1,0,0));
        m[1] = math.mul(q , new float3(0,1,0));
        m[2] = math.mul(q , new float3(0,0,1));
    }

    /// <summary>Returns the float3x3 matrix result of a matrix multiplication between a float3x2 matrix and a float2x3 matrix.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float3x3 MulTranspose(this float3 a, float3 b)
    {
        return math.float3x3(a * b.x, a * b.y, a * b.z);
    }



}

