using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Mathematics;
using Mathematics.Extensions;

public class MsdEcs : MonoBehaviour
{
    [Range(0, 1)]
    public float Alpha = 1;

    [Range(0, 1)]
    public float Beta = 1;

    public float Drag;

    private Vector3[] vertices;
    private float9[] q_tilde;
    private float3[] q;
    private float3[] x;
    private float[] m;
    private MeshFilter meshFilter;
    private Rigidbody[] bodies;
    private float3 x0Cm;
    private quaternion rq = quaternion.identity;
    private float3x3 Aqq;
    private float9x9 Aqq_tilde;
    private Transform parent;

    public void Awake()
    {

        //var oldCollider = GetComponent<Collider>();
        meshFilter = GetComponent<MeshFilter>();

        vertices = meshFilter.mesh.vertices;
        bodies = new Rigidbody[vertices.Length];
        var colliders = new List<Collider>();

        parent = new GameObject("Particles").transform;
        parent.SetParent(transform, false);

        for (var i = 0; i < vertices.Length; i++)
        {
            var o = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            o.name = $"Particle {i}";
            o.transform.SetParent(parent, false);
            o.transform.localPosition = vertices[i];
            var c = o.GetComponent<SphereCollider>();
            foreach (var cc in colliders)
            {
                Physics.IgnoreCollision(c, cc, true);
            }
            colliders.Add(c);
            //c.radius = 0.1f;
            o.transform.localScale = Vector3.one * 0.1f;
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
        GetPositionsAndMasses(bodies, x, m);
        x0Cm = CenterOfMass(x, m);

        q = x.Select(x0i => x0i - x0Cm).ToArray();
        q_tilde = q.Select(qi => new float9(
            qi.x, qi.y, qi.z,
            qi.x * qi.x, qi.y * qi.y, qi.z * qi.z,
            qi.x * qi.y, qi.y * qi.z, qi.z * qi.x)
        ).ToArray();

        Aqq = float3x3.zero;
        Aqq_tilde = float9x9.zero;
        for (int i = 0; i < x.Length; i++)
        {
            var mass = bodies[i].mass;
            Aqq += mass * mathExt.mulT(q[i], q[i]);
            Aqq_tilde += mass * mathExt.mulT(q_tilde[i], q_tilde[i]);
        }

        Aqq = math.inverse(Aqq);
        Aqq_tilde = mathExt.inverse(Aqq_tilde);
    }

    public void FixedUpdate()
    {

        GetPositionsAndMasses(bodies, x, m);
        var xcm = CenterOfMass(x, m);

        //var t = transform;
        //parent.SetParent(null,true);
        //t.position = xcm;
        //parent.SetParent(t,true);

        float3x3 Apq = float3x3.zero;
        float3x9 Apq_tilde = float3x9.zero;

        for (int i = 0; i < x.Length; i++)
        {
            var mass = bodies[i].mass;
            var pi = x[i] - xcm;
            Apq += mass * mathExt.mulT(pi, q[i]);
            Apq_tilde += mass * mathExt.mulT(pi, q_tilde[i]);
        }

        ExtractRotation(ref Apq, ref rq);

        var R = new float3x3(rq);
        
        var R_tilde = float3x9.zero;
        R_tilde.c0 = R.c0;
        R_tilde.c1 = R.c1;
        R_tilde.c2 = R.c2;

        var A = math.mul(Apq, Aqq);
        PerserveVolume(ref A);

        var A_tilde = mathExt.mul(Apq_tilde, Aqq_tilde);

        var T = Beta * A + (1 - Beta) * R;
        var T_tilde = Beta * A_tilde + (1 - Beta) * R_tilde;

        DrawPt(xcm);
        for (int i = 0; i < x.Length; i++)
        {
            var rot1 = math.mul(T, q[i]);
            var rot2 = mathExt.mul(T_tilde,  q_tilde[i]);
            var gi = rot1 + xcm;
            var diff = gi - x[i];
            DrawPt(x[i], Color.red);
            DrawPt(gi, Color.green);


            var accel = Alpha / Time.fixedDeltaTime * diff;
            bodies[i].AddForce(accel, ForceMode.VelocityChange);
        }
    }

    void DrawPt(Vector3 gi, Color? color = null)
    {
        ;
        var c = color ?? Color.white;
        Debug.DrawLine(gi, gi + new Vector3(0, 0, .1f), c);
        Debug.DrawLine(gi, gi + new Vector3(0, .1f, 0), c);
        Debug.DrawLine(gi, gi + new Vector3(.1f, 0, 0), c);

    }
    private void Update()
    {
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = this.transform.InverseTransformPoint(bodies[i].position);
        }
        meshFilter.mesh.vertices = vertices;
        meshFilter.mesh.RecalculateNormals();
    }



    private void GetPositionsAndMasses(Rigidbody[] rs, float3[] ps, float[] ms)
    {
        var dt = Time.fixedDeltaTime;
        for (int i = 0; i < rs.Length; i++)
        {
            var position = rs[i].position;
            ps[i] = position + rs[i].velocity * dt;
            ms[i] = rs[i].mass;
        }
    }

    public static float3 CenterOfMass(float3[] ps, float[] ms)
    {
        float3 r = float3.zero;
        float m = 0;
        for (var i = 0; i < ps.Length; i++)
        {
            r += ps[i] * ms[i];
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
            if (w < 0.00001) break;
            r = math.mul(quaternion.AxisAngle(omega / w, w), r);
            r = math.normalize(r);
        }
    }


    public void PerserveVolume(ref float3x3 a)
    {
        var det = math.determinant(a);
        var cbrtDet = math.pow(math.abs(det), 1 / 3.0);
        var div = det < 0 ? -cbrtDet : cbrtDet;
        a = a / (float)div;
    }


}




