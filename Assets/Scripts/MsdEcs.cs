using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Mathematics.Extensions;
using Unity.Mathematics;
using UnityEngine;

// ReSharper disable InconsistentNaming - Some Variables are named according to their name in [Muller05]'s paper

/// <summary>
/// This component is an implementation of the 2005 paper
///
/// [Muller05]
/// Muller, Matthias, et al. "Meshless deformations based on shape matching."
/// ACM transactions on graphics (TOG). Vol. 24. No. 3. ACM, 2005.
/// https://www.cs.drexel.edu/~david/Classes/Papers/MeshlessDeformations_SIG05.pdf
///
/// It also uses a computational approach to find rotations described in:
/// [Muller16]
/// Muller, Matthias, et al. "A robust method to extract the rotational part of deformations."
/// Proceedings of the 9th International Conference on Motion in Games. ACM, 2016
/// https://animation.rwth-aachen.de/media/papers/2016-MIG-StableRotation.pdf
///  
/// </summary>
[RequireComponent(typeof(MeshFilter))]
public class MsdEcs : MonoBehaviour
{
    [Range(0, 1), Tooltip(
         "This determines the acceleration with which the object tries to deform back into its original shape. " +
         "A value of 1 results in rigid-like behaviour.")]
    public float Alpha = 1;

    [Range(0, 1), Tooltip(
         "Interpolation between 0: rotation only or 1: shearing/bending. Works only on Linear and Quadratic Modes ")]
    public float Beta = 0;


    [Tooltip("Dampening of velocity. This is applied to all particle rigidbodies")]
    public float Drag = 0;
    private float oldDrag;

    [Tooltip("The matching mode determines how the shape can deform. All modes allow small deformations, but this controls changes in the overall frame of the shape." +
             "Rigid allows only rotations. Linear allows only shears and stretches. Quadratic allows bending. ")]
    public MatchingModes MatchingMode = MatchingModes.Quadratic;

    [Tooltip("If true, this transform will update it's position and rotation to the center of mass of the particles. A bit slow.")]
    public bool ShouldUpdatePivot = true;

    // Object properties
    private MeshFilter meshFilter;
    private Quaternion initialRotation;
    private Transform particleParent;

    // collections to avoid re-allocating. Variables are named according to their name in [Muller05]'s paper<
    private float3[] q;
    private float3[] x;
    private float[] m;
    private float3 x0cm;
    private float3x3 Aqq;
    private float3x3 T;
    //Quadratic terms
    private float9[] q_tilde;
    private float9x9 Aqq_tilde;
    private float3x9 T_tilde;
    private float3x9 R_tilde;

    // Rendering and physic entities;
    private Particle[] particles;
    private Vector3[] vertices;

    // Seed for ExtractRotation. We start with the identity quaternion because the particles' original position (q)
    // calculated using world-aligned axes. See [Muller2016]
    // Rq stands for "R quaternion", implying this is the quaternion version of R.
    private quaternion Rq = quaternion.identity;


    public void Awake()
    {
        initialRotation = transform.rotation;

        meshFilter = GetComponent<MeshFilter>();
        vertices = meshFilter.mesh.vertices;
        particles = CreateParticlesAtVertices();

        // Initial allocations
        x = new float3[particles.Length];
        m = new float[particles.Length];
        q = new float3[particles.Length];
        q_tilde = new float9[particles.Length];
        Aqq = float3x3.zero;
        Aqq_tilde = float9x9.zero;

        GetPositionsAndMasses(particles, x, m, out x0cm);

        // We calculate the quadratic terms and Aqq matrices regardless of the mode in case it gets switched at runtime.
        for (int i = 0; i < x.Length; i++)
        {
            q[i] = x[i] - x0cm;
            q_tilde[i] = GetQuadratic(q[i]);
            Aqq += m[i] * mathExt.mulT(q[i], q[i]);
            Aqq_tilde += m[i] * mathExt.mulT(q_tilde[i], q_tilde[i]);
        }

        Aqq = math.inverse(Aqq);
        Aqq_tilde = mathExt.inverse(Aqq_tilde);
    }



    // The contents of this method are outlined in [Muller05]
    public void FixedUpdate()
    {
        var mode = MatchingMode;

        GetPositionsAndMasses(particles, x, m, out var xcm);

        float3x3 Apq = float3x3.zero;
        float3x9 Apq_tilde = float3x9.zero;

        for (int i = 0; i < x.Length; i++)
        {
            var pi = x[i] - xcm;
            Apq += m[i] * mathExt.mulT(pi, q[i]);

            if (MatchingMode == MatchingModes.Quadratic)
                Apq_tilde += m[i] * mathExt.mulT(pi, q_tilde[i]);
        }


        // This part is different. It doesn't use polar decomposition. It instead uses the method outlined in [Muller16]
        ExtractRotation(ref Apq, ref Rq);
        var R = new float3x3(Rq);

        if (ShouldUpdatePivot) UpdatePivot(xcm, Rq);

        switch (mode)
        {
            case MatchingModes.Linear:
            {
                var A = math.mul(Apq, Aqq);
            
                // Divide by cube root of determinant to preserve volume
                var det = math.determinant(A);
                var cbrtDet = math.pow(math.abs(det), 1 / 3.0f);
                var div = det < 0 ? -cbrtDet : cbrtDet;
                A = A / div;

                T = Beta * A + (1 - Beta) * R;
                break;
            }
            case MatchingModes.Quadratic:
            {
                R_tilde.c0 = R.c0;
                R_tilde.c1 = R.c1;
                R_tilde.c2 = R.c2;

                var A_tilde = mathExt.mul(Apq_tilde, Aqq_tilde);

                // Todo: Can we preserve volume on a 3x9 transformation?

                T_tilde = Beta * A_tilde + (1 - Beta) * R_tilde;
                break;
            }
        }

        for (int i = 0; i < x.Length; i++)
        {
            float3 gi;
            switch (mode)
            {
                case MatchingModes.Rigid:
                    gi = math.mul(R, q[i]) + xcm; 
                    break;
                case MatchingModes.Linear: 
                    gi = math.mul(T, q[i]) + xcm; 
                    break;
                case MatchingModes.Quadratic: 
                    gi = mathExt.mul(T_tilde, q_tilde[i]) + xcm; 
                    break;
                default: 
                    throw new ArgumentOutOfRangeException();
            }

            var accel = (Alpha / Time.fixedDeltaTime) * (gi - x[i]);
            particles[i].Rigidbody.AddForce(accel, ForceMode.VelocityChange);
        }
    }

    private void UpdatePivot(in float3 xcm, in quaternion rotDif)
    {
        var t = transform;
        particleParent.SetParent(null, true);
        t.position = xcm;
        t.rotation = initialRotation * rotDif;
        particleParent.SetParent(t, true);
    }

    // [Muller16]
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float9 GetQuadratic(float3 qi) => new float9(
        qi.x, qi.y, qi.z,
        qi.x * qi.x, qi.y * qi.y, qi.z * qi.z,
        qi.x * qi.y, qi.y * qi.z, qi.z * qi.x);


    private void Update()
    {
        foreach (var particle in particles)
        {
            for (var i = 0; i < particle.Indexes.Length; i++)
            {
                var index = particle.Indexes[i];
                Vector3 offset = particle.Offsets[i];
                vertices[index] = transform.InverseTransformPoint(particle.Rigidbody.position) + offset;
            }
        }
        meshFilter.mesh.vertices = vertices;
        meshFilter.mesh.RecalculateNormals();
        meshFilter.mesh.RecalculateBounds();

        if (Drag != oldDrag)
        {
            oldDrag = Drag;
            foreach (var body in particles)
            {
                body.Rigidbody.drag = Drag;
            }
        }
    }

    private void GetPositionsAndMasses(Particle[] rs, float3[] ps, float[] ms, out float3 centerOfMass)
    {

        float3 weightedSum = float3.zero;
        float totalMass = 0;

        var dt = Time.fixedDeltaTime;
        for (int i = 0; i < rs.Length; i++)
        {
            var position = rs[i].Rigidbody.position;
            ps[i] = position + rs[i].Rigidbody.velocity * dt;
            ms[i] = rs[i].Rigidbody.mass;

            weightedSum += ps[i] * ms[i];
            totalMass += ms[i];
        }

        centerOfMass = weightedSum / totalMass;
    }

    private Particle[] CreateParticlesAtVertices()
    {

        var result = new Particle[vertices.Length];
        var colliders = new List<Collider>();

        particleParent = new GameObject("Particles").transform;
        particleParent.SetParent(transform, false);

        for (var i = 0; i < result.Length; i++)
        {
            var rb = MakeParticle(vertices[i], colliders);
            result[i].Rigidbody = rb;
            result[i].Indexes = new[] { i };
            result[i].Offsets = new[] { float3.zero };
        }

        return result;
    }

    private Rigidbody MakeParticle(Vector3 localPosition, List<Collider> colliders)
    {
        var o = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        o.name = "Particle";
        o.transform.SetParent(particleParent, false);
        o.transform.localPosition = localPosition;
        var c = o.GetComponent<SphereCollider>();
        foreach (var cc in colliders)
        {
            Physics.IgnoreCollision(c, cc, true);
        }

        colliders.Add(c);
        //c.radius = 0.1f;
        o.transform.localScale = Vector3.one * 0.1f;
        var rb = o.AddComponent<Rigidbody>();
        rb.freezeRotation = true;
        rb.drag = Drag;
        rb.constraints = RigidbodyConstraints.FreezeRotation;
        return rb;
    }


    // ReSharper disable once UnusedMember.Local - Useful for debugging.
    void DrawPt(Vector3 gi, Color? color = null)
    {
        var c = color ?? Color.white;
        Debug.DrawLine(gi, gi + new Vector3(0, 0, .1f), c);
        Debug.DrawLine(gi, gi + new Vector3(0, .1f, 0), c);
        Debug.DrawLine(gi, gi + new Vector3(.1f, 0, 0), c);

    }
}

public enum MatchingModes
{
    Rigid,
    Linear,
    Quadratic
}

public struct Particle
{
    public int[] Indexes;
    public float3[] Offsets;
    public Rigidbody Rigidbody;
}

