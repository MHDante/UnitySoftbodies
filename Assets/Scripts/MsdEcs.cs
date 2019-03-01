using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Mathematics.Extensions;
using Unity.Burst;
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
    private static int counter;

    [Range(0, 1), Tooltip(
         "This determines the acceleration with which the object tries to deform back into its original shape. " +
         "A value of 1 results in rigid-like behaviour.")]
    public float Alpha = 1;

    [Range(0, 1), Tooltip(
         "Interpolation between 0: rotation only or 1: shearing/bending. Works only on Linear and Quadratic Modes.")]
    public float Beta = 0;


    [Tooltip("The matching mode determines how the shape can deform. All modes allow small deformations, but this controls changes in the " +
             "overall frame of the shape. Rigid allows only rotations. Linear allows only shears and stretches. Quadratic allows bending. ")]
    public MatchingModes MatchingMode = MatchingModes.Quadratic;

    public ParticleOptions ParticleOptions = new ParticleOptions()
    {
        Distance = .2f,
        Size = .2f,
        Mass = 0.008f,
        Shape = PrimitiveType.Sphere,
        Mode = ParticleModes.SpaceFillingLattice
    };

    // Object properties
    [SerializeField, HideInInspector] private MeshFilter meshFilter;

    // collections and variables are kept as fields to avoid re-allocating.
    // Variables are named according to their name in [Muller05]'s paper

    [SerializeField, HideInInspector] private float3[] q;           // Initial relative (from center of mass) locations of particles
    [SerializeField, HideInInspector] private float9[] q_tilde;     // Quadratic q
    [SerializeField, HideInInspector] private float3[] x;           // Current world position of particles
    [SerializeField, HideInInspector] private float[] m;            // Current Masses of particles
    [SerializeField, HideInInspector] private float3 xcm;           // Current Center of Mass
    [SerializeField, HideInInspector] private float3x3 Aqq;         // Scaling part of A. Can be precomputed
    [SerializeField, HideInInspector] private float9x9 Aqq_tilde;   // Quadratic Aqq
    [SerializeField, HideInInspector] private float3x3 R;           // The rotation part of Apq
    [SerializeField, HideInInspector] private quaternion Rq;        // Rq stands for "R quaternion". See [Muller16]
    [SerializeField, HideInInspector] private float3x9 R_tilde;     // Quadratic R
    [SerializeField, HideInInspector] private float3x3 T;           // Interpolation between R and A. See [Muller05] Section 4.2
    [SerializeField, HideInInspector] private float3x9 T_tilde;     // Quadratic T

    // Particles are represented by tiny rigidbodies in the physics system.
    [SerializeField, HideInInspector] private Rigidbody[] particles;
    [SerializeField, HideInInspector] private Transform particleParent;

    // This is ultimately the data that we wish to modify.
    [SerializeField, HideInInspector] private float3[] origVertices; // The positions of the mesh vertices relative to the center of mass.      
    [SerializeField, HideInInspector] private float9[] origVertices_tilde;
    [SerializeField, HideInInspector] private float3[] origNormals;
    [SerializeField, HideInInspector] private float9[] origNormals_tilde;
    [SerializeField, HideInInspector] private float3[] origTangents;
    [SerializeField, HideInInspector] private float9[] origTangents_tilde;

    // We populate these arrays with transformed data and send it back to the mesh filter
    [SerializeField, HideInInspector] private Vector3[] vertices;
    [SerializeField, HideInInspector] private Vector3[] normals;
    [SerializeField, HideInInspector] private Vector4[] tangents;

    // These variables are used to match the pivot of this object to the particle's motion.
    [SerializeField, HideInInspector] private Quaternion initialRotation;
    [SerializeField, HideInInspector] private Quaternion prevRot;
    [SerializeField, HideInInspector] private Vector3 prevPos;

    public void Awake()
    {
        initialRotation = transform.rotation;
        prevPos = transform.position;
        prevRot = transform.rotation;
        meshFilter = GetComponent<MeshFilter>();
        var mesh = meshFilter.mesh;
        mesh.MarkDynamic();
        vertices = mesh.vertices;
        normals = mesh.normals;
        tangents = mesh.tangents;

        switch (ParticleOptions.Mode)
        {
            case ParticleModes.SpaceFillingLattice:
                particles = CreateParticleLattice();
                break;
            case ParticleModes.ParticlesAtVertices:
                particles = CreateParticlesAtVertices();
                break;
            default:
                throw new ArgumentOutOfRangeException();
        }

        // Initial allocations

        origVertices = new float3[vertices.Length];
        origVertices_tilde = new float9[vertices.Length];
        origNormals = new float3[vertices.Length];
        origNormals_tilde = new float9[vertices.Length];
        origTangents = new float3[vertices.Length];
        origTangents_tilde = new float9[vertices.Length];

        x = new float3[particles.Length];
        m = new float[particles.Length];
        q = new float3[particles.Length];
        q_tilde = new float9[particles.Length];

        //Seed for ExtractRotation. We start with the identity quaternion because the particles' original position (q) is calculated using
        // world-aligned axes. See [Muller2016]. 
        Rq = quaternion.identity;

        GetPositionsAndMasses(particles, x, m, out var x0cm);

        for (int i = 0; i < vertices.Length; i++)
        {
            origVertices[i] = (float3)transform.TransformPoint(vertices[i]) - x0cm;
            origVertices_tilde[i] = GetQuadratic(origVertices[i]);

            origNormals[i] = transform.TransformDirection(normals[i]);
            origNormals_tilde[i] = GetQuadratic(origNormals[i]);

            origTangents[i] = transform.TransformDirection(tangents[i]);
            origTangents_tilde[i] = GetQuadratic(origTangents[i]);
        }

        float3x3 temp = float3x3.zero;
        // We calculate the quadratic terms and Aqq matrices regardless of the mode in case it gets switched at runtime.
        for (int i = 0; i < x.Length; i++)
        {
            q[i] = x[i] - x0cm;
            q_tilde[i] = GetQuadratic(q[i]);
            mathExt.mulT(m[i] * q[i], q[i], ref temp);
            Aqq += temp;
            Aqq_tilde += mathExt.mulT(m[i] * q_tilde[i], q_tilde[i]);
        }

        Aqq = math.inverse(Aqq);
        Aqq_tilde = mathExt.inverse(Aqq_tilde);
    }

    // The contents of this method are outlined in [Muller05]
    [BurstCompile]
    public void FixedUpdate()
    {
        MatchPivot();
        GetPositionsAndMasses(particles, x, m, out xcm);

        GetApqMatrix(out var Apq, out var Apq_tilde);

        ExtractRotation(ref Apq, ref Rq);

        R = new float3x3(Rq);

        var t = transform;
        prevPos = t.position = xcm;
        prevRot = t.rotation = initialRotation * Rq;

        GetTransformations(Apq, Apq_tilde);

        ApplyGoalPositions();
    }

    [BurstCompile]
    private void Update()
    {
        // Update Mesh
        for (var i = 0; i < vertices.Length; i++)
        {
            float3 vert = float3.zero;
            float3 tan = float3.zero;
            float3 norm = float3.zero;
            switch (MatchingMode)
            {
                case MatchingModes.Rigid:
                    vert = math.mul(R, origVertices[i]);
                    norm = math.mul(R, origNormals[i]);
                    tan = math.mul(R, origTangents[i]);

                    break;
                case MatchingModes.Linear:
                    vert = math.mul(T, origVertices[i]);
                    norm = math.mul(T, origNormals[i]);
                    tan = math.mul(T, origTangents[i]);
                    break;
                case MatchingModes.Quadratic:
                    mathExt.mul(T_tilde, origVertices_tilde[i], ref vert);
                    mathExt.mul(T_tilde, origNormals_tilde[i], ref norm);
                    mathExt.mul(T_tilde, origTangents_tilde[i], ref tan);

                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            vert += xcm;
            vertices[i] = transform.InverseTransformPoint(vert);
            normals[i] = transform.InverseTransformDirection(norm);
            tan = transform.InverseTransformDirection(tan);
            tangents[i].x = tan.x;
            tangents[i].y = tan.y;
            tangents[i].z = tan.z;
        }

        meshFilter.mesh.vertices = vertices;
        meshFilter.mesh.normals = normals;
        meshFilter.mesh.tangents = tangents;
    }


    private void MatchPivot()
    {
        var t = transform;

        if (t.position != prevPos || t.rotation != prevRot)
        {
            GetPositionsAndMasses(particles, x, m, out xcm);

            foreach (var particle in particles)
            {
                var cm = (Vector3)xcm;
                var pos = particle.position;
                if (t.rotation != prevRot)
                {
                    var diff = t.rotation * Quaternion.Inverse(prevRot);
                    pos = diff * (pos - cm) + cm;
                }

                particle.position = pos + t.position - prevPos;
            }
        }
    }

    private void ApplyGoalPositions()
    {
        for (int i = 0; i < x.Length; i++)
        {
            float3 gi = float3.zero;
            switch (MatchingMode)
            {
                case MatchingModes.Rigid:
                    gi = math.mul(R, q[i]) + xcm;
                    break;
                case MatchingModes.Linear:
                    gi = math.mul(T, q[i]) + xcm;
                    break;
                case MatchingModes.Quadratic:
                    mathExt.mul(T_tilde, q_tilde[i], ref gi);
                    gi += xcm;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            var accel = (Alpha / Time.fixedDeltaTime) * (gi - x[i]);
            particles[i].AddForce(accel, ForceMode.VelocityChange);
        }
    }

    private void GetTransformations(in float3x3 Apq, in float3x9 Apq_tilde)
    {
        Matrix4x4 mat = Matrix4x4.zero;

        switch (MatchingMode)
        {
            case MatchingModes.Rigid:

                R.ToOldMatrix(ref mat);
                break;
            case MatchingModes.Linear:
                {
                    var A = math.mul(Apq, Aqq);

                    // Divide by cube root of determinant to preserve volume
                    var det = math.determinant(A);
                    var cbrtDet = math.pow(math.abs(det), 1 / 3.0f);
                    var div = det < 0 ? -cbrtDet : cbrtDet;
                    A = A / div;

                    T = Beta * A + (1 - Beta) * R;

                    T.ToOldMatrix(ref mat);
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
    }

    private void GetApqMatrix(out float3x3 Apq, out float3x9 Apq_tilde)
    {
        Apq = float3x3.zero;
        Apq_tilde = float3x9.zero;
        float3x3 temp = float3x3.zero;
        float3x9 temp2 = float3x9.zero;

        for (int i = 0; i < x.Length; i++)
        {
            var pi = x[i] - xcm;

            mathExt.mulT(m[i] * pi, q[i], ref temp);
            mathExt.plusEquals(ref Apq, temp);

            if (MatchingMode == MatchingModes.Quadratic)
            {
                mathExt.mulT(m[i] * pi, q_tilde[i], ref temp2);
                mathExt.plusEquals(ref Apq_tilde, temp2);
            }
        }
    }

    // [Muller16]
    static void ExtractRotation(ref float3x3 a, ref quaternion r, int maxIterations = 9)
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
    private static float9 GetQuadratic(float3 w) => new float9(
        w.x, w.y, w.z,
        w.x * w.x, w.y * w.y, w.z * w.z,
        w.x * w.y, w.y * w.z, w.z * w.x);


    private void GetPositionsAndMasses(Rigidbody[] rs, float3[] ps, float[] ms, out float3 centerOfMass)
    {
        float3 weightedSum = float3.zero;
        float totalMass = 0;

        var dt = Time.fixedDeltaTime;
        for (int i = 0; i < rs.Length; i++)
        {
            var position = rs[i].position;
            var vel = rs[i].velocity;
            ps[i].x = position.x + vel.x * dt;
            ps[i].y = position.y + vel.y * dt;
            ps[i].z = position.z + vel.z * dt;

            ms[i] = rs[i].mass;

            weightedSum.x += ps[i].x * ms[i];
            weightedSum.y += ps[i].y * ms[i];
            weightedSum.z += ps[i].z * ms[i];
            totalMass += ms[i];
        }

        centerOfMass = weightedSum / totalMass;
    }


    private Rigidbody[] CreateParticleLattice()
    {
        var oldCollider = GetComponent<Collider>();
        if (oldCollider == null)
        {
            var meshCollider = gameObject.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = meshFilter.sharedMesh;
            oldCollider = meshCollider;
        }
        var result = new List<Rigidbody>(100);
        var colliders = new List<Collider>(100);

        particleParent = new GameObject($"{name} Particles").transform;
        particleParent.localScale = transform.lossyScale;
        particleParent.position = transform.position;
        particleParent.rotation = transform.rotation;

        var t = transform;
        var dist = ParticleOptions.Distance;
        var halfC = dist / 2;
        var bounds = meshFilter.mesh.bounds;
        var min = bounds.min;
        var max = bounds.max + Vector3.one * halfC;
        var cells = new int3(math.ceil((max - min) / dist));
        var zero = t.TransformPoint(Vector3.zero);
        var right = t.TransformPoint(Vector3.right);
        var up = t.TransformPoint(Vector3.up);
        var forward = t.TransformPoint(Vector3.forward);

        var worldExtents = new Vector3(
            Vector3.Distance(zero, right),
            Vector3.Distance(zero, up),
            Vector3.Distance(zero, forward)
        );

        Collider[] results = new Collider[5];

        var halfextents = worldExtents * halfC;

        for (int i = 0; i < cells.x; i++)
        {
            for (int j = 0; j < cells.y; j++)
            {
                for (int k = 0; k < cells.z; k++)
                {
                    var center = new Vector3(i * dist, j * dist, k * dist);
                    center += min;
                    Array.Clear(results, 0, results.Length);

                    var size = Physics.OverlapBoxNonAlloc(transform.TransformPoint(center), halfextents, results, t.rotation);
                    if (size >= results.Length)
                    {
                        results = new Collider[size];
                        Physics.OverlapBoxNonAlloc(transform.TransformPoint(center), halfextents, results, t.rotation);
                    }
                    if (!results.Contains(oldCollider)) continue;

                    result.Add(MakeParticle(center, colliders));
                }
            }
        }

        Destroy(oldCollider);
        Destroy(GetComponent<Rigidbody>());
        return result.ToArray();
    }

    private Rigidbody[] CreateParticlesAtVertices()
    {
        var result = new Rigidbody[vertices.Length];
        var colliders = new List<Collider>();

        particleParent = new GameObject($"{name} Particles").transform;
        particleParent.localScale = transform.lossyScale;
        particleParent.position = transform.position;
        particleParent.rotation = transform.rotation;

        for (var i = 0; i < result.Length; i++)
        {
            var rb = MakeParticle(vertices[i], colliders);
            result[i] = rb;
        }

        return result;
    }

    private Rigidbody MakeParticle(Vector3 localPosition, List<Collider> colliders)
    {
        var o = GameObject.CreatePrimitive(ParticleOptions.Shape);
        o.GetComponent<MeshRenderer>().enabled = false;
        o.name = $"{name} Particle {counter++}";
        o.transform.SetParent(particleParent, false);
        o.transform.localPosition = localPosition;
        var c = o.GetComponent<Collider>();
        foreach (var cc in colliders)
        {
            Physics.IgnoreCollision(c, cc, true);
        }

        colliders.Add(c);
        o.transform.localScale = Vector3.one * ParticleOptions.Size;

        var rb = o.AddComponent<Rigidbody>();
        rb.freezeRotation = true;
        rb.drag = ParticleOptions.Drag;
        rb.useGravity = true;
        rb.mass = ParticleOptions.Mass;
        c.sharedMaterial = ParticleOptions.PhysicsMaterial;
        o.layer = gameObject.layer;
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
    Rigid = 0,
    Linear = 1,
    Quadratic = 2
}

[Serializable]
public struct ParticleOptions
{
    [Header("These options will have no effect during runtime. Right-Click for more options.")]
    [Min(0.001f)] public float Distance;
    [Min(0.001f)] public float Size;
    [Min(0.001f)] public float Mass;
    public PrimitiveType Shape;
    public PhysicMaterial PhysicsMaterial;
    public ParticleModes Mode;

    [Min(0f), Tooltip("Dampening of velocity. This is applied to all particle rigidbodies")]
    public float Drag;
}

public enum ParticleModes
{
    SpaceFillingLattice,
    ParticlesAtVertices,
}