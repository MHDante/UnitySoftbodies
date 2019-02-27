using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices.ComTypes;
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
    [Range(0, 1), Tooltip(
         "This determines the acceleration with which the object tries to deform back into its original shape. " +
         "A value of 1 results in rigid-like behaviour.")]
    public float Alpha = 1;

    [Range(0, 1), Tooltip(
         "Interpolation between 0: rotation only or 1: shearing/bending. Works only on Linear and Quadratic Modes ")]
    public float Beta = 0;


    [Min(0f), Tooltip("Dampening of velocity. This is applied to all particle rigidbodies")]
    public float Drag = 0;
    private float oldDrag;

    [Tooltip("The matching mode determines how the shape can deform. All modes allow small deformations, but this controls changes in the overall frame of the shape." +
             "Rigid allows only rotations. Linear allows only shears and stretches. Quadratic allows bending. ")]
    public MatchingModes MatchingMode = MatchingModes.Quadratic;

    [Tooltip("If true, this transform will update it's position and rotation to the center of mass of the particles. A bit slow.")]
    public bool ShouldUpdatePivot = true;

    [Min(0.001f)]
    public float ParticleDistance = .1f;
    [Min(0.001f)]
    public float ParticleSize = .2f;
    [Min(0.001f)]
    public float ParticleMass = 0.008f;
    [Range(0, 1)]
    public float ParticleOverlap = 1;

    // Object properties
    private MeshFilter meshFilter;
    private Quaternion initialRotation;
    private Transform particleParent;
    private Material material;

    // collections to avoid re-allocating. Variables are named according to their name in [Muller05]'s paper<
    private float3[] q;
    private float3[] x;
    private float[] m;
    private float3 x0cm;
    private float3 xcm;
    private float3x3 Aqq;
    private float3x3 R;
    private float3x3 T;
    //Quadratic terms
    private float9[] q_tilde;
    private float9x9 Aqq_tilde;
    private float3x9 T_tilde;
    private float3x9 R_tilde;

    // Rendering and physics entities;
    private Rigidbody[] particles;
    //private List<int>[] vertexToParticleMap;
    //private List<Vector3>[] vertexOffsets;
    private Vector3[] vertices;
    private float3[] worldVertices;
    private float9[] worldVertices_tilde;

    // Seed for ExtractRotation. We start with the identity quaternion because the particles' original position (q)
    // calculated using world-aligned axes. See [Muller2016]
    // Rq stands for "R quaternion", implying this is the quaternion version of R.
    private quaternion Rq = quaternion.identity;
    private Vector3[] positions;

    private static readonly int SHADER_Mode = Shader.PropertyToID("_Mode");
    private static readonly int SHADER_T_Tilde = Shader.PropertyToID("_T");
    private static readonly int SHADER_T = Shader.PropertyToID("_T");
    private static readonly int SHADER_OriginalMatrix = Shader.PropertyToID("_OriginalMatrix");
    private static readonly int SHADER_x0cm = Shader.PropertyToID("_x0cm");
    private static readonly int SHADER_xcm = Shader.PropertyToID("_xcm");
    private static int counter;

    public void Awake()
    {
        initialRotation = transform.rotation;

        meshFilter = GetComponent<MeshFilter>();
        material = GetComponent<MeshRenderer>().material;
        vertices = meshFilter.mesh.vertices;

        worldVertices = new float3[vertices.Length];
        worldVertices_tilde = new float9[vertices.Length];
        //worldVertices_tilde = new float9[vertices.Length];
        //vertexToParticleMap = new List<int>[vertices.Length];
        //vertexOffsets = new List<Vector3>[vertices.Length];

        particles = CreateParticleLattice();

        positions = new Vector3[particles.Length];
        // Initial allocations
        x = new float3[particles.Length];
        m = new float[particles.Length];
        q = new float3[particles.Length];
        q_tilde = new float9[particles.Length];
        Aqq = float3x3.zero;
        Aqq_tilde = float9x9.zero;

        GetPositionsAndMasses(particles, x, m, out x0cm);

        material.SetVector(SHADER_x0cm, new Vector4(x0cm.x, x0cm.y, x0cm.z, 1));
        material.SetMatrix(SHADER_OriginalMatrix, transform.localToWorldMatrix);

        for (int i = 0; i < vertices.Length; i++)
        {
            worldVertices[i] = (float3)transform.TransformPoint(vertices[i]) - x0cm;
            worldVertices_tilde[i] = GetQuadratic(worldVertices[i]);
        }

        float3x3 temp = float3x3.zero;
        // We calculate the quadratic terms and Aqq matrices regardless of the mode in case it gets switched at runtime.
        for (int i = 0; i < x.Length; i++)
        {
            q[i] = x[i] - x0cm;
            q_tilde[i] = GetQuadratic(q[i]);
            mathExt.mulT(m[i] * q[i], q[i], ref temp);
            Aqq += temp;
            Aqq_tilde +=  mathExt.mulT(m[i] *q_tilde[i], q_tilde[i]);
        }

        Aqq = math.inverse(Aqq);
        Aqq_tilde = mathExt.inverse(Aqq_tilde);
    }



    // The contents of this method are outlined in [Muller05]
    [BurstCompile]
    public void FixedUpdate()
    {
        GetPositionsAndMasses(particles, x, m, out xcm);
        material.SetInt(SHADER_Mode, (int)MatchingMode);
        material.SetVector(SHADER_xcm, new Vector4(xcm.x, xcm.y, xcm.z, 1));

        GetApqMatrix(out var Apq, out var Apq_tilde);

        // This part is different. It doesn't use polar decomposition. It instead uses the method outlined in [Muller16]
        ExtractRotation(ref Apq, ref Rq);

        R = new float3x3(Rq);

        if (ShouldUpdatePivot) UpdatePivot(xcm, Rq);

        GetTransformations(Apq, Apq_tilde);

        ApplyGoalPositions();
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
        Matrix4x4[] mat_tilde = new Matrix4x4[3];

        switch (MatchingMode)
        {
            case MatchingModes.Rigid:

                R.ToOldMatrix(ref mat);
                material.SetMatrix(SHADER_T, mat);
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

                    material.SetMatrix(SHADER_T, mat);
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
                    T_tilde.ToOldMatrix(mat_tilde);

                    material.SetMatrixArray("_T_tilde", mat_tilde);

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
            mathExt.PlusEquals(ref Apq, temp);

            if (MatchingMode == MatchingModes.Quadratic)
            {

               mathExt.mulT(m[i] * pi, q_tilde[i], ref temp2);
               mathExt.PlusEquals(ref Apq_tilde, temp2);
               ;
            }
        }
    }

    private void UpdatePivot(in float3 xcm, in quaternion rotDif)
    {
        var t = transform;
        particleParent.SetParent(null, true);
        t.position = xcm;
        //t.rotation = initialRotation * rotDif;
        particleParent.SetParent(t, true);
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


    private void Update()
    {

        //for (var i = 0; i < vertices.Length; i++)
        //{
        //    float3 vert;
        //    switch (MatchingMode)
        //    {
        //        case MatchingModes.Rigid:
        //            vert = math.mul(R, worldVertices[i]);
        //            break;
        //        case MatchingModes.Linear:
        //            vert = math.mul(T, worldVertices[i]);
        //            break;
        //        case MatchingModes.Quadratic:
        //            vert = mathExt.mul(T_tilde, worldVertices_tilde[i]);
        //            break;
        //        default:
        //            throw new ArgumentOutOfRangeException();
        //    }
        //    vert += xcm;
        //    vert = transform.InverseTransformPoint(vert);
        //    vertices[i] = vert;
        //}

        //meshFilter.mesh.vertices = vertices;
        ////meshFilter.mesh.RecalculateNormals();
        //meshFilter.mesh.RecalculateBounds();

        if (Drag != oldDrag)
        {
            oldDrag = Drag;
            foreach (var body in particles)
            {
                body.drag = Drag;
            }
        }
    }

    private void GetPositionsAndMasses(Rigidbody[] rs, float3[] ps, float[] ms, out float3 centerOfMass)
    {

        float3 weightedSum = float3.zero;
        float totalMass = 0;

        var dt = Time.fixedDeltaTime;
        for (int i = 0; i < rs.Length; i++)
        {
            var position = rs[i].position;
            ps[i] = position + rs[i].velocity * dt;
            ms[i] = rs[i].mass;

            weightedSum += ps[i] * ms[i];
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

        particleParent = new GameObject("Particles").transform;
        particleParent.SetParent(transform, false);

        var t = transform;
        var halfC = ParticleDistance / 2;
        var bounds = meshFilter.mesh.bounds;
        var min = bounds.min;
        var max = bounds.max + Vector3.one * halfC;
        var cells = new int3(math.ceil((max - min) / ParticleDistance));
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

        var halfextents = worldExtents * .5f * ParticleDistance;
        var diff = Vector3.one * ParticleDistance * (1 + ParticleOverlap);

        for (int i = 0; i < cells.x; i++)
        {
            for (int j = 0; j < cells.y; j++)
            {
                for (int k = 0; k < cells.z; k++)
                {
                    var center = new Vector3(i * ParticleDistance, j * ParticleDistance, k * ParticleDistance);
                    center += min;
                    Array.Clear(results, 0, results.Length);

                    var size = Physics.OverlapBoxNonAlloc(transform.TransformPoint(center), halfextents, results, t.rotation);
                    if (size >= results.Length)
                    {
                        results = new Collider[size];
                        Physics.OverlapBoxNonAlloc(transform.TransformPoint(center), halfextents, results, t.rotation);
                    }
                    if (!results.Contains(oldCollider)) continue;

                    //for (int l = 0; l < vertices.Length; l++)
                    //{
                    //    float3 offset = vertices[l] - center;
                    //    var maxOffset = new float3(halfC * (1+ParticleOverlap));
                    //    var absOffset = math.abs(offset);


                    //    if(math.any(absOffset > maxOffset))
                    //        continue;

                    //    vertexToParticleMap[l].Add(result.Count);
                    //    vertexOffsets[l].Add(offset);
                    //}
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

        particleParent = new GameObject("Particles").transform;
        particleParent.SetParent(transform, false);

        for (var i = 0; i < result.Length; i++)
        {
            var rb = MakeParticle(vertices[i], colliders);
            result[i] = rb;
        }

        return result;
    }

    private Rigidbody MakeParticle(Vector3 localPosition, List<Collider> colliders)
    {
        var o = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        o.GetComponent<MeshRenderer>().enabled = false;
        o.name = $"Particle {counter++}";
        o.transform.SetParent(particleParent, false);
        o.transform.localPosition = localPosition;
        var c = o.GetComponent<SphereCollider>();
        foreach (var cc in colliders)
        {
            Physics.IgnoreCollision(c, cc, true);
        }

        colliders.Add(c);
        //c.radius = 0.1f;
        o.transform.localScale = Vector3.one * ParticleSize;

        var rb = o.AddComponent<Rigidbody>();
        rb.freezeRotation = true;
        rb.drag = Drag;
        rb.useGravity = true;
        rb.mass = ParticleMass;
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
    Rigid = 0,
    Linear = 1,
    Quadratic = 2
}


