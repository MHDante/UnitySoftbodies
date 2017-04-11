﻿using UnityEngine;
using System.Linq;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;

public class MeshlessShapeMatching : MonoBehaviour, IDeformer
{
    Mesh startingMesh;
    Mesh currentMesh;
    ParticleSystem partSys;
    ParticleSystem.Particle[] particles;


    Vector<float>[] x_0;
    Vector<float>[] x;
    float[] m;
    Vector<float>[] q;
    Vector<float>[] p;
    Vector3[] f;
    Vector<float> x_0_cm;
    Matrix<float> A_qq;
    [Range(0, 1)] public float alpha = 1;
    [Range(0,1)]public float velocityDamp = 1;

    public Vector3 defaultForce;
    // Use this for initialization
    void Start()
    {
        var meshFilter = GetComponent<MeshFilter>();
        var meshCollider = GetComponent<MeshCollider>();
        partSys = GetComponent<ParticleSystem>();
        var main = partSys.main;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        
        startingMesh = meshFilter.mesh;
        currentMesh = Instantiate(startingMesh);
        meshFilter.mesh = currentMesh;
        meshCollider.sharedMesh = currentMesh;

        particles = currentMesh.vertices.Select(
            v => new ParticleSystem.Particle
            {
                position = transform.TransformPoint(v),
                startColor = Color.blue,
                startSize = 0.1f,
                startLifetime = 100000,
                remainingLifetime = 100000,
            }
        ).ToArray();

        x_0 = particles.Select(part => part.position.ToNumerics()).ToArray();
        x = particles.Select(part => part.position.ToNumerics()).ToArray();
        m = Enumerable.Repeat(1f, currentMesh.vertexCount).ToArray();
        f = Enumerable.Repeat(defaultForce, currentMesh.vertexCount).ToArray();
        x_0_cm = x_0.Mean(m);
        q = x_0.Select(x0_i=>x0_i - x_0_cm).ToArray();
        A_qq = q.Zip(m, (q_i, m_i) => m_i * q_i.ToColumnMatrix() * q_i.ToRowMatrix()).Aggregate((sum,mat)=>sum+mat).Inverse();


    }

    void Update()
    {

        partSys.SetParticles(particles, particles.Length);

    }

    void FixedUpdate()
    {
        for (int i = 0; i < particles.Length; i++)  x[i].SetValue(particles[i].position);

        var x_cm = x.Mean(m);
        p = x.Select(x_i => x_i - x_cm).ToArray();
        var A_pq = Matrix<float>.Build.Dense(3, 3, 0);

        //Todo:could cache weighted q_i row matrices
        for (int i = 0; i < q.Length; i++)
        {
            A_pq += m[i] * p[i].ToColumnMatrix() * q[i].ToRowMatrix();
        }

        var S = (A_pq.Transpose() * A_pq).Sqrt();
        var R = A_pq * S.Inverse();

        var g = q.Select(q_i => (R * q_i) + x_cm).Select(v=>v.ToUnity()).ToArray();

        var h = Time.fixedDeltaTime;
        for (int i = 0; i < particles.Length; i++)
        {
            var start = particles[i].velocity;
            var middle = alpha * ((g[i] - particles[i].position) / h);
            var end = h * f[i] / m[i];
            f[i] = defaultForce;

            particles[i].velocity = start + middle + end;

            particles[i].position += particles[i].velocity * h;
            particles[i].velocity *= velocityDamp;
        }
        
        currentMesh.SetVertices(particles.Select(p => transform.InverseTransformPoint(p.position)).ToList());
        currentMesh.RecalculateNormals();
    }


    public Vector3[] WorldVertices
    {
        get { return particles.Select(p => p.position).ToArray(); }
    }

    public void SetWorldVertexPosition(int index, Vector3 position, bool resetVelocity = true)
    {
        particles[index].position = position;
        if (resetVelocity) particles[index].velocity = Vector3.zero;
    }

    public void SetWorldVertexVelocity(int index, Vector3 velocity)
    {
        particles[index].velocity = velocity;
    }
}