using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;

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
    IEnumerable<Matrix<float>> p;
    Vector3[] f;
    Vector<float> x_0_cm;
    Matrix<float> A_qq;
    [Range(0.01f, 1)] public float alpha = 1;
    [Range(0f, 0.49f)] public float beta = 1;
    [Range(0.01f,1)]public float velocityDamp = 1;
    [Range(0.01f,1)]public float bounceVelDamp = 1;

    readonly Dictionary<int,Vector3> targets = new Dictionary<int, Vector3>();
    public Vector3 defaultForce;
    public LayerMask collidesWith;
    public float pullForce = 1;
    MeshCollider meshCollider;
    Vector<float>[] q_hat;
    Matrix<float> A_hat_qq;
    public bool quadratic;
    Matrix<float> A_pq;
    Matrix<float> A_hat_pq;
    Matrix<float>[] q_rm;
    Matrix<float>[] q_hat_rm;

    // Use this for initialization
    void Start()
    {
        var meshFilter = GetComponent<MeshFilter>();
        meshCollider = GetComponent<MeshCollider>();
        partSys = GetComponent<ParticleSystem>();
        var main = partSys.main;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        
        startingMesh = meshFilter.mesh;
        currentMesh = Instantiate(startingMesh);
        
        //remove scaling from mesh;
        currentMesh.vertices = startingMesh.vertices.Select(v => v.PointwiseMult(transform.localScale)).ToArray();
        //remove children to keep from being affected by rescaling, then re-add
        var children = Enumerable.Range(0, transform.childCount).Select(i => transform.GetChild(i)).ToArray();
        transform.DetachChildren();
        transform.localScale = Vector3.one;
        foreach (var child in children) child.parent = transform;

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
        f = Enumerable.Repeat(defaultForce, particles.Length).ToArray();
        x_0_cm = x_0.Mean(m);
        q = x_0.Select(x0_i=>x0_i - x_0_cm).ToArray();
        A_qq = q.Zip(m, (q_i, m_i) => m_i * q_i.ToColumnMatrix() * q_i.ToRowMatrix()).Aggregate((sum,mat)=>sum+mat).Inverse();


        q_hat = q.Select(q_i => Vector<float>.Build.Dense(new[] { q_i[0], q_i[1], q_i[2], q_i[0] * q_i[0], q_i[1] * q_i[1], q_i[2] * q_i[2], q_i[0] * q_i[1], q_i[1] * q_i[2], q_i[2] * q_i[0] })).ToArray();
        q_rm = q.Select((q_i, i) => q_i.ToRowMatrix() * m[i]).ToArray();
        q_hat_rm = q_hat.Select((q_i, i) => q_i.ToRowMatrix() * m[i]).ToArray();

        var A_hat_qq_list = q_hat.Zip(m, (q_hat_i, m_i) => m_i * q_hat_i.ToColumnMatrix() * q_hat_i.ToRowMatrix()).ToArray();
        var A_hat_qq_sum = A_hat_qq_list.Aggregate((sum, mat) => sum + mat);
        //Todo: why is this a pseudoinverse?
        A_hat_qq = A_hat_qq_sum.PseudoInverse();

        A_pq = Matrix<float>.Build.Dense(3, 3, 0);
        A_hat_pq = Matrix<float>.Build.Dense(3, 9, 0);
    }

    void Update()
    {

        partSys.SetParticles(particles, particles.Length);

    }

    void FixedUpdate()
    {
        for (int i = 0; i < particles.Length; i++)  x[i].SetValue(particles[i].position);

        var x_cm = x.Mean(m);
        

        
        A_pq.Clear();
        foreach (var mat in x.Zip(q_rm, (x_i, q_rm_i) => (x_i - x_cm).ToColumnMatrix() * q_rm_i))
        {
            A_pq.Add(mat, A_pq);
        }
        if(quadratic)
        {
            A_hat_pq.Clear();
            foreach (var mat in x.Zip(q_hat_rm, (x_i, q_hat_rm_i) => (x_i - x_cm).ToColumnMatrix() * q_hat_rm_i))
            {
                A_hat_pq.Add(mat, A_hat_pq);
            }
        } 

        var S = (A_pq.Transpose() * A_pq).Sqrt();
        var R = A_pq * S.Inverse();

        Vector3[] g;
        if (!quadratic)
        {
            var A = A_pq * A_qq;
            var detA = A.Determinant();
            A /= Mathf.Pow(detA, 1 / 3f);
            var T = beta * A + (1 - beta) * R;
            g = q.Select(q_i => (T * q_i) + x_cm).Select(v => v.ToUnity()).ToArray();
        }
        else
        {
            var A_hat = A_hat_pq * A_hat_qq;
            var R_hat = Matrix<float>.Build.Dense(3, 9, (i, j) => j < 3 ? R[i, j] : 0);
            var T_hat = beta * A_hat + (1 - beta) * R_hat;
            g = q_hat.Select(q_hat_i => (T_hat * q_hat_i) + x_cm).Select(v => v.ToUnity()).ToArray();

        }

        var h = Time.fixedDeltaTime;

        foreach (var i in targets.Keys.ToList())
        {
            f[i] += pullForce * (targets[i]-g[i])/h;
            targets.Remove(i);
        }


        for (int i = 0; i < particles.Length; i++)
        {
            var start = particles[i].velocity;
            var middle = alpha * ((g[i] - particles[i].position) / h);
            var end = h * f[i] / m[i];
            f[i] = defaultForce;
            
            particles[i].velocity = start + middle + end;
            if (Vector3.Dot(particles[i].velocity, start) < 0) particles[i].velocity *= bounceVelDamp;
            particles[i].velocity *= velocityDamp;
            
            var hitInfos = Physics.RaycastAll(new Ray(particles[i].position - particles[i].velocity*h, particles[i].velocity.normalized),
                particles[i].velocity.magnitude*h*2, collidesWith ).Where(hit=>hit.transform!=this.transform);

            bool didhit = hitInfos.Any();

            particles[i].position = didhit? hitInfos.First().point : particles[i].position + particles[i].velocity * h;
            particles[i].velocity = didhit? Vector3.zero : particles[i].velocity;
        }

        //update unity pivot
        transform.position = x_cm.ToUnity();
        var targetQuat = R.ToUnity().ExtractRotation();
        transform.rotation = Quaternion.Slerp(transform.rotation, targetQuat, alpha);

        currentMesh.SetVertices(particles.Select(part => transform.InverseTransformPoint(part.position)).ToList());
        currentMesh.RecalculateNormals();
        meshCollider.sharedMesh = currentMesh;
    }


    public Vector3[] WorldVertices
    {
        get { return particles.Select(part => part.position).ToArray(); }
    }

    public void SetWorldVertexPosition(int index, Vector3 position, bool resetVelocity = false)
    {
        //particles[index].position = position;
        targets[index] = position;
        if (resetVelocity) particles[index].velocity = Vector3.zero;
 
    }

    public void SetWorldVertexVelocity(int index, Vector3 velocity)
    {
        particles[index].velocity = velocity;
    }
}