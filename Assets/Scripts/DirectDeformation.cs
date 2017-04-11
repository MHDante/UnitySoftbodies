using UnityEngine;
using System.Linq;

public class DirectDeformation : MonoBehaviour, IDeformer
{
    Mesh startingMesh;
    Mesh currentMesh;
    ParticleSystem partSys;
    ParticleSystem.Particle[] particles;


    // Use this for initialization
	void Start ()
	{
        var meshFilter = GetComponent<MeshFilter>();
	    var meshCollider = GetComponent<MeshCollider>();
	    partSys = GetComponent<ParticleSystem>();
	    startingMesh = meshFilter.mesh;
	    currentMesh = Instantiate(startingMesh);
        meshFilter.mesh = currentMesh;
	    meshCollider.sharedMesh = currentMesh;
        
	    particles = currentMesh.vertices.Select(
            v => new ParticleSystem.Particle
            {
                position = v,
                startColor = Color.blue,
                startSize = 0.1f,
                startLifetime = 100000,
                remainingLifetime = 100000,
            }
            ).ToArray();
	}

    void Update()
    {

        partSys.SetParticles(particles, particles.Length);
        
    }

    void FixedUpdate()
    {
        currentMesh.SetVertices(particles.Select(p => p.position).ToList());
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