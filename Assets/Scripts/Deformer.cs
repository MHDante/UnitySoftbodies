using UnityEngine;

public interface IDeformer
{
    Vector3[] WorldVertices { get; }
    void SetWorldVertexPosition(int index, Vector3 position, bool resetVelocity = true);
    void SetWorldVertexVelocity(int index, Vector3 velocity);
    


}