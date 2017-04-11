//using System;
//using UnityEngine;
//using System.Collections;
//using System.Collections.Generic;
//using System.Linq;
//using System.Reflection.Emit;

//using MathNet.Numerics.LinearAlgebra;
//using MathNet.Numerics.LinearAlgebra.Single;


//public class MeshlessShapeMatching : IDeformer
//{


//    static MatrixBuilder<float> _M = Matrix<float>.Build;
//    static VectorBuilder<float> _V = Vector<float>.Build;


//    MeshFilter meshFilter;
//    Mesh startingMesh;
//    Mesh currentMesh;

//    private Vector<float>[] q;
//    private Vector<float>[] p;
//    private float[] m;
//    private Vector3[] v;
//    [Range(0,1)]
//    public float alpha;
//    public float forceMult;

//    private Vector3[] f_ext;

//    // Use this for initialization
//	void Start ()
//	{
//      meshFilter = GetComponent<MeshFilter>();
//	    startingMesh = meshFilter.mesh;
//	    currentMesh = Instantiate(startingMesh);
//	    meshFilter.mesh = currentMesh;

//        q = new Vector<float>[startingMesh.vertexCount];
//        p = new Vector<float>[startingMesh.vertexCount];
//        m = new float[startingMesh.vertexCount];
//        v = new Vector3[startingMesh.vertexCount];
//        f_ext= new Vector3[startingMesh.vertexCount];
//        for (int i = 0; i < m.Length; i++) m[i] = 1;
//	    for (int i = 0; i < v.Length; i++) v[i] = Vector3.zero;


//        q = startingMesh.vertices.Select(v=>_V.Dense(new[] { v.x,v.y,v.z})).ToArray();
//        //var q_hat = q.Select(q_i=>V.Dense(new[] {q_i.x, q_i.y, q_i.z, q_i.x* q_i.x, q_i.y* q_i.y, q_i.z* q_i.z, q_i.x*q_i.y, q_i.y*q_i.z, q_i.z*q_i.x}));
//	    //var A_hat_qq = q_hat.Select((q_hat_i, i) => q_hat_i.ToRowMatrix().LeftMultiply(q_hat_i).Multiply(m[i]));
//        var A_qq_list = q.Select((q_i, i) => {
//                        return q_i.ToColumnMatrix().Multiply(q_i.ToRowMatrix()).Multiply(m[i]);
//        }
//            ).ToArray();
//	    var A_qq = A_qq_list.Aggregate((m1, m2) =>
//        m1.Add(m2)
//        );



//    }

//	// Update is called once per frame
//	void FixedUpdate ()
//	{

//	    var x = currentMesh.vertices;
//        p = x.Select(v => _V.Dense(new[] { v.x, v.y, v.z })).ToArray();
//        var A_pq_list = p.Select((p_i, i) => p_i.ToColumnMatrix().Multiply(q[i].ToRowMatrix()).Multiply(m[i]));
//        var A_pq = A_pq_list.Aggregate((m1, m2) => m1.Add(m2));

//	    var svd = A_pq.Svd();
//	    var V = svd.VT.Transpose();
//	    var sigma = _M.DiagonalOfDiagonalVector(svd.S);
//	    var S = V.Multiply(sigma).Multiply(svd.VT);
//	    var R = svd.W*svd.VT;

//	    var g = q.Select(q_i => R.Multiply(q_i)).Select(v => new Vector3(v[0], v[1], v[2])).ToArray();


//	    var h = Time.fixedDeltaTime;
//	    for (int i = 0; i < v.Length; i++)
//	    {
//	        var start = v[i];
//	        var middle = alpha*((g[i] - x[i])/h);
//	        var end = h*f_ext[i]/m[i];
//	        f_ext[i] = Vector3.zero;

//            v[i] = start + middle + end;
//            bool isZero = v[i] == Vector3.zero;
//	        if (!isZero)
//	        {
//	            //print(isZero);
//	        }
//	    }

//	    for (int i = 0; i < x.Length; i++)
//	    {
//	        x[i] += h*v[i];
//	    } 


//        currentMesh.vertices = x;

//	}


//    public override void SetGoalPosition(int selectedIndex, Vector3 point)
//    {

//        f_ext[selectedIndex] = point - currentMesh.vertices[selectedIndex] * forceMult;
//    }

//    public override Vector3[] GetVerts()
//    {
//        return currentMesh.vertices;
//    }
//}