using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using UnityEngine;
    static class Utils
    {
        public static float DistanceToLine(this Vector3 point, Ray ray)
        {
            return Vector3.Cross(ray.direction, point - ray.origin).magnitude;
        }

        public static Vector3 Mean(this IEnumerable<Vector3> vectors)
        {
            var enumerable = vectors as IList<Vector3> ?? vectors.ToList();
            return enumerable.Aggregate((t, i) => t + i) / enumerable.Count();
        }

        public static Vector<float> Mean(this IEnumerable<Vector<float>> vectors, IEnumerable<float> weights = null)
        {
            var vectorArray = vectors as Vector<float>[] ?? vectors.ToArray();
            var sum = weights != null ?
                        vectorArray.Zip(weights, (v, z) => v * z).Aggregate((a, v) => a + v) : 
                        vectorArray.Aggregate((a, v) => a + v);

            return sum / vectorArray.Length;
        }


        public static Vector<float> ToNumerics(this Vector3 v)
        {
            return Vector<float>.Build.Dense(new[] { v.x, v.y, v.z });
        }
        public static Vector3 ToUnity(this Vector<float> v)
        {
            return new Vector3(v[0], v[1], v[2]);
        }
        public static Matrix4x4 ToUnity(this Matrix<float> M)
        {
            var ret = new Matrix4x4();
            var limit = M.ColumnCount;
            if(limit!= M.RowCount || (limit !=3 && limit !=4) )throw new ArgumentException("Bad Matrix Dimensions");
            for (int i = 0; i < limit; i++)
            {
                for (int j = 0; j < limit; j++)
                {
                    ret[i, j] = M[i, j];
                }
            }
            return ret;
        }

        public static Vector3 ExtractPosition(this Matrix4x4 m)
        {
            return m.GetColumn(3);
        }

        public static Quaternion ExtractRotation(this Matrix4x4 m)
        {
            return Quaternion.LookRotation(
                m.GetColumn(2),
                m.GetColumn(1)
            );
        }

        public static Vector3 ExtractScale(this Matrix4x4 m)
        {
            return new Vector3(
                m.GetColumn(0).magnitude,
                m.GetColumn(1).magnitude,
                m.GetColumn(2).magnitude
            );
        }

        public static void SetValue(this Vector<float> to, Vector3 from)
        {
            to[0] = from[0];
            to[1] = from[1];
            to[2] = from[2];
        }

        public static Matrix<float> Sqrt(this Matrix<float> A)
        {

            //1- [E D] = eig(A); sqrtm(A) = E * sqrt(D) * E' where D is a diagonal matrix.
            //> sqrt(D) is formed by taking the square root of the diagonal entries in D.


            Matrix<float> M = A.Transpose();
            //Todo: is there an advantage to the jacobi method? What method does this function use?
            var evd = M.Evd();
            var D = evd.D;
            var E = evd.EigenVectors;
            D = D.PointwiseSqrt();
        
            return (E * D * E.Transpose()).Transpose();
        
        }

        public static Vector3 PointwiseMult(this Vector3 a, Vector3 b)
        {
            return new Vector3(a.x*b.x,a.y*b.y,a.z*b.z);
        }
}
