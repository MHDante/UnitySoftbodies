using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
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

    public static Vector3 PointwiseMult(this Vector3 a, Vector3 b)
    {
        return new Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
    }
        
}
