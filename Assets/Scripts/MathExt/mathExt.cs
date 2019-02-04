using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

namespace Mathematics.Extensions
{
    public static class mathExt
    {
        /// <summary>Returns the float3x3 matrix result of a matrix multiplication between a 3x1 column vector and a 1x3 row vector (b Transpose).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 mulT(this in float3 a, in float3 bT) =>
            new float3x3(a * bT.x, a * bT.y, a * bT.z);


        public static float9x9 mulT(this in float9 a, in float9 bT) =>
            new float9x9(
                a * bT.r0,
                a * bT.r1,
                a * bT.r2,
                a * bT.r3,
                a * bT.r4,
                a * bT.r5,
                a * bT.r6,
                a * bT.r7,
                a * bT.r8
            );


        public static float3x9 mulT(this in float3 a, in float9 bT) =>
            new float3x9(
                a * bT.r0,
                a * bT.r1,
                a * bT.r2,
                a * bT.r3,
                a * bT.r4,
                a * bT.r5,
                a * bT.r6,
                a * bT.r7,
                a * bT.r8
            );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 mul(this in float3x9 a, in float9 b)
        {
            float3 result = new float3();

            for (int i = 0; i < 3; i++)
            {
                float s = 0;
                for (int j = 0; j < 9; j++) s += a[j][i] * b[j];
                result[i] = s;
            }

            return result;
        }


        // Gauss-Jordan inverse algorithm. 
        public static float9x9 inverse(this in float9x9 a)
        {
            // rows are columns, ok?
            // index using [row][column]
            float9x9 aT = a.transpose();
            float9x9 augmented = float9x9.identity;

            for (int column = 0; column < 9; column++)
            {

                //Find Pivot
                float max = 0;
                int pivot = column;

                for (int row = column; row < 9; row++)
                {
                    var val = math.abs(aT[row][column]);
                    if (val > max)
                    {
                        pivot = row;
                        max = val;
                    }
                }
                if (max <= 0) throw new ArithmeticException("Un-invertible Matrix");

                // Row Swap if necessary
                if (pivot != column)
                {
                    // these are all rows.
                    var temp = aT[column];
                    aT[column] = aT[pivot];
                    aT[pivot] = temp;

                    temp = augmented[column]; // this is a row.
                    augmented[column] = augmented[pivot];
                    augmented[pivot] = temp;
                }

                // Row scale to set diagonals to 1
                {
                    var scale = aT[column][column];
                    Debug.Assert(scale == max);

                    aT[column] /= scale;
                    augmented[column] /= scale;
                }

                // Eliminate.
                for (int row = 0; row < 9; row++)
                {
                    if (row == column)
                    {
                        Debug.Assert(aT[row][column] == 1);
                        continue;
                    }

                    var scale = aT[row][column];
                    if (aT[row][column] == 0) continue;

                    aT[row] -= aT[column] / scale;
                    augmented[row] -= augmented[column] / scale;
                }
            }

            return augmented.transpose();
        }

        public static float csum(in float9 x) => x.r0 + x.r1 + x.r2 + x.r3 + x.r4 + x.r5 + x.r6 + x.r7 + x.r8;
    }


}