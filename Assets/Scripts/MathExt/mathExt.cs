using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

namespace Mathematics.Extensions
{

    public static class mathExt
    {
        /// <summary>Returns the float3x3 matrix result of a matrix multiplication between a 3x1 column vector and a 1x3 row vector (b Transpose).</summary>
        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void mulT(in float3 a, in float3 bT, ref float3x3 result)
        {
            result.c0.x = a.x * bT.x;
            result.c1.x = a.x * bT.y;
            result.c2.x = a.x * bT.z;

            
            result.c0.y = a.y * bT.x;
            result.c1.y = a.y * bT.y;
            result.c2.y = a.y * bT.z;

            
            result.c0.z = a.z * bT.x;
            result.c1.z = a.z * bT.y;
            result.c2.z = a.z * bT.z;
        }


        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float9x9 mulT(in float9 a, in float9 bT) =>
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


        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void mulT(in float3 a, in float9 bT, ref float3x9 result)
        {
            result.c0.x = a.x * bT.r0;
            result.c1.x = a.x * bT.r1;
            result.c2.x = a.x * bT.r2;
            result.c3.x = a.x * bT.r3;
            result.c4.x = a.x * bT.r4;
            result.c5.x = a.x * bT.r5;
            result.c6.x = a.x * bT.r6;
            result.c7.x = a.x * bT.r7;
            result.c8.x = a.x * bT.r8;

            result.c0.y = a.y * bT.r0;
            result.c1.y = a.y * bT.r1;
            result.c2.y = a.y * bT.r2;
            result.c3.y = a.y * bT.r3;
            result.c4.y = a.y * bT.r4;
            result.c5.y = a.y * bT.r5;
            result.c6.y = a.y * bT.r6;
            result.c7.y = a.y * bT.r7;
            result.c8.y = a.y * bT.r8;

            result.c0.z = a.z * bT.r0;
            result.c1.z = a.z * bT.r1;
            result.c2.z = a.z * bT.r2;
            result.c3.z = a.z * bT.r3;
            result.c4.z = a.z * bT.r4;
            result.c5.z = a.z * bT.r5;
            result.c6.z = a.z * bT.r6;
            result.c7.z = a.z * bT.r7;
            result.c8.z = a.z * bT.r8;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void mul(in float3x9 a, in float9 b, ref float3 result)
        {
            result.x =
              a.c0.x * b.r0
            + a.c1.x * b.r1
            + a.c2.x * b.r2
            + a.c3.x * b.r3
            + a.c4.x * b.r4
            + a.c5.x * b.r5
            + a.c6.x * b.r6
            + a.c7.x * b.r7
            + a.c8.x * b.r8;

            
            result.y =
              a.c0.y * b.r0
            + a.c1.y * b.r1
            + a.c2.y * b.r2
            + a.c3.y * b.r3
            + a.c4.y * b.r4
            + a.c5.y * b.r5
            + a.c6.y * b.r6
            + a.c7.y * b.r7
            + a.c8.y * b.r8;

            
            result.z =
              a.c0.z * b.r0
            + a.c1.z * b.r1
            + a.c2.z * b.r2
            + a.c3.z * b.r3
            + a.c4.z * b.r4
            + a.c5.z * b.r5
            + a.c6.z * b.r6
            + a.c7.z * b.r7
            + a.c8.z * b.r8;
        }


        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 mul(in float3x9 a, in float9x9 b)
        {
            float3x9 result = new float3x9();

            for (int row = 0; row < 3; row++)
            {
                var rowVector = a.row(row);
                for (int column = 0; column < 9; column++)
                {
                    result[column][row] = csum(rowVector * b[column]);
                }
            }

            return result;
        }

        public static void ToOldMatrix(this in float3x3 mIn, ref Matrix4x4 mOut, float last = 0)
        {
            mOut[0, 0] = mIn[0][0];
            mOut[0, 1] = mIn[1][0];
            mOut[0, 2] = mIn[2][0];
            mOut[0, 3] = 0;
            mOut[1, 0] = mIn[0][1];
            mOut[1, 1] = mIn[1][1];
            mOut[1, 2] = mIn[2][1];
            mOut[1, 3] = 0;
            mOut[2, 0] = mIn[0][2];
            mOut[2, 1] = mIn[1][2];
            mOut[2, 2] = mIn[2][2];
            mOut[2, 3] = 0;
            mOut[3, 0] = 0;
            mOut[3, 1] = 0;
            mOut[3, 2] = 0;
            mOut[3, 3] = last;
        }


        public static void ToOldMatrix(this in float3x9 mIn, Matrix4x4[] mOut)
        {
            for (int i = 0; i < 3; i++)
            {
                mOut[i][0, 0] = mIn[i * 3 + 0][0];
                mOut[i][0, 1] = mIn[i * 3 + 1][0];
                mOut[i][0, 2] = mIn[i * 3 + 2][0];
                mOut[i][0, 3] = 0;
                mOut[i][1, 0] = mIn[i * 3 + 0][1];
                mOut[i][1, 1] = mIn[i * 3 + 1][1];
                mOut[i][1, 2] = mIn[i * 3 + 2][1];
                mOut[i][1, 3] = 0;
                mOut[i][2, 0] = mIn[i * 3 + 0][2];
                mOut[i][2, 1] = mIn[i * 3 + 1][2];
                mOut[i][2, 2] = mIn[i * 3 + 2][2];
                mOut[i][2, 3] = 0;
                mOut[i][3, 0] = 0;
                mOut[i][3, 1] = 0;
                mOut[i][3, 2] = 0;
                mOut[i][3, 3] = 0;
            }
        }

        // Gauss-Jordan inverse algorithm. 
        [BurstCompile]
        public static float9x9 inverse(in float9x9 a)
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

                    aT[row] -= aT[column] * scale;
                    augmented[row] -= augmented[column] * scale;
                }
            }

            return augmented.transpose();
        }

        [BurstCompile]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float csum(in float9 x) => x.r0 + x.r1 + x.r2 + x.r3 + x.r4 + x.r5 + x.r6 + x.r7 + x.r8;

        public static void PlusEquals(ref float3x9 a, in float3x9 b)
        {
            a.c0.x += b.c0.x;
            a.c1.x += b.c1.x;
            a.c2.x += b.c2.x;
            a.c3.x += b.c3.x;
            a.c4.x += b.c4.x;
            a.c5.x += b.c5.x;
            a.c6.x += b.c6.x;
            a.c7.x += b.c7.x;
            a.c8.x += b.c8.x;

            a.c0.y += b.c0.y;
            a.c1.y += b.c1.y;
            a.c2.y += b.c2.y;
            a.c3.y += b.c3.y;
            a.c4.y += b.c4.y;
            a.c5.y += b.c5.y;
            a.c6.y += b.c6.y;
            a.c7.y += b.c7.y;
            a.c8.y += b.c8.y;

            a.c0.z += b.c0.z;
            a.c1.z += b.c1.z;
            a.c2.z += b.c2.z;
            a.c3.z += b.c3.z;
            a.c4.z += b.c4.z;
            a.c5.z += b.c5.z;
            a.c6.z += b.c6.z;
            a.c7.z += b.c7.z;
            a.c8.z += b.c8.z;

        }
        public static void PlusEquals(ref float3x3 a, in float3x3 b)
        {
            a.c0.x += b.c0.x;
            a.c1.x += b.c1.x;
            a.c2.x += b.c2.x;

            a.c0.y += b.c0.y;
            a.c1.y += b.c1.y;
            a.c2.y += b.c2.y;

            a.c0.z += b.c0.z;
            a.c1.z += b.c1.z;
            a.c2.z += b.c2.z;

        }
    }




}