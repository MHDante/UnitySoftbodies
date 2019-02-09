using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;

namespace Mathematics.Extensions
{
    public static class mathExt
    {
        /// <summary>Returns the float3x3 matrix result of a matrix multiplication between a 3x1 column vector and a 1x3 row vector (b Transpose).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x3 mulT(in float3 a, in float3 bT) =>
            new float3x3(a * bT.x, a * bT.y, a * bT.z);


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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 mulT(in float3 a, in float9 bT) =>
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
        public static float3 mul(in float3x9 a, in float9 b)
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
                mOut[i][0, 0] = mIn[i*3+0][0];
                mOut[i][0, 1] = mIn[i*3+1][0];
                mOut[i][0, 2] = mIn[i*3+2][0];
                mOut[i][0, 3] = 0;
                mOut[i][1, 0] = mIn[i*3+0][1];
                mOut[i][1, 1] = mIn[i*3+1][1];
                mOut[i][1, 2] = mIn[i*3+2][1];
                mOut[i][1, 3] = 0;
                mOut[i][2, 0] = mIn[i*3+0][2];
                mOut[i][2, 1] = mIn[i*3+1][2];
                mOut[i][2, 2] = mIn[i*3+2][2];
                mOut[i][2, 3] = 0;
                mOut[i][3, 0] = 0;
                mOut[i][3, 1] = 0;
                mOut[i][3, 2] = 0;
                mOut[i][3, 3] = 0;
            }
        }

        // Gauss-Jordan inverse algorithm. 
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float csum(in float9 x) => x.r0 + x.r1 + x.r2 + x.r3 + x.r4 + x.r5 + x.r6 + x.r7 + x.r8;
    }


}