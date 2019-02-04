// GENERATED CODE
using System;
using System.Runtime.CompilerServices;

#pragma warning disable 0660, 0661

namespace Unity.Mathematics
{
    [System.Serializable]
    public partial struct float3x9 : System.IEquatable<float3x9>, IFormattable
    {
        public float3 c0;
        public float3 c1;
        public float3 c2;
        public float3 c3;
        public float3 c4;
        public float3 c5;
        public float3 c6;
        public float3 c7;
        public float3 c8;

        /// <summary>float3x9 zero value.</summary>
        public static readonly float3x9 zero;

        /// <summary>Constructs a float3x9 matrix from float3 vectors.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3x9(float3 c0, float3 c1, float3 c2, float3 c3, float3 c4, float3 c5, float3 c6, float3 c7, float3 c8)
        { 
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
            this.c3 = c3;
            this.c4 = c4;
            this.c5 = c5;
            this.c6 = c6;
            this.c7 = c7;
            this.c8 = c8;
        }

        /// <summary>Constructs a float3x9 matrix from 27 float values given in row-major order.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3x9(float m00, float m01, float m02, float m03, float m04, float m05, float m06, float m07, float m08,
                        float m10, float m11, float m12, float m13, float m14, float m15, float m16, float m17, float m18,
                        float m20, float m21, float m22, float m23, float m24, float m25, float m26, float m27, float m28)
        { 
            this.c0 = new float3(m00, m10, m20);
            this.c1 = new float3(m01, m11, m21);
            this.c2 = new float3(m02, m12, m22);
            this.c3 = new float3(m03, m13, m23);
            this.c4 = new float3(m04, m14, m24);
            this.c5 = new float3(m05, m15, m25);
            this.c6 = new float3(m06, m16, m26);
            this.c7 = new float3(m07, m17, m27);
            this.c8 = new float3(m08, m18, m28);
        }

        /// <summary>Constructs a float3x9 matrix from a single float value by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3x9(float v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
            this.c3 = v;
            this.c4 = v;
            this.c5 = v;
            this.c6 = v;
            this.c7 = v;
            this.c8 = v;
        }
        /// <summary>Constructs a float3x9 matrix from a single int value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3x9(int v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
            this.c3 = v;
            this.c4 = v;
            this.c5 = v;
            this.c6 = v;
            this.c7 = v;
            this.c8 = v;
        }
        
        /// <summary>Constructs a float3x9 matrix from a single uint value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3x9(uint v)
        {
            this.c0 = v;
            this.c1 = v;
            this.c2 = v;
            this.c3 = v;
            this.c4 = v;
            this.c5 = v;
            this.c6 = v;
            this.c7 = v;
            this.c8 = v;
        }
        

        /// <summary>Constructs a float3x9 matrix from a single double value by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3x9(double v)
        {
            this.c0 = (float3)v;
            this.c1 = (float3)v;
            this.c2 = (float3)v;
            this.c3 = (float3)v;
            this.c4 = (float3)v;
            this.c5 = (float3)v;
            this.c6 = (float3)v;
            this.c7 = (float3)v;
            this.c8 = (float3)v;
        }
        


        /// <summary>Implicitly converts a single float value to a float3x9 matrix by assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float3x9(float v) { return new float3x9(v); }
        
        /// <summary>Implicitly converts a single int value to a float3x9 matrix by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float3x9(int v) { return new float3x9(v); }
        
        /// <summary>Implicitly converts a single uint value to a float3x9 matrix by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float3x9(uint v) { return new float3x9(v); }
        
        /// <summary>Explicitly converts a single double value to a float3x9 matrix by converting it to float and assigning it to every component.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float3x9(double v) { return new float3x9(v); }
        

        /// <summary>Returns the result of a componentwise multiplication operation on two float3x9 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator * (float3x9 lhs, float3x9 rhs) { return new float3x9 (lhs.c0 * rhs.c0, lhs.c1 * rhs.c1, lhs.c2 * rhs.c2, lhs.c3 * rhs.c3, lhs.c4 * rhs.c4, lhs.c5 * rhs.c5, lhs.c6 * rhs.c6, lhs.c7 * rhs.c7, lhs.c8 * rhs.c8); }

        /// <summary>Returns the result of a componentwise multiplication operation on a float3x9 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator * (float3x9 lhs, float rhs) { return new float3x9 (lhs.c0 * rhs, lhs.c1 * rhs, lhs.c2 * rhs, lhs.c3 * rhs, lhs.c4 * rhs, lhs.c5 * rhs, lhs.c6 * rhs, lhs.c7 * rhs, lhs.c8 * rhs); }

        /// <summary>Returns the result of a componentwise multiplication operation on a float value and a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator * (float lhs, float3x9 rhs) { return new float3x9 (lhs * rhs.c0, lhs * rhs.c1, lhs * rhs.c2, lhs * rhs.c3, lhs * rhs.c4, lhs * rhs.c5, lhs * rhs.c6, lhs * rhs.c7, lhs * rhs.c8); }


        /// <summary>Returns the result of a componentwise addition operation on two float3x9 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator + (float3x9 lhs, float3x9 rhs) { return new float3x9 (lhs.c0 + rhs.c0, lhs.c1 + rhs.c1, lhs.c2 + rhs.c2, lhs.c3 + rhs.c3, lhs.c4 + rhs.c4, lhs.c5 + rhs.c5, lhs.c6 + rhs.c6, lhs.c7 + rhs.c7, lhs.c8 + rhs.c8); }

        /// <summary>Returns the result of a componentwise addition operation on a float3x9 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator + (float3x9 lhs, float rhs) { return new float3x9 (lhs.c0 + rhs, lhs.c1 + rhs, lhs.c2 + rhs, lhs.c3 + rhs, lhs.c4 + rhs, lhs.c5 + rhs, lhs.c6 + rhs, lhs.c7 + rhs, lhs.c8 + rhs); }

        /// <summary>Returns the result of a componentwise addition operation on a float value and a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator + (float lhs, float3x9 rhs) { return new float3x9 (lhs + rhs.c0, lhs + rhs.c1, lhs + rhs.c2, lhs + rhs.c3, lhs + rhs.c4, lhs + rhs.c5, lhs + rhs.c6, lhs + rhs.c7, lhs + rhs.c8); }


        /// <summary>Returns the result of a componentwise subtraction operation on two float3x9 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator - (float3x9 lhs, float3x9 rhs) { return new float3x9 (lhs.c0 - rhs.c0, lhs.c1 - rhs.c1, lhs.c2 - rhs.c2, lhs.c3 - rhs.c3, lhs.c4 - rhs.c4, lhs.c5 - rhs.c5, lhs.c6 - rhs.c6, lhs.c7 - rhs.c7, lhs.c8 - rhs.c8); }

        /// <summary>Returns the result of a componentwise subtraction operation on a float3x9 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator - (float3x9 lhs, float rhs) { return new float3x9 (lhs.c0 - rhs, lhs.c1 - rhs, lhs.c2 - rhs, lhs.c3 - rhs, lhs.c4 - rhs, lhs.c5 - rhs, lhs.c6 - rhs, lhs.c7 - rhs, lhs.c8 - rhs); }

        /// <summary>Returns the result of a componentwise subtraction operation on a float value and a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator - (float lhs, float3x9 rhs) { return new float3x9 (lhs - rhs.c0, lhs - rhs.c1, lhs - rhs.c2, lhs - rhs.c3, lhs - rhs.c4, lhs - rhs.c5, lhs - rhs.c6, lhs - rhs.c7, lhs - rhs.c8); }


        /// <summary>Returns the result of a componentwise division operation on two float3x9 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator / (float3x9 lhs, float3x9 rhs) { return new float3x9 (lhs.c0 / rhs.c0, lhs.c1 / rhs.c1, lhs.c2 / rhs.c2, lhs.c3 / rhs.c3, lhs.c4 / rhs.c4, lhs.c5 / rhs.c5, lhs.c6 / rhs.c6, lhs.c7 / rhs.c7, lhs.c8 / rhs.c8); }

        /// <summary>Returns the result of a componentwise division operation on a float3x9 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator / (float3x9 lhs, float rhs) { return new float3x9 (lhs.c0 / rhs, lhs.c1 / rhs, lhs.c2 / rhs, lhs.c3 / rhs, lhs.c4 / rhs, lhs.c5 / rhs, lhs.c6 / rhs, lhs.c7 / rhs, lhs.c8 / rhs); }

        /// <summary>Returns the result of a componentwise division operation on a float value and a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator / (float lhs, float3x9 rhs) { return new float3x9 (lhs / rhs.c0, lhs / rhs.c1, lhs / rhs.c2, lhs / rhs.c3, lhs / rhs.c4, lhs / rhs.c5, lhs / rhs.c6, lhs / rhs.c7, lhs / rhs.c8); }


        /// <summary>Returns the result of a componentwise modulus operation on two float3x9 matrices.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator % (float3x9 lhs, float3x9 rhs) { return new float3x9 (lhs.c0 % rhs.c0, lhs.c1 % rhs.c1, lhs.c2 % rhs.c2, lhs.c3 % rhs.c3, lhs.c4 % rhs.c4, lhs.c5 % rhs.c5, lhs.c6 % rhs.c6, lhs.c7 % rhs.c7, lhs.c8 % rhs.c8); }

        /// <summary>Returns the result of a componentwise modulus operation on a float3x9 matrix and a float value.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator % (float3x9 lhs, float rhs) { return new float3x9 (lhs.c0 % rhs, lhs.c1 % rhs, lhs.c2 % rhs, lhs.c3 % rhs, lhs.c4 % rhs, lhs.c5 % rhs, lhs.c6 % rhs, lhs.c7 % rhs, lhs.c8 % rhs); }

        /// <summary>Returns the result of a componentwise modulus operation on a float value and a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator % (float lhs, float3x9 rhs) { return new float3x9 (lhs % rhs.c0, lhs % rhs.c1, lhs % rhs.c2, lhs % rhs.c3, lhs % rhs.c4, lhs % rhs.c5, lhs % rhs.c6, lhs % rhs.c7, lhs % rhs.c8); }


        /// <summary>Returns the result of a componentwise increment operation on a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator ++ (float3x9 val) { return new float3x9 (++val.c0, ++val.c1, ++val.c2, ++val.c3, ++val.c4, ++val.c5, ++val.c6, ++val.c7, ++val.c8); }


        /// <summary>Returns the result of a componentwise decrement operation on a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator -- (float3x9 val) { return new float3x9 (--val.c0, --val.c1, --val.c2, --val.c3, --val.c4, --val.c5, --val.c6, --val.c7, --val.c8); }

        /// <summary>Returns the result of a componentwise unary minus operation on a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator - (float3x9 val) { return new float3x9 (-val.c0, -val.c1, -val.c2, -val.c3, -val.c4, -val.c5, -val.c6, -val.c7, -val.c8); }


        /// <summary>Returns the result of a componentwise unary plus operation on a float3x9 matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3x9 operator + (float3x9 val) { return new float3x9 (+val.c0, +val.c1, +val.c2, +val.c3, +val.c4, +val.c5, +val.c6, +val.c7, +val.c8); }

        
        /// <summary>Returns the float3 element at a specified index.</summary>
        unsafe public float3 this[int index]
        {
            get
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= 9)
                    throw new System.ArgumentException("index must be between[0...8]");
#endif
                fixed (float3x9* array = &this) { return ((float3*)array)[index]; }
            }
            set
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if ((uint)index >= 9)
                    throw new System.ArgumentException("index must be between[0...8]");
#endif
                fixed (float3* array = &c0) { array[index] = value; }
            }
        }

        /// <summary>Returns true if the float3x9 is equal to a given float3x9, false otherwise.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(float3x9 rhs) { return c0.Equals(rhs.c0) && c1.Equals(rhs.c1) && c2.Equals(rhs.c2) && c3.Equals(rhs.c3) && c4.Equals(rhs.c4) && c5.Equals(rhs.c5) && c6.Equals(rhs.c6) && c7.Equals(rhs.c7) && c8.Equals(rhs.c8); }

        /// <summary>Returns true if the float3x9 is equal to a given float3x9, false otherwise.</summary>
        public override bool Equals(object o) { return Equals((float3x9)o); }


        /// <summary>Returns a string representation of the float3x9.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("float3x9({0}f, {1}f, {2}f, {3}f, {4}f, {5}f, {6}f, {7}f, {8}f,  {9}f, {10}f, {11}f, {12}f, {13}f, {14}f, {15}f, {16}f, {17}f,  {18}f, {19}f, {20}f, {21}f, {22}f, {23}f, {24}f, {25}f, {26}f)", c0.x, c1.x, c2.x, c3.x, c4.x, c5.x, c6.x, c7.x, c8.x, c0.y, c1.y, c2.y, c3.y, c4.y, c5.y, c6.y, c7.y, c8.y, c0.z, c1.z, c2.z, c3.z, c4.z, c5.z, c6.z, c7.z, c8.z);
        }

        /// <summary>Returns a string representation of the float3x9 using a specified format and culture-specific format information.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("float3x9({0}f, {1}f, {2}f, {3}f, {4}f, {5}f, {6}f, {7}f, {8}f,  {9}f, {10}f, {11}f, {12}f, {13}f, {14}f, {15}f, {16}f, {17}f,  {18}f, {19}f, {20}f, {21}f, {22}f, {23}f, {24}f, {25}f, {26}f)", c0.x.ToString(format, formatProvider), c1.x.ToString(format, formatProvider), c2.x.ToString(format, formatProvider), c3.x.ToString(format, formatProvider), c4.x.ToString(format, formatProvider), c5.x.ToString(format, formatProvider), c6.x.ToString(format, formatProvider), c7.x.ToString(format, formatProvider), c8.x.ToString(format, formatProvider), c0.y.ToString(format, formatProvider), c1.y.ToString(format, formatProvider), c2.y.ToString(format, formatProvider), c3.y.ToString(format, formatProvider), c4.y.ToString(format, formatProvider), c5.y.ToString(format, formatProvider), c6.y.ToString(format, formatProvider), c7.y.ToString(format, formatProvider), c8.y.ToString(format, formatProvider), c0.z.ToString(format, formatProvider), c1.z.ToString(format, formatProvider), c2.z.ToString(format, formatProvider), c3.z.ToString(format, formatProvider), c4.z.ToString(format, formatProvider), c5.z.ToString(format, formatProvider), c6.z.ToString(format, formatProvider), c7.z.ToString(format, formatProvider), c8.z.ToString(format, formatProvider));
        }

    }
}
