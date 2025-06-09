using System;

namespace OpenRCF
{
    namespace AlgebraBase
    {
        public class MatrixBase
        {
            protected float[,] matrix;

            protected MatrixBase(int rows, int columns)
            {
                matrix = new float[rows, columns];
            }

            public int GetLength(int i) { return matrix.GetLength(i); }

            public float[,] Get
            {
                get
                {
                    float[,] result = new float[GetLength(0), GetLength(1)];

                    for (int i = 0; i < GetLength(0); i++)
                    {
                        for (int j = 0; j < GetLength(1); j++)
                        {
                            result[i, j] = matrix[i, j];
                        }
                    }

                    return result;
                }
            }

            public float[,] Set
            {
                set
                {
                    if (GetLength(0) == value.GetLength(0) && GetLength(1) == value.GetLength(1))
                    {
                        for (int i = 0; i < GetLength(0); i++)
                        {
                            for (int j = 0; j < GetLength(1); j++)
                            {
                                matrix[i, j] = value[i, j];
                            }
                        }
                    }
                    else
                    {
                        ConsoleWriteErrorMessage();
                    }
                }
            }

            public ref float[,] Ref { get { return ref matrix; } }

            public float this[int i, int j]
            {
                get
                {
                    if (i < matrix.GetLength(0) && j < matrix.GetLength(1)) return matrix[i, j];
                    else
                    {
                        Console.WriteLine("Error : i or j of Matrix[i, j] is incorrect.");
                        return 0;
                    }
                }
                set
                {
                    if (i < matrix.GetLength(0) && j < matrix.GetLength(1)) matrix[i, j] = value;
                    else Console.WriteLine("Error : i or j of Matrix[i, j] is incorrect.");
                }
            }

            public float[] this[int i]
            {
                get
                {
                    float[] result = new float[GetLength(1)];

                    if (i < GetLength(0))
                    {
                        for (int j = 0; j < GetLength(1); j++)
                        {
                            result[j] = matrix[i, j];
                        }
                    }
                    else
                    {
                        Console.WriteLine("Error : i of Matrix[i] is incorrect.");
                    }

                    return result;
                }
                set
                {
                    if (GetLength(1) == value.GetLength(0) && i < GetLength(0))
                    {
                        for (int j = 0; j < matrix.GetLength(1); j++)
                        {
                            matrix[i, j] = value[j];
                        }
                    }
                    else
                    {
                        Console.WriteLine("Error : i of Matrix[i] is incorrect.");
                    }
                }
            }

            public void SetValue(params float[] values)
            {
                int k = 0;

                for (int i = 0; i < GetLength(0); i++)
                {
                    for (int j = 0; j < GetLength(1); j++)
                    {
                        if (k < values.Length)
                        {
                            matrix[i, j] = values[k];
                            k++;
                        }
                        else
                        {
                            return;
                        }
                    }
                }
            }

            public float[] GetColumn(int column)
            {
                float[] result = new float[GetLength(0)];

                if (column < GetLength(1))
                {
                    for (int i = 0; i < GetLength(0); i++)
                    {
                        result[i] = matrix[i, column];
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }

                return result;
            }

            public void SetColumn(int column, float[] vector)
            {
                if (GetLength(0) == vector.Length && column < GetLength(1))
                {
                    for (int i = 0; i < GetLength(0); i++)
                    {
                        matrix[i, column] = vector[i];
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }
            }

            public void SetIdentity()
            {
                for (int i = 0; i < GetLength(0); i++)
                {
                    for (int j = 0; j < GetLength(1); j++)
                    {
                        if (i == j) matrix[i, j] = 1;
                        else matrix[i, j] = 0;
                    }
                }
            }

            protected static void ConsoleWriteErrorMessage()
            {
                Console.WriteLine("Size of Matrix do Not match.");
            }

            public void ConsoleWrite(byte digit = 2)
            {
                string text;
                string Fn = "F" + digit.ToString();
                int cursorPos;

                for (int i = 0; i < GetLength(0); i++)
                {
                    for (int j = 0; j < GetLength(1); j++)
                    {
                        text = this[i, j].ToString(Fn);
                        cursorPos = (12 + digit) * (j + 1) - text.Length;

                        if (0 < cursorPos && cursorPos < Console.BufferWidth)
                        {
                            Console.CursorLeft = cursorPos;
                            Console.Write(text);
                        }
                    }
                    Console.WriteLine();
                }
                Console.WriteLine();
            }

            public float Trace
            {
                get
                {
                    float result = 0;

                    for (int i = 0; i < GetLength(0) && i < GetLength(1); i++)
                    {
                        result += matrix[i, i];
                    }

                    return result;
                }
            }
        }

        public class VectorBase
        {
            protected float[] vector;

            protected VectorBase(int n)
            {
                vector = new float[n];
            }

            public int Length { get { return vector.Length; } }

            public ref float[] Ref { get { return ref vector; } }

            public float this[int i]
            {
                get
                {
                    if (i < Length) return vector[i];
                    else
                    {
                        Console.WriteLine("Error : i of Vector[i] is incorrect.");
                        return 0;
                    }
                }
                set
                {
                    if (i < Length) vector[i] = value;
                    else Console.WriteLine("Error : i of Vector[i] is incorrect.");
                }
            }

            public float[] Get
            {
                get
                {
                    float[] result = new float[vector.Length];
                    for (int i = 0; i < vector.Length; i++) { result[i] = vector[i]; }
                    return result;
                }
            }

            public float[] Set
            {
                set
                {
                    if (vector.Length == value.Length) for (int i = 0; i < value.Length; i++) { vector[i] = value[i]; }
                    else ConsoleWriteErrorMessage();
                }
            }

            public void SetZeroVector() { for (int i = 0; i < vector.Length; i++) { vector[i] = 0; } }

            protected static void ConsoleWriteErrorMessage()
            {
                Console.WriteLine("Error : Size of Vector do Not match.");
            }

            public void ConsoleWrite(byte digit = 2)
            {
                Console.Write("  [");
                for (int i = 0; i < vector.Length; i++)
                {
                    if (0 <= vector[i]) Console.Write("   " + vector[i].ToString("F" + digit.ToString()) + "   ");
                    else Console.Write("  " + vector[i].ToString("F" + digit.ToString()) + "   ");
                }
                Console.WriteLine("]^T");
            }

        }

        public class BlockVectorBase
        {
            protected Vector[] vector;

            public int[] Rows { get; private set; }

            protected BlockVectorBase(int[] rows)
            {
                vector = new Vector[rows.Length];

                for (int I = 0; I < vector.Length; I++)
                {
                    vector[I] = new Vector(rows[I]);
                }

                Rows = rows;
            }

            public int Length { get { return vector.Length; } }

            public ref Vector[] Ref { get { return ref vector; } }

            public Vector this[int I]
            {
                get
                {
                    if (I < Length) return vector[I];
                    else
                    {
                        Console.WriteLine("Error : i of BlockVector[i] is incorrect.");
                        return new Vector(0);
                    }
                }
                set
                {
                    if (I < Length) vector[I].Set = value.Get;
                    else Console.WriteLine("Error : i of BlockVector[i] is incorrect.");
                }
            }

            public Vector[] Get
            {
                get
                {
                    Vector[] result = new Vector[Length];

                    for (int I = 0; I < Length; I++)
                    {
                        result[I] = new Vector(vector[I].Length);
                        result[I].Ref = vector[I].Get;
                    }

                    return result;
                }
            }

            public Vector[] Set
            {
                set
                {
                    if (Length == value.Length)
                    {
                        for (int I = 0; I < vector.Length; I++)
                        {
                            vector[I].Set = value[I].Get;
                        }
                    }
                    else
                    {
                        ConsoleWriteErrorMessage();
                    }
                }
            }

            public void SetZeroVector() { for (int I = 0; I < vector.Length; I++) { vector[I].SetZeroVector(); } }

            protected static void ConsoleWriteErrorMessage()
            {
                Console.WriteLine("Error : Size of BlockVector do Not match.");
            }

        }

    }

    public class Matrix : AlgebraBase.MatrixBase
    {
        public bool IsFixedZero { get; private set; } = false;

        public Matrix(int rows, int columns) : base(rows, columns) { }

        public void SetDiagonal(float diag)
        {
            for (int i = 0; i < GetLength(0); i++)
            {
                for (int j = 0; j < GetLength(1); j++)
                {
                    if (i == j) matrix[i, j] = diag;
                    else matrix[i, j] = 0;
                }
            }
        }

        public void SetDiagonal(params float[] diag)
        {
            int k = 0;

            for (int i = 0; i < GetLength(0); i++)
            {
                for (int j = 0; j < GetLength(1); j++)
                {
                    if (k < diag.Length)
                    {
                        if (i == j)
                        {
                            matrix[i, j] = diag[k];
                            k++;
                        }
                        else matrix[i, j] = 0;
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }

        public void SetZeroMatrix()
        {
            for (int i = 0; i < GetLength(0); i++)
            {
                for (int j = 0; j < GetLength(1); j++)
                {
                    matrix[i, j] = 0;
                }
            }
        }

        public void FixZeroMatrix()
        {
            SetZeroMatrix();
            IsFixedZero = true;
        }

        private static Random random = new Random();

        public void SetRandom(int min = -10, int max = 10)
        {
            for (int i = 0; i < GetLength(0); i++)
            {
                for (int j = 0; j < GetLength(1); j++)
                {
                    matrix[i, j] = random.Next(min, max);
                }
            }
        }

        public void Follow(Matrix m) { matrix = m.Ref; }

        public void Follow(RotationMatrix m) { matrix = m.Ref; }

        public void QuitFollow() { matrix = Get; }

        public static implicit operator Matrix(float[,] m)
        {
            Matrix result = new Matrix(m.GetLength(0), m.GetLength(1));
            result.Ref = m;
            return result;
        }

        public static implicit operator Matrix(RotationMatrix m)
        {
            Matrix result = new Matrix(m.GetLength(0), m.GetLength(1));
            result.Set = m.Ref;
            return result;
        }

        public float[,] Times(float[,] m)
        {
            float[,] result = new float[GetLength(0), m.GetLength(1)];

            if (GetLength(1) == m.GetLength(0))
            {
                for (int i = 0; i < GetLength(0); i++)
                {
                    for (int j = 0; j < m.GetLength(1); j++)
                    {
                        for (int k = 0; k < GetLength(1); k++)
                        {
                            result[i, j] += matrix[i, k] * m[k, j];
                        }
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public float[,] Times(Matrix m) { return Times(m.Get); }

        public float[] Times(float[] v)
        {
            float[] result = new float[GetLength(0)];

            if (GetLength(1) == v.Length)
            {
                for (int i = 0; i < GetLength(0); i++)
                {
                    for (int j = 0; j < v.Length; j++)
                    {
                        result[i] += matrix[i, j] * v[j];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public float[] Times(Vector v) { return Times(v.Get); }

        private static Matrix Product(float[,] m1, float[,] m2)
        {
            Matrix result = new Matrix(m1.GetLength(0), m2.GetLength(1));

            if (m1.GetLength(1) == m2.GetLength(0))
            {
                for (int i = 0; i < m1.GetLength(0); i++)
                {
                    for (int j = 0; j < m2.GetLength(1); j++)
                    {
                        for (int k = 0; k < m1.GetLength(1); k++)
                        {
                            result.Ref[i, j] += m1[i, k] * m2[k, j];
                        }
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static Matrix operator *(Matrix m1, Matrix m2) { return Product(m1.Get, m2.Get); }

        public static Matrix operator *(Matrix m1, float[,] m2) { return Product(m1.Get, m2); }

        public static Matrix operator *(float[,] m1, Matrix m2) { return Product(m1, m2.Get); }

        private static Vector Product(float[,] m, float[] v)
        {
            Vector result = new Vector(m.GetLength(0));

            if (m.GetLength(1) == v.Length)
            {
                for (int i = 0; i < m.GetLength(0); i++)
                {
                    for (int j = 0; j < v.Length; j++)
                    {
                        result.Ref[i] += m[i, j] * v[j];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static Vector operator *(Matrix m, Vector v) { return Product(m.Get, v.Get); }

        public static Vector operator *(Matrix m, float[] v) { return Product(m.Get, v); }

        private static Matrix Product(float s, float[,] m)
        {
            Matrix result = new Matrix(m.GetLength(0), m.GetLength(1));

            for (int i = 0; i < m.GetLength(0); i++)
            {
                for (int j = 0; j < m.GetLength(1); j++)
                {
                    result.Ref[i, j] = s * m[i, j];
                }
            }

            return result;
        }

        public static Matrix operator *(float s, Matrix m) { return Product(s, m.Get); }

        public static Matrix operator *(Matrix m, float s) { return Product(s, m.Get); }

        private static Matrix HadamardProduct(float[,] m1, float[,] m2)
        {
            Matrix result = new Matrix(m1.GetLength(0), m1.GetLength(1));

            if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1))
            {
                for (int i = 0; i < m1.GetLength(0); i++)
                {
                    for (int j = 0; j < m1.GetLength(1); j++)
                    {
                        result.Ref[i, j] = m1[i, j] * m2[i, j];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static Matrix operator ^(Matrix m1, Matrix m2) { return HadamardProduct(m1.Get, m2.Get); }

        public static Matrix operator ^(Matrix m1, float[,] m2) { return HadamardProduct(m1.Get, m2); }

        public static Matrix operator ^(float[,] m1, Matrix m2) { return HadamardProduct(m1, m2.Get); }

        private static Matrix Plus(float[,] m1, float[,] m2)
        {
            Matrix result = new Matrix(m1.GetLength(0), m1.GetLength(1));

            if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1))
            {
                for (int i = 0; i < m1.GetLength(0); i++)
                {
                    for (int j = 0; j < m1.GetLength(1); j++)
                    {
                        result.Ref[i, j] = m1[i, j] + m2[i, j];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static Matrix operator +(Matrix m1, Matrix m2) { return Plus(m1.Get, m2.Get); }

        public static Matrix operator +(Matrix m1, float[,] m2) { return Plus(m1.Get, m2); }

        public static Matrix operator +(float[,] m1, Matrix m2) { return Plus(m1, m2.Get); }

        private static Matrix Minus(float[,] m1, float[,] m2)
        {
            Matrix result = new Matrix(m1.GetLength(0), m1.GetLength(1));

            if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1))
            {
                for (int i = 0; i < m1.GetLength(0); i++)
                {
                    for (int j = 0; j < m1.GetLength(1); j++)
                    {
                        result.Ref[i, j] = m1[i, j] - m2[i, j];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static Matrix operator -(Matrix m1, Matrix m2) { return Minus(m1.Get, m2.Get); }

        public static Matrix operator -(Matrix m1, float[,] m2) { return Minus(m1.Get, m2); }

        public static Matrix operator -(float[,] m1, Matrix m2) { return Minus(m1, m2.Get); }

        public static Matrix operator -(Matrix m)
        {
            Matrix result = new Matrix(m.GetLength(0), m.GetLength(1));

            for (int i = 0; i < m.GetLength(0); i++)
            {
                for (int j = 0; j < m.GetLength(1); j++)
                {
                    result.Ref[i, j] = -m.Ref[i, j];
                }
            }

            return result;
        }

        public Matrix Trans
        {
            get
            {
                Matrix result = new Matrix(GetLength(1), GetLength(0));

                for (int i = 0; i < GetLength(1); i++)
                {
                    for (int j = 0; j < GetLength(0); j++)
                    {
                        result.Ref[i, j] = matrix[j, i];
                    }
                }

                return result;
            }
        }

        public Matrix Inv
        {
            get
            {
                Matrix result = new Matrix(GetLength(0), GetLength(1));

                if (GetLength(0) == GetLength(1))
                {
                    result.SetIdentity();

                    float[,] origin = Get;
                    int max;
                    float tmp;

                    for (int i = 0; i < GetLength(0); i++)
                    {
                        max = i;

                        for (int j = i + 1; j < GetLength(0); j++)
                        {
                            if (Math.Abs(origin[max, i]) < Math.Abs(origin[j, i]))
                            {
                                max = j;
                            }
                        }

                        if (max != i)
                        {
                            for (int k = 0; k < GetLength(0); k++)
                            {
                                tmp = origin[max, k];
                                origin[max, k] = origin[i, k];
                                origin[i, k] = tmp;

                                tmp = result.Ref[max, k];
                                result.Ref[max, k] = result.Ref[i, k];
                                result.Ref[i, k] = tmp;
                            }
                        }

                        tmp = origin[i, i];

                        if (0.00001f < Math.Abs(tmp))
                        {
                            for (int k = 0; k < GetLength(0); k++)
                            {
                                origin[i, k] /= tmp;
                                result.Ref[i, k] /= tmp;
                            }

                            for (int j = 0; j < GetLength(0); j++)
                            {
                                if (i != j)
                                {
                                    tmp = origin[j, i] / origin[i, i];

                                    for (int k = 0; k < GetLength(0); k++)
                                    {
                                        origin[j, k] = origin[j, k] - origin[i, k] * tmp;
                                        result.Ref[j, k] = result.Ref[j, k] - result.Ref[i, k] * tmp;
                                    }
                                }
                            }
                        }
                        else
                        {
                            Console.WriteLine("Error : Unable to compute inverse matrix.");
                            break;
                        }
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }

                return result;
            }
        }

        public Matrix PseInv
        {
            get
            {
                if (GetLength(0) < GetLength(1))
                {
                    Matrix m = Get * Trans;
                    return Trans * m.Inv;
                }
                else if (GetLength(0) > GetLength(1))
                {
                    Matrix m = Trans * Get;
                    return m.Inv * Trans;
                }
                else
                {
                    return Inv;
                }
            }
        }

    }

    public class RotationMatrix : AlgebraBase.MatrixBase
    {
        public RotationMatrix(double roll = 0, double pitch = 0, double yaw = 0) : base(3, 3)
        {
            if (Math.Abs(roll) + Math.Abs(pitch) + Math.Abs(yaw) < 0.0001f)
            {
                SetIdentity();
            }
            else
            {
                SetRollPitchYaw((float)roll, (float)pitch, (float)yaw);
            }
        }

        public void SetRollPitchYaw(float roll, float pitch, float yaw)
        {
            float Sr = (float)Math.Sin(roll);
            float Sp = (float)Math.Sin(pitch);
            float Sy = (float)Math.Sin(yaw);

            float Cr = (float)Math.Cos(roll);
            float Cp = (float)Math.Cos(pitch);
            float Cy = (float)Math.Cos(yaw);

            matrix[0, 0] = Cp * Cy;
            matrix[1, 0] = Sr * Sp * Cy + Cr * Sy;
            matrix[2, 0] = -Cr * Sp * Cy + Sr * Sy;

            matrix[0, 1] = -Cp * Sy;
            matrix[1, 1] = -Sr * Sp * Sy + Cr * Cy;
            matrix[2, 1] = Cr * Sp * Sy + Sr * Cy;

            matrix[0, 2] = Sp;
            matrix[1, 2] = -Sr * Cp;
            matrix[2, 2] = Cr * Cp;
        }

        public void SetRollPitchYaw(double roll, double pitch, double yaw)
        {
            SetRollPitchYaw((float)roll, (float)pitch, (float)yaw);
        }

        public float[] RollPitchYaw
        {
            get
            {
                float[] result = new float[3];
                result[0] = (float)Math.Atan2(-matrix[1, 2], matrix[2, 2]);
                result[1] = (float)Math.Atan2(matrix[0, 2], -matrix[1, 2] * Math.Sin(result[0]) + matrix[2, 2] * Math.Cos(result[0]));
                result[2] = (float)Math.Atan2(-matrix[0, 1], matrix[0, 0]);
                return result;
            }
        }

        private static float[,] Rx(float theta)
        {
            float[,] result = new float[3, 3];

            result[0, 0] = 1;
            result[0, 1] = 0;
            result[0, 2] = 0;

            result[1, 0] = 0;
            result[1, 1] = (float)Math.Cos(theta);
            result[1, 2] = -(float)Math.Sin(theta);

            result[2, 0] = 0;
            result[2, 1] = (float)Math.Sin(theta);
            result[2, 2] = (float)Math.Cos(theta);

            return result;
        }

        public void SetRx(float theta) { Set = Rx(theta); }

        public void SetRx(double theta) { Set = Rx((float)theta); }

        private static float[,] Ry(float theta)
        {
            float[,] result = new float[3, 3];

            result[0, 0] = (float)Math.Cos(theta);
            result[0, 1] = 0;
            result[0, 2] = (float)Math.Sin(theta);

            result[1, 0] = 0;
            result[1, 1] = 1;
            result[1, 2] = 0;

            result[2, 0] = -(float)Math.Sin(theta);
            result[2, 1] = 0;
            result[2, 2] = (float)Math.Cos(theta);

            return result;
        }

        public void SetRy(float theta) { Set = Ry(theta); }

        public void SetRy(double theta) { Set = Ry((float)theta); }

        private static float[,] Rz(float theta)
        {
            float[,] result = new float[3, 3];

            result[0, 0] = (float)Math.Cos(theta);
            result[0, 1] = -(float)Math.Sin(theta);
            result[0, 2] = 0;

            result[1, 0] = (float)Math.Sin(theta);
            result[1, 1] = (float)Math.Cos(theta);
            result[1, 2] = 0;

            result[2, 0] = 0;
            result[2, 1] = 0;
            result[2, 2] = 1;

            return result;
        }

        public void SetRz(float theta) { Set = Rz(theta); }

        public void SetRz(double theta) { Set = Rz((float)theta); }

        private static float[,] Rn(float theta, float[] axis)
        {
            float[,] result = new float[3, 3];

            if (axis.Length == 3)
            {
                float Cth = (float)Math.Cos(theta);
                float Sth = (float)Math.Sin(theta);

                result[0, 0] = Cth + axis[0] * axis[0] * (1 - Cth);
                result[0, 1] = axis[0] * axis[1] * (1 - Cth) - axis[2] * Sth;
                result[0, 2] = axis[0] * axis[2] * (1 - Cth) + axis[1] * Sth;

                result[1, 0] = axis[0] * axis[1] * (1 - Cth) + axis[2] * Sth;
                result[1, 1] = Cth + axis[1] * axis[1] * (1 - Cth);
                result[1, 2] = axis[1] * axis[2] * (1 - Cth) - axis[0] * Sth;

                result[2, 0] = axis[0] * axis[2] * (1 - Cth) - axis[1] * Sth;
                result[2, 1] = axis[1] * axis[2] * (1 - Cth) + axis[0] * Sth;
                result[2, 2] = Cth + axis[2] * axis[2] * (1 - Cth);
            }
            else
            {
                result[0, 0] = 1;
                result[1, 1] = 1;
                result[2, 2] = 1;

                Console.WriteLine("Error : axis of Rn(theta, axis) is incorrect.");
            }

            return result;
        }

        public void SetRn(float theta, float[] axis) { Set = Rn(theta, axis); }

        public void SetRn(float theta, Vector axis) { Set = Rn(theta, axis.Get); }

        public void Follow(RotationMatrix m) { matrix = m.Ref; }

        public void QuitFollow() { matrix = Get; }

        public RotationMatrix Trans
        {
            get
            {
                RotationMatrix result = new RotationMatrix();

                for (int i = 0; i < GetLength(1); i++)
                {
                    for (int j = 0; j < GetLength(0); j++)
                    {
                        result.Ref[i, j] = matrix[j, i];
                    }
                }

                return result;
            }
        }

        public RotationMatrix Inv { get { return Trans; } }

        public static implicit operator RotationMatrix(float[,] m)
        {
            RotationMatrix result = new RotationMatrix();
            result.Set = m;
            return result;
        }

        public float[,] Times(float[,] m)
        {
            float[,] result = new float[GetLength(0), m.GetLength(1)];

            if (GetLength(1) == m.GetLength(0))
            {
                for (int i = 0; i < GetLength(0); i++)
                {
                    for (int j = 0; j < m.GetLength(1); j++)
                    {
                        for (int k = 0; k < GetLength(1); k++)
                        {
                            result[i, j] += matrix[i, k] * m[k, j];
                        }
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public float[,] Times(RotationMatrix m) { return Times(m.Get); }

        public void SetTimesRx(float theta) { Set = Times(Rx(theta)); }

        public void SetTimesRy(float theta) { Set = Times(Ry(theta)); }

        public void SetTimesRz(float theta) { Set = Times(Rz(theta)); }

        public void SetTimesRn(float theta, float[] axis) { Set = Times(Rn(theta, axis)); }

        public float[] Times(float[] v)
        {
            float[] result = new float[GetLength(0)];

            if (GetLength(1) == v.Length)
            {
                for (int i = 0; i < GetLength(0); i++)
                {
                    for (int j = 0; j < v.Length; j++)
                    {
                        result[i] += matrix[i, j] * v[j];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public float[] Times(Vector v) { return Times(v.Get); }

        private static RotationMatrix Product(float[,] m1, float[,] m2)
        {
            RotationMatrix result = new RotationMatrix();
            result.Ref[0, 0] = 0;
            result.Ref[1, 1] = 0;
            result.Ref[2, 2] = 0;

            if (m1.GetLength(0) == 3 && m1.GetLength(1) == 3 && m2.GetLength(0) == 3 && m2.GetLength(1) == 3)
            {
                for (int i = 0; i < m1.GetLength(0); i++)
                {
                    for (int j = 0; j < m2.GetLength(1); j++)
                    {
                        for (int k = 0; k < m1.GetLength(1); k++)
                        {
                            result.Ref[i, j] += m1[i, k] * m2[k, j];
                        }
                    }
                }
            }
            else
            {
                result.SetIdentity();
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static RotationMatrix operator *(RotationMatrix m1, RotationMatrix m2) { return Product(m1.Get, m2.Get); }

        public static RotationMatrix operator *(float[,] m1, RotationMatrix m2) { return Product(m1, m2.Get); }

        public static RotationMatrix operator *(RotationMatrix m1, float[,] m2) { return Product(m1.Get, m2); }

        private static Vector Product(float[,] m, float[] v)
        {
            Vector result = new Vector(m.GetLength(0));

            if (m.GetLength(1) == v.Length)
            {
                for (int i = 0; i < m.GetLength(0); i++)
                {
                    for (int j = 0; j < v.Length; j++)
                    {
                        result.Ref[i] += m[i, j] * v[j];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static Vector operator *(RotationMatrix m, Vector v) { return Product(m.Get, v.Get); }

        public static Vector operator *(RotationMatrix m, float[] v) { return Product(m.Get, v); }

        public static RotationMatrix operator -(RotationMatrix m)
        {
            RotationMatrix result = new RotationMatrix();

            for (int i = 0; i < m.GetLength(0); i++)
            {
                for (int j = 0; j < m.GetLength(1); j++)
                {
                    result.Ref[i, j] = -m.Ref[i, j];
                }
            }

            return result;
        }

        private readonly static float epsilon = 0.0001f;
        public Vector AngleAxisVector
        {
            get
            {
                Vector result = new Vector(3);
                float[] s = new float[3];
                s[0] = matrix[2, 1] - matrix[1, 2];
                s[1] = matrix[0, 2] - matrix[2, 0];
                s[2] = matrix[1, 0] - matrix[0, 1];
                float t = Trace;

                if (-1 + epsilon < t && t < 3 - epsilon)
                {
                    float sNorm = (float)Math.Sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
                    float theta = (float)Math.Atan2(sNorm, Trace - 1);

                    if (epsilon < sNorm)
                    {
                        result.Ref[0] = (theta / sNorm) * s[0];
                        result.Ref[1] = (theta / sNorm) * s[1];
                        result.Ref[2] = (theta / sNorm) * s[2];
                    }
                    else
                    {
                        Console.WriteLine("Error : Norm of s in Angle Axis Vector is Zero.");
                    }
                }
                else if (3 - epsilon <= t)
                {
                    result.Ref[0] = 0.5f * s[0];
                    result.Ref[1] = 0.5f * s[1];
                    result.Ref[2] = 0.5f * s[2];
                }
                else if (t <= -1 + epsilon)
                {
                    float nxAbs = (float)Math.Sqrt(0.5f * (matrix[0, 0] + 1));
                    float nyAbs = (float)Math.Sqrt(0.5f * (matrix[1, 1] + 1));
                    float nzAbs = (float)Math.Sqrt(0.5f * (matrix[2, 2] + 1));

                    if (matrix[0, 1] > -epsilon && matrix[1, 2] > -epsilon && matrix[2, 0] > -epsilon)
                    {
                        result.Ref[0] = (float)Math.PI * nxAbs;
                        result.Ref[1] = (float)Math.PI * nyAbs;
                        result.Ref[2] = (float)Math.PI * nzAbs;

                    }
                    else if (matrix[0, 1] > -epsilon && matrix[1, 2] < epsilon && matrix[2, 0] < epsilon)
                    {
                        result.Ref[0] = (float)Math.PI * nxAbs;
                        result.Ref[1] = (float)Math.PI * nyAbs;
                        result.Ref[2] = -(float)Math.PI * nzAbs;
                    }
                    else if (matrix[0, 1] < epsilon && matrix[1, 2] < epsilon && matrix[2, 0] > -epsilon)
                    {
                        result.Ref[0] = (float)Math.PI * nxAbs;
                        result.Ref[1] = -(float)Math.PI * nyAbs;
                        result.Ref[2] = (float)Math.PI * nzAbs;
                    }
                    else if (matrix[0, 1] < epsilon && matrix[1, 2] > -epsilon && matrix[2, 0] < epsilon)
                    {
                        result.Ref[0] = -(float)Math.PI * nxAbs;
                        result.Ref[1] = (float)Math.PI * nyAbs;
                        result.Ref[2] = (float)Math.PI * nzAbs;
                    }
                }

                return result;
            }

        }

        public float[] HomArray16
        {
            get
            {
                float[] result = new float[16];

                result[0] = matrix[0, 0];
                result[1] = matrix[1, 0];
                result[2] = matrix[2, 0];

                result[4] = matrix[0, 1];
                result[5] = matrix[1, 1];
                result[6] = matrix[2, 1];

                result[8] = matrix[0, 2];
                result[9] = matrix[1, 2];
                result[10] = matrix[2, 2];

                result[15] = 1;

                return result;
            }
        }

    }

    public class BlockMatrix
    {
        private Matrix[,] matrix;

        public int[] Rows { get; private set; }

        public int[] Columns { get; private set; }

        public BlockMatrix(int[] rows, int[] columns)
        {
            matrix = new Matrix[rows.Length, columns.Length];

            for (int I = 0; I < rows.Length; I++)
            {
                for (int J = 0; J < columns.Length; J++)
                {
                    matrix[I, J] = new Matrix(rows[I], columns[J]);
                }
            }

            Rows = rows;
            Columns = columns;
        }

        private static int Sum(int[] ints)
        {
            int result = 0;
            for (int i = 0; i < ints.Length; i++) { result += ints[i]; }
            return result;
        }

        public int GetLength(int i) { return matrix.GetLength(i); }

        public Matrix this[int I, int J]
        {
            get
            {
                if (I < GetLength(0) && J < GetLength(1)) return matrix[I, J];
                else
                {
                    Console.WriteLine("Error : i or j of BlockMatrix[i, j] is incorrect.");
                    return new Matrix(0, 0);
                }
            }
            set
            {
                if (I < GetLength(0) && J < GetLength(1)) matrix[I, J] = value;
                else Console.WriteLine("Error : i or j of BlockMatrix[i, j] is incorrect.");
            }
        }

        public ref Matrix[,] Ref { get { return ref matrix; } }

        public void SetDiagonal(float diag)
        {
            for (int I = 0; I < GetLength(0); I++)
            {
                for (int J = 0; J < GetLength(1); J++)
                {
                    if (I == J) matrix[I, J].SetDiagonal(diag);
                    else matrix[I, J].SetZeroMatrix(); ;
                }
            }
        }

        public void SetDiagonal(BlockVector v)
        {
            int K = 0;

            for (int I = 0; I < GetLength(0); I++)
            {
                for (int J = 0; J < GetLength(1); J++)
                {
                    if (K < v.Length)
                    {
                        if (I == J)
                        {
                            matrix[I, J].SetDiagonal(v.Ref[K].Get);
                            K++;
                        }
                        else matrix[I, J].SetZeroMatrix();
                    }
                    else
                    {
                        return;
                    }
                }
            }
        }

        public Matrix Matrix
        {
            get
            {
                Matrix result = new Matrix(Sum(Rows), Sum(Columns));

                int row = 0, col = 0;

                for (int I = 0; I < GetLength(0); I++)
                {
                    for (int J = 0; J < GetLength(1); J++)
                    {
                        for (int i = 0; i < matrix[I, J].GetLength(0); i++)
                        {
                            for (int j = 0; j < matrix[I, J].GetLength(1); j++)
                            {
                                result.Ref[row + i, col + j] = matrix[I, J].Ref[i, j];
                            }
                        }
                        col += matrix[0, J].GetLength(1);
                    }
                    row += matrix[I, 0].GetLength(0);
                    col = 0;
                }

                return result;
            }
            set
            {
                if (value.GetLength(0) == Sum(Rows) && value.GetLength(1) == Sum(Columns))
                {
                    int row = 0, col = 0;

                    for (int I = 0; I < GetLength(0); I++)
                    {
                        for (int J = 0; J < GetLength(1); J++)
                        {
                            for (int i = 0; i < matrix[I, J].GetLength(0); i++)
                            {
                                for (int j = 0; j < matrix[I, J].GetLength(1); j++)
                                {
                                    matrix[I, J].Ref[i, j] = value.Ref[row + i, col + j];
                                }
                            }
                            col += matrix[0, J].GetLength(1);
                        }
                        row += matrix[I, 0].GetLength(0);
                        col = 0;
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }
            }
        }

        public void ConsoleWrite(byte digit = 2) { Matrix.ConsoleWrite(digit); }

        public void FixBlockDiagonal()
        {
            for (int I = 0; I < GetLength(0); I++)
            {
                for (int J = 0; J < GetLength(1); J++)
                {
                    if (I != J) matrix[I, J].FixZeroMatrix();
                }
            }
        }

        public void FixBlockLowerTriangle()
        {
            for (int I = 0; I < GetLength(0); I++)
            {
                for (int J = 0; J < GetLength(1); J++)
                {
                    if (I < J) matrix[I, J].FixZeroMatrix();
                }
            }
        }

        public void FixBlockUpperTriangle()
        {
            for (int I = 0; I < GetLength(0); I++)
            {
                for (int J = 0; J < GetLength(1); J++)
                {
                    if (I > J) matrix[I, J].FixZeroMatrix();
                }
            }
        }

        public void SetIdentity()
        {
            for (int I = 0; I < GetLength(0); I++)
            {
                for (int J = 0; J < GetLength(1); J++)
                {
                    if (I == J) matrix[I, J].SetIdentity();
                    else matrix[I, J].SetZeroMatrix();
                }
            }
        }

        public BlockMatrix Trans
        {
            get
            {
                BlockMatrix result = new BlockMatrix(Columns, Rows);

                for (int I = 0; I < GetLength(1); I++)
                {
                    for (int J = 0; J < GetLength(0); J++)
                    {
                        result.Ref[I, J] = matrix[J, I].Trans;
                    }
                }

                return result;
            }
        }

        public static BlockMatrix operator *(BlockMatrix m1, BlockMatrix m2)
        {
            BlockMatrix result = new BlockMatrix(m1.Rows, m2.Columns);

            if (m1.GetLength(1) == m2.GetLength(0))
            {
                for (int I = 0; I < m1.GetLength(0); I++)
                {
                    for (int J = 0; J < m2.GetLength(1); J++)
                    {
                        for (int K = 0; K < m1.GetLength(1); K++)
                        {
                            if (!m1.Ref[I, K].IsFixedZero && !m2.Ref[K, J].IsFixedZero)
                            {
                                result.Ref[I, J] += (m1.Ref[I, K] * m2.Ref[K, J]);
                            }
                        }
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static BlockVector operator *(BlockMatrix m, BlockVector v)
        {
            BlockVector result = new BlockVector(m.Rows);

            if (m.GetLength(1) == v.Length)
            {
                for (int I = 0; I < m.GetLength(0); I++)
                {
                    for (int J = 0; J < v.Length; J++)
                    {
                        if (!m.Ref[I, J].IsFixedZero)
                        {
                            result.Ref[I] += (m.Ref[I, J] * v.Ref[J]);
                        }
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static BlockMatrix operator *(float s, BlockMatrix m)
        {
            BlockMatrix result = new BlockMatrix(m.Rows, m.Columns);

            for (int I = 0; I < m.GetLength(0); I++)
            {
                for (int J = 0; J < m.GetLength(1); J++)
                {
                    result.Ref[I, J] = s * m.Ref[I, J];
                }
            }

            return result;
        }

        public static BlockMatrix operator *(BlockMatrix m, float s) { return s * m; }

        public static BlockMatrix operator +(BlockMatrix m1, BlockMatrix m2)
        {
            BlockMatrix result = new BlockMatrix(m1.Rows, m1.Columns);

            if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1))
            {
                for (int I = 0; I < m1.GetLength(0); I++)
                {
                    for (int J = 0; J < m1.GetLength(1); J++)
                    {
                        result.Ref[I, J] = m1.Ref[I, J] + m2.Ref[I, J];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static BlockMatrix operator -(BlockMatrix m1, BlockMatrix m2)
        {
            BlockMatrix result = new BlockMatrix(m1.Rows, m1.Columns);

            if (m1.GetLength(0) == m2.GetLength(0) && m1.GetLength(1) == m2.GetLength(1))
            {
                for (int I = 0; I < m1.GetLength(0); I++)
                {
                    for (int J = 0; J < m1.GetLength(1); J++)
                    {
                        result.Ref[I, J] = m1.Ref[I, J] - m2.Ref[I, J];
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public BlockMatrix Inv
        {
            get
            {
                BlockMatrix result = new BlockMatrix(Rows, Columns);
                result.Matrix = Matrix.Inv;
                return result;
            }
        }

        public float Trace
        {
            get
            {
                float result = 0;

                for (int I = 0; I < GetLength(0) && I < GetLength(1); I++)
                {
                    result += matrix[I, I].Trace;
                }

                return result;
            }
        }

        private static void ConsoleWriteErrorMessage()
        {
            Console.WriteLine("Size of Block Matrix do Not match.");
        }

    }

    public class Vector : AlgebraBase.VectorBase
    {
        public Vector(int n) : base(n) { }

        public void SetValue(params float[] values)
        {
            for (int i = 0; i < Length; i++)
            {
                if (i < values.Length) vector[i] = values[i];
                else break;
            }
        }

        public float SquareSum
        {
            get
            {
                float result = 0;
                for (int i = 0; i < Length; i++) { result += vector[i] * vector[i]; }
                return result;
            }
        }

        public float Norm { get { return (float)Math.Sqrt(SquareSum); } }

        public Vector Abs
        {
            get
            {
                Vector result = new Vector(Length);
                for (int i = 0; i < Length; i++) { result.Ref[i] = Math.Abs(vector[i]); }
                return result;
            }
        }

        public float Sum
        {
            get
            {
                float result = 0;
                for (int i = 0; i < Length; i++) { result += vector[i]; }
                return result;
            }
        }

        public float AbsSum
        {
            get
            {
                float result = 0;
                for (int i = 0; i < Length; i++) { result += Math.Abs(vector[i]); }
                return result;
            }
        }

        public void SetLimit(float min, float max)
        {
            for (int i = 0; i < Length; i++)
            {
                if (vector[i] < min) vector[i] = min;
                else if (max < vector[i]) vector[i] = max;
            }
        }

        public void SetUnitVectorX(float alpha = 1)
        {
            SetZeroVector();
            vector[0] = alpha;
        }

        public void SetUnitVectorY(float alpha = 1)
        {
            SetZeroVector();
            vector[1] = alpha;
        }

        public void SetUnitVectorZ(float alpha = 1)
        {
            SetZeroVector();
            vector[2] = alpha;
        }

        public void SetUnitVector(int n, float alpha = 1)
        {
            SetZeroVector();
            vector[n] = alpha;
        }

        private static Random random = new Random();

        public void SetRandom(int min = -10, int max = 10)
        {
            for (int i = 0; i < Length; i++) { vector[i] = random.Next(min, max); }
        }

        public void Follow(Vector v) { vector = v.Ref; }

        public void QuitFollow() { vector = Get; }

        public static implicit operator Vector(float[] v)
        {
            Vector result = new Vector(v.Length);
            result.Ref = v;
            return result;
        }

        public float[] Plus(params float[] v)
        {
            float[] result = new float[vector.Length];
            if (Length == v.Length) for (int i = 0; i < vector.Length; i++) { result[i] = vector[i] + v[i]; }
            else ConsoleWriteErrorMessage();
            return result;
        }

        public float[] Plus(Vector v) { return Plus(v.Get); }

        private static Vector Plus(float[] v1, float[] v2)
        {
            Vector result = new Vector(v1.Length);
            if (v1.Length == v2.Length) for (int i = 0; i < v1.Length; i++) { result.Ref[i] = v1[i] + v2[i]; }
            else ConsoleWriteErrorMessage();
            return result;
        }

        public static Vector operator +(Vector v1, Vector v2) { return Plus(v1.Get, v2.Get); }

        public static Vector operator +(Vector v1, float[] v2) { return Plus(v1.Get, v2); }

        public static Vector operator +(float[] v1, Vector v2) { return Plus(v1, v2.Get); }

        private static Vector Minus(float[] v1, float[] v2)
        {
            Vector result = new Vector(v1.Length);
            if (v1.Length == v2.Length) for (int i = 0; i < v1.Length; i++) { result.Ref[i] = v1[i] - v2[i]; }
            else ConsoleWriteErrorMessage();
            return result;
        }

        public static Vector operator -(Vector v1, Vector v2) { return Minus(v1.Get, v2.Get); }

        public static Vector operator -(Vector v1, float[] v2) { return Minus(v1.Get, v2); }

        public static Vector operator -(float[] v1, Vector v2) { return Minus(v1, v2.Get); }

        private static Vector Add(float s, float[] v)
        {
            Vector result = new Vector(v.Length);
            for (int i = 0; i < v.Length; i++) { result.Ref[i] = s + v[i]; }
            return result;
        }

        public static Vector operator ++(Vector v) { return Add(1, v.Get); }

        public static Vector operator --(Vector v) { return Add(-1, v.Get); }

        public static Vector operator -(Vector v)
        {
            Vector result = new Vector(v.Length);
            for (int i = 0; i < v.Length; i++) { result.Ref[i] = -v.Ref[i]; }
            return result;
        }

        public static Vector operator *(float s, Vector v)
        {
            Vector result = new Vector(v.Length);
            for (int i = 0; i < v.Length; i++) { result.Ref[i] = s * v.Ref[i]; }
            return result;
        }

        public static Vector operator *(Vector v, float s) { return s * v; }

        public class VectorT : AlgebraBase.VectorBase
        {
            public VectorT(int n) : base(n) { }

            public static implicit operator VectorT(float[] v)
            {
                VectorT result = new VectorT(v.Length);
                result.Ref = v;
                return result;
            }

            private static float DotProduct(float[] v1, float[] v2)
            {
                float result = 0;
                if (v1.Length == v2.Length) for (int i = 0; i < v1.Length; i++) { result += v1[i] * v2[i]; }
                else ConsoleWriteErrorMessage();
                return result;
            }

            public static float operator *(VectorT v1, Vector v2) { return DotProduct(v1.Get, v2.Get); }

            public static float operator *(VectorT v1, float[] v2) { return DotProduct(v1.Get, v2); }

            public static VectorT operator *(float s, VectorT v)
            {
                VectorT result = new VectorT(v.Length);
                for (int i = 0; i < v.Length; i++) { result.Ref[i] = s * v.Ref[i]; }
                return result;
            }

            private static float[,] OuterProduct(float[] v1, VectorT v2)
            {
                float[,] result = new float[v1.Length, v2.Length];

                for (int i = 0; i < v1.Length; i++)
                {
                    for (int j = 0; j < v2.Length; j++)
                    {
                        result[i, j] = v1[i] * v2.Ref[j];
                    }
                }

                return result;
            }

            public static float[,] operator *(Vector v1, VectorT v2) { return OuterProduct(v1.Get, v2); }

            public static float[,] operator *(float[] v1, VectorT v2) { return OuterProduct(v1, v2); }

            private static VectorT Product(VectorT v, float[,] m)
            {
                VectorT result = new VectorT(m.GetLength(1));

                if (v.Length == m.GetLength(0))
                {
                    for (int i = 0; i < m.GetLength(1); i++)
                    {
                        for (int j = 0; j < m.GetLength(0); j++)
                        {
                            result.Ref[i] += v.Ref[j] * m[j, i];
                        }
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }

                return result;
            }

            public static VectorT operator *(VectorT v, Matrix m) { return Product(v, m.Get); }

            public static VectorT operator *(VectorT v, float[,] m) { return Product(v, m); }

            public static VectorT operator +(VectorT v1, VectorT v2)
            {
                VectorT result = new VectorT(v1.Length);
                if (v1.Length == v2.Length) for (int i = 0; i < v1.Length; i++) { result.Ref[i] = v1.Ref[i] + v2.Ref[i]; }
                else ConsoleWriteErrorMessage();
                return result;
            }

            public static VectorT operator -(VectorT v1, VectorT v2)
            {
                VectorT result = new VectorT(v1.Length);
                if (v1.Length == v2.Length) for (int i = 0; i < v1.Length; i++) { result.Ref[i] = v1.Ref[i] - v2.Ref[i]; }
                else ConsoleWriteErrorMessage();
                return result;
            }

        }

        public VectorT Trans
        {
            get
            {
                VectorT result = new VectorT(this.Length);
                result.Ref = Get;
                return result;
            }
            set
            {
                Ref = value.Get;
            }
        }

        private static Vector CrossProduct(float[] v1, float[] v2)
        {
            Vector result = new Vector(3);

            if (v1.Length == 3 && v2.Length == 3)
            {
                result.Ref[0] = v1[1] * v2[2] - v1[2] * v2[1];
                result.Ref[1] = v1[2] * v2[0] - v1[0] * v2[2];
                result.Ref[2] = v1[0] * v2[1] - v1[1] * v2[0];
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static Vector operator *(Vector v1, Vector v2) { return CrossProduct(v1.Get, v2.Get); }

        public static Vector operator *(float[] v1, Vector v2) { return CrossProduct(v1, v2.Get); }

        public static Vector operator *(Vector v1, float[] v2) { return CrossProduct(v1.Get, v2); }

        private static Vector HadamardProduct(float[] v1, float[] v2)
        {
            Vector result = new Vector(v1.Length);
            if (v1.Length == v2.Length) for (int i = 0; i < v1.Length; i++) { result.Ref[i] = v1[i] * v2[i]; }
            else ConsoleWriteErrorMessage();
            return result;
        }

        public static Vector operator ^(Vector v1, Vector v2) { return HadamardProduct(v1.Get, v2.Get); }

        public static Vector operator ^(float[] v1, Vector v2) { return HadamardProduct(v1, v2.Get); }

        public static Vector operator ^(Vector v1, float[] v2) { return HadamardProduct(v1.Get, v2); }

        private static Vector DirectSum(float[] v1, float[] v2)
        {
            Vector result = new Vector(v1.Length + v2.Length);
            for (int i = 0; i < v1.Length; i++) { result.Ref[i] = v1[i]; }
            for (int i = 0; i < v2.Length; i++) { result.Ref[i + v1.Length] = v2[i]; }
            return result;
        }

        public static Vector operator &(Vector v1, Vector v2) { return DirectSum(v1.Get, v2.Get); }

        public static Vector operator &(float[] v1, Vector v2) { return DirectSum(v1, v2.Get); }

        public static Vector operator &(Vector v1, float[] v2) { return DirectSum(v1.Get, v2); }

        public static bool operator <(float s, Vector v)
        {
            bool result = true;
            for (int i = 0; i < v.Length; i++) { if (v.Ref[i] < s) { result = false; break; } }
            return result;
        }

        public static bool operator >(float s, Vector v)
        {
            bool result = true;
            for (int i = 0; i < v.Length; i++) { if (v.Ref[i] > s) { result = false; break; } }
            return result;
        }

        public static bool operator <(Vector v, float s)
        {
            bool result = true;
            for (int i = 0; i < v.Length; i++) { if (s < v.Ref[i]) { result = false; break; } }
            return result;
        }

        public static bool operator >(Vector v, float s)
        {
            bool result = true;
            for (int i = 0; i < v.Length; i++) { if (s > v.Ref[i]) { result = false; break; } }
            return result;
        }

        public float Distance(float[] v)
        {
            float result = 0;

            if (this.Length == v.Length)
            {
                for (int i = 0; i < this.Length; i++)
                {
                    result += (vector[i] - v[i]) * (vector[i] - v[i]);
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return (float)Math.Sqrt(result);
        }

        public float Distance(Vector v) { return Distance(v.Get); }

        public void MoveTo(Vector v, float distance)
        {
            if (this.Length == v.Length)
            {
                float d = Distance(v);
                if (d <= distance) Set = v.Get;
                else Set = (vector + (distance / d) * (v - vector)).Get;
            }
            else
            {
                ConsoleWriteErrorMessage();
            }
        }

        public Vector Normalize
        {
            get
            {
                Vector result = new Vector(this.Length);
                float norm = Norm;

                if (0.0001f < norm)
                {
                    for (int i = 0; i < this.Length; i++)
                    {
                        result.Ref[i] = vector[i] / norm;
                    }
                }
                else
                {
                    result.Ref[0] = 1;
                }

                return result;
            }
        }

        public float FormedAngle(Vector v)
        {
            float result;
            float dotProduct = Trans * v;
            float normProduct = this.Norm * v.Norm;

            if (0.0001f < normProduct)
            {
                float d = dotProduct / normProduct;

                if (1 <= d) result = 0;
                else if (d <= -1) result = (float)Math.PI;
                else result = (float)Math.Acos(d);
            }
            else
            {
                result = 0;
            }

            return result;
        }

        public float[,] ConvertRotationMatrix
        {
            get
            {
                float[,] result = new float[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
                float norm = Norm;

                if (0.0001f < norm && 2 < Length)
                {
                    float normXY = (float)Math.Sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
                    float[] n = new float[3];

                    if (0.0001f < normXY)
                    {
                        n[0] = -vector[1] / normXY;
                        n[1] = vector[0] / normXY;
                        n[2] = 0;
                    }
                    else
                    {
                        if (0 < vector[2]) return result;
                        else n = new float[3] { 0, 1, 0 };
                    }

                    float Sth = normXY / norm;
                    float Cth = vector[2] / norm;

                    result[0, 0] = Cth + n[0] * n[0] * (1 - Cth);
                    result[0, 1] = n[0] * n[1] * (1 - Cth) - n[2] * Sth;
                    result[0, 2] = n[0] * n[2] * (1 - Cth) + n[1] * Sth;

                    result[1, 0] = n[0] * n[1] * (1 - Cth) + n[2] * Sth;
                    result[1, 1] = Cth + n[1] * n[1] * (1 - Cth);
                    result[1, 2] = n[1] * n[2] * (1 - Cth) - n[0] * Sth;

                    result[2, 0] = n[0] * n[2] * (1 - Cth) - n[1] * Sth;
                    result[2, 1] = n[1] * n[2] * (1 - Cth) + n[0] * Sth;
                    result[2, 2] = Cth + n[2] * n[2] * (1 - Cth);

                    return result;
                }
                else
                {
                    return result;
                }
            }
        }

        public float[] HomArray16
        {
            get
            {
                float[] result = new float[16];
                float[,] matrix = ConvertRotationMatrix;

                result[0] = matrix[0, 0];
                result[1] = matrix[1, 0];
                result[2] = matrix[2, 0];

                result[4] = matrix[0, 1];
                result[5] = matrix[1, 1];
                result[6] = matrix[2, 1];

                result[8] = matrix[0, 2];
                result[9] = matrix[1, 2];
                result[10] = matrix[2, 2];

                result[15] = 1;

                return result;
            }
        }
    }

    public class BlockVector : AlgebraBase.BlockVectorBase
    {
        public BlockVector(params int[] rows) : base(rows) { }

        public float SquareSum
        {
            get
            {
                float result = 0;
                for (int I = 0; I < Length; I++) { result += vector[I].SquareSum; }
                return result;
            }
        }

        public float Norm { get { return (float)Math.Sqrt(SquareSum); } }

        public BlockVector Abs
        {
            get
            {
                BlockVector result = new BlockVector(Rows);

                for (int I = 0; I < Length; I++)
                {
                    result.Ref[I] = new Vector(vector[I].Length);
                    result.Ref[I] = vector[I].Abs;
                }

                return result;
            }
        }

        public float Sum
        {
            get
            {
                float result = 0;
                for (int I = 0; I < Length; I++) { result += vector[I].Sum; }
                return result;
            }
        }

        public void SetLimit(float min, float max)
        {
            for (int I = 0; I < Length; I++)
            {
                vector[I].SetLimit(min, max);
            }
        }

        public void SetRandom(int min = -10, int max = 10)
        {
            for (int I = 0; I < Length; I++)
            {
                vector[I].SetRandom(min, max);
            }
        }

        private int RowsSum
        {
            get
            {
                int result = 0;
                for (int i = 0; i < Rows.Length; i++) { result += Rows[i]; }
                return result;
            }
        }

        public Vector Vector
        {
            get
            {
                Vector result = new Vector(RowsSum);

                int row = 0;

                for (int I = 0; I < Length; I++)
                {
                    for (int i = 0; i < vector[I].Length; i++)
                    {
                        result.Ref[row + i] = vector[I].Ref[i];
                    }

                    row += vector[I].Length;
                }

                return result;
            }
            set
            {
                if (RowsSum == value.Length)
                {
                    int row = 0;

                    for (int I = 0; I < Length; I++)
                    {
                        for (int i = 0; i < vector[I].Length; i++)
                        {
                            vector[I].Ref[i] = value.Ref[row + i];
                        }

                        row += vector[I].Length;
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }
            }
        }

        public void ConsoleWrite(byte digit = 2) { Vector.ConsoleWrite(digit); }

        public static BlockVector operator +(BlockVector v1, BlockVector v2)
        {
            BlockVector result = new BlockVector(v1.Rows);

            if (v1.Length == v2.Length)
            {
                for (int I = 0; I < v1.Length; I++)
                {
                    if (v1.Ref[I].Length == v2.Ref[I].Length)
                    {
                        result.Ref[I] = v1.Ref[I] + v2.Ref[I];
                    }
                    else
                    {
                        ConsoleWriteErrorMessage();
                        break;
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static BlockVector operator -(BlockVector v1, BlockVector v2)
        {
            BlockVector result = new BlockVector(v1.Rows);

            if (v1.Length == v2.Length)
            {
                for (int I = 0; I < v1.Length; I++)
                {
                    if (v1.Ref[I].Length == v2.Ref[I].Length)
                    {
                        result.Ref[I] = v1.Ref[I] - v2.Ref[I];
                    }
                    else
                    {
                        ConsoleWriteErrorMessage();
                        break;
                    }
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public class BlockVectorT : AlgebraBase.BlockVectorBase
        {
            public BlockVectorT(int[] rows) : base(rows) { }

            public static float operator *(BlockVectorT v1, BlockVector v2)
            {
                float result = 0;

                if (v1.Length == v2.Length)
                {
                    for (int I = 0; I < v1.Length; I++)
                    {
                        result += v1.Ref[I].Trans * v2.Ref[I];
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }

                return result;
            }

            public static BlockVectorT operator *(float s, BlockVectorT v)
            {
                BlockVectorT result = new BlockVectorT(v.Rows);
                for (int i = 0; i < v.Length; i++) { result.Ref[i] = s * v.Ref[i]; }
                return result;
            }

            public static BlockVectorT operator *(BlockVectorT v, BlockMatrix m)
            {
                BlockVectorT result = new BlockVectorT(m.Columns);

                if (v.Length == m.GetLength(0))
                {
                    for (int I = 0; I < m.GetLength(1); I++)
                    {
                        for (int J = 0; J < m.GetLength(0); J++)
                        {
                            result.Ref[I].Trans += v.Ref[J].Trans * m.Ref[J, I];
                        }
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }

                return result;
            }

            public static BlockMatrix operator *(BlockVector v1, BlockVectorT v2)
            {
                BlockMatrix result = new BlockMatrix(v1.Rows, v2.Rows);

                for (int I = 0; I < v1.Length; I++)
                {
                    for (int J = 0; J < v2.Length; J++)
                    {
                        result.Ref[I, J] = v1.Ref[I] * v2.Ref[J].Trans;
                    }
                }

                return result;
            }

            public static BlockVectorT operator +(BlockVectorT v1, BlockVectorT v2)
            {
                BlockVectorT result = new BlockVectorT(v1.Rows);

                if (v1.Length == v2.Length)
                {
                    for (int i = 0; i < v1.Length; i++)
                    {
                        result.Ref[i] = v1.Ref[i] + v2.Ref[i];
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }

                return result;
            }

            public static BlockVectorT operator -(BlockVectorT v1, BlockVectorT v2)
            {
                BlockVectorT result = new BlockVectorT(v1.Rows);

                if (v1.Length == v2.Length)
                {
                    for (int i = 0; i < v1.Length; i++)
                    {
                        result.Ref[i] = v1.Ref[i] - v2.Ref[i];
                    }
                }
                else
                {
                    ConsoleWriteErrorMessage();
                }

                return result;
            }

        }

        public BlockVectorT Trans
        {
            get
            {
                BlockVectorT result = new BlockVectorT(Rows);
                result.Set = Get;
                return result;
            }
            set
            {
                Set = value.Get;
            }
        }

        public static BlockVector operator *(BlockVector v1, BlockVector v2)
        {
            BlockVector result = new BlockVector(v1.Rows);

            if (v1.Length == v2.Length)
            {
                for (int I = 0; I < v1.Length; I++)
                {
                    result.Ref[I] = v1.Ref[I] * v2.Ref[I];
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static BlockVector operator *(float s, BlockVector v)
        {
            BlockVector result = new BlockVector(v.Rows);
            for (int I = 0; I < v.Length; I++) { result.Ref[I] = s * v.Ref[I]; }
            return result;
        }

        public static BlockVector operator ^(BlockVector v1, BlockVector v2)
        {
            BlockVector result = new BlockVector(v1.Rows);

            if (v1.Length == v2.Length)
            {
                for (int I = 0; I < v1.Length; I++)
                {
                    result.Ref[I] = v1.Ref[I] ^ v2.Ref[I];
                }
            }
            else
            {
                ConsoleWriteErrorMessage();
            }

            return result;
        }

        public static bool operator <(float s, BlockVector v)
        {
            bool result = true;
            for (int I = 0; I < v.Length; I++) { if (v.Ref[I] < s) { result = false; break; } }
            return result;
        }

        public static bool operator >(float s, BlockVector v)
        {
            bool result = true;
            for (int I = 0; I < v.Length; I++) { if (v.Ref[I] > s) { result = false; break; } }
            return result;
        }

        public static bool operator <(BlockVector v, float s)
        {
            bool result = true;
            for (int I = 0; I < v.Length; I++) { if (s < v.Ref[I]) { result = false; break; } }
            return result;
        }

        public static bool operator >(BlockVector v, float s)
        {
            bool result = true;
            for (int I = 0; I < v.Length; I++) { if (s > v.Ref[I]) { result = false; break; } }
            return result;
        }

    }

    static class Float3
    {
        public static float Distance(float[] v1, float[] v2)
        {
            float[] e = new float[3] { v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2] };
            return (float)Math.Sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        }

        public static float Dot(float[] v1, float[] v2)
        {
            return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
        }

        public static float[] Cross(float[] v1, float[] v2)
        {
            float[] result = new float[3];

            result[0] = v1[1] * v2[2] - v1[2] * v2[1];
            result[1] = v1[2] * v2[0] - v1[0] * v2[2];
            result[2] = v1[0] * v2[1] - v1[1] * v2[0];

            return result;
        }

        public static float Norm(float[] v)
        {
            return (float)Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        }

        public static float[] Plus(float[] v1, float[] v2)
        {
            return new float[3] { v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2] };
        }

        public static float[] Minus(float[] v1, float[] v2)
        {
            return new float[3] { v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2] };
        }

        public static float Sum(float[] v)
        {
            return v[0] + v[1] + v[2];
        }

        public static float AbsSum(float[] v)
        {
            return Math.Abs(v[0]) + Math.Abs(v[1]) + Math.Abs(v[2]);
        }

        public static float[] Times(float a, float[] v)
        {
            return new float[3] { a * v[0], a * v[1], a * v[2] };
        }

    }

}
