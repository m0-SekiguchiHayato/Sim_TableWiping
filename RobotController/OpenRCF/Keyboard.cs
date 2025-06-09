


// hayato
// 一部のキーを, 関数実行用からbool変数格納用に変更



using System;
using System.Collections.Generic;
using System.Windows.Input;

namespace OpenRCF
{
    public static class Keyboard
    {
        //public static Action KeyEventA { set { KeyEvent[Key.A] = value; } }
        public static bool KeyEventA { get { return BoolKey[Key.A]; } }     // hayato
        public static Action KeyEventB { set { KeyEvent[Key.B] = value; } }
        public static Action KeyEventC { set { KeyEvent[Key.C] = value; } }
        //public static Action KeyEventD { set { KeyEvent[Key.D] = value; } }
        public static bool KeyEventD { get { return BoolKey[Key.D]; } }     // hayato
        //public static Action KeyEventE { set { KeyEvent[Key.E] = value; } }
        public static bool KeyEventE { get { return BoolKey[Key.E]; } }     // hayato
        public static Action KeyEventF { set { KeyEvent[Key.F] = value; } }
        public static Action KeyEventG { set { KeyEvent[Key.G] = value; } }
        public static Action KeyEventH { set { KeyEvent[Key.H] = value; } }
        //public static Action KeyEventI { set { KeyEvent[Key.I] = value; } }
        public static bool KeyEventI { get { return BoolKey[Key.I]; } }     // hayato
        //public static Action KeyEventJ { set { KeyEvent[Key.J] = value; } }
        public static bool KeyEventJ { get { return BoolKey[Key.J]; } }     // hayato
        //public static Action KeyEventK { set { KeyEvent[Key.K] = value; } }
        public static bool KeyEventK { get { return BoolKey[Key.K]; } }     // hayato
        //public static Action KeyEventL { set { KeyEvent[Key.L] = value; } }
        public static bool KeyEventL { get { return BoolKey[Key.L]; } }     // hayato
        public static Action KeyEventM { set { KeyEvent[Key.M] = value; } }
        public static Action KeyEventN { set { KeyEvent[Key.N] = value; } }
        //public static Action KeyEventO { set { KeyEvent[Key.O] = value; } }
        public static bool KeyEventO { get { return BoolKey[Key.O]; } }     // hayato
        public static Action KeyEventP { set { KeyEvent[Key.P] = value; } }
        //public static Action KeyEventQ { set { KeyEvent[Key.Q] = value; } }
        public static bool KeyEventQ { get { return BoolKey[Key.Q]; } }     // hayato
        public static Action KeyEventR { set { KeyEvent[Key.R] = value; } }
        //public static Action KeyEventS { set { KeyEvent[Key.S] = value; } }
        public static bool KeyEventS { get { return BoolKey[Key.S]; } }     // hayato
        public static Action KeyEventT { set { KeyEvent[Key.T] = value; } }
        //public static Action KeyEventU { set { KeyEvent[Key.U] = value; } }
        public static bool KeyEventU { get { return BoolKey[Key.U]; } }     // hayato
        public static Action KeyEventV { set { KeyEvent[Key.V] = value; } }
        //public static Action KeyEventW { set { KeyEvent[Key.W] = value; } }
        public static bool KeyEventW { get { return BoolKey[Key.W]; } }     // hayato
        public static Action KeyEventX { set { KeyEvent[Key.X] = value; } }
        public static Action KeyEventY { set { KeyEvent[Key.Y] = value; } }
        public static Action KeyEventZ { set { KeyEvent[Key.Z] = value; } }
        public static Action KeyEventDelete { set { KeyEvent[Key.Delete] = value; } }
        public static Action KeyEventBack { set { KeyEvent[Key.Back] = value; } }
        public static bool IsSpaceDowned { get { return ModifierKey[Key.Space]; } }

        private static Dictionary<Key, Action> KeyEvent = new Dictionary<Key, Action>()
        {
            //{ Key.A, () => { } },
            { Key.B, () => { } },
            { Key.C, () => { } },
            //{ Key.D, () => { } },
            //{ Key.E, () => { } },
            { Key.F, () => { } },
            { Key.G, () => { } },
            { Key.H, () => { } },
            //{ Key.I, () => { } },
            //{ Key.J, () => { } },
            //{ Key.K, () => { } },
            //{ Key.L, () => { } },
            { Key.M, () => { } },
            { Key.N, () => { } },
            //{ Key.O, () => { } },
            { Key.P, () => { } },
            //{ Key.Q, () => { } },
            { Key.R, () => { } },
            //{ Key.S, () => { } },
            { Key.T, () => { } },
            //{ Key.U, () => { } },
            { Key.V, () => { } },
            //{ Key.W, () => { } },
            { Key.X, () => { } },
            { Key.Y, () => { } },
            { Key.Z, () => { } },
            { Key.Delete, () => { } },
            { Key.Back, () => { } },
        };


        private static Dictionary<Key, bool> BoolKey = new Dictionary<Key, bool>()     // hayato
        {
            { Key.A, false },
            { Key.D, false },
            { Key.E, false },
            { Key.I, false },
            { Key.J, false },
            { Key.K, false },
            { Key.L, false },
            { Key.O, false },
            { Key.Q, false },
            { Key.S, false },
            { Key.U, false },
            { Key.W, false },
        };


        private static Dictionary<Key, bool> ModifierKey = new Dictionary<Key, bool>()
        {
            { Key.LeftShift, false },
            { Key.RightShift, false },
            { Key.LeftCtrl, false },
            { Key.RightCtrl, false },
            { Key.Space, false },
        };

        private static Key digitKeyOld = Key.D0;
        private static Dictionary<Key, bool> DigitKey = new Dictionary<Key, bool>()
        {
            { Key.D0, true },
            { Key.D1, false },
            { Key.D2, false },
            { Key.D3, false },
            { Key.D4, false },
            { Key.D5, false },
            { Key.D6, false },
            { Key.D7, false },
            { Key.D8, false },
            { Key.D9, false },
        };

        private readonly static Dictionary<Key, int[]> ArrowKey = new Dictionary<Key, int[]>()
        {
            { Key.Right, new int[] { 1, 0, 0 } },
            { Key.Left, new int[] { -1, 0, 0 } },
            { Key.Up, new int[] { 0, 1, 0 } },
            { Key.Down, new int[] { 0, -1, 0 } },
            { Key.OemPeriod, new int[] { 0, 0, 1 } },
            { Key.OemComma, new int[] { 0, 0, -1 } },
            { Key.OemBackslash, new int[] { 0, 0, 1 } },
            { Key.OemQuestion, new int[] { 0, 0, -1 } },
            { Key.Delete, new int[] { 0, 0, 0 } },
        };

        public static Vector[] ShiftVector = new Vector[10];
        public static RotationMatrix[] CtrlMatrix = new RotationMatrix[10];
        public static Vector SpaceVector = new Vector(3);

        static Keyboard()
        {
            for (int i = 0; i < ShiftVector.Length; i++)
            {
                ShiftVector[i] = new Vector(3);
            }

            for (int i = 0; i < CtrlMatrix.Length; i++)
            {
                CtrlMatrix[i] = new RotationMatrix();
            }
        }

        public static float ShiftDistance = 0.05f;

        private static float[] vector = new float[3];
        private static void UpdateShiftVector(int[] arrowVector, int mashNum)
        {
            if (mashNum < 7)
            {
                vector[0] = ShiftDistance * arrowVector[0];
                vector[1] = ShiftDistance * arrowVector[1];
                vector[2] = ShiftDistance * arrowVector[2];
            }
            else
            {
                vector[0] = 3 * ShiftDistance * arrowVector[0];
                vector[1] = 3 * ShiftDistance * arrowVector[1];
                vector[2] = 3 * ShiftDistance * arrowVector[2];
            }

            if (DigitKey[Key.D0]) ShiftVector[0].Set = ShiftVector[0].Plus(vector);
            else if (DigitKey[Key.D1]) ShiftVector[1].Set = ShiftVector[1].Plus(vector);
            else if (DigitKey[Key.D2]) ShiftVector[2].Set = ShiftVector[2].Plus(vector);
            else if (DigitKey[Key.D3]) ShiftVector[3].Set = ShiftVector[3].Plus(vector);
            else if (DigitKey[Key.D4]) ShiftVector[4].Set = ShiftVector[4].Plus(vector);
            else if (DigitKey[Key.D5]) ShiftVector[5].Set = ShiftVector[5].Plus(vector);
            else if (DigitKey[Key.D6]) ShiftVector[6].Set = ShiftVector[6].Plus(vector);
            else if (DigitKey[Key.D7]) ShiftVector[7].Set = ShiftVector[7].Plus(vector);
            else if (DigitKey[Key.D8]) ShiftVector[8].Set = ShiftVector[8].Plus(vector);
            else if (DigitKey[Key.D9]) ShiftVector[9].Set = ShiftVector[9].Plus(vector);
        }

        public static float SpaceDistance = 0.2f;

        private static void UpdateSpaceVector(int[] arrowVector)
        {
            vector[0] = SpaceDistance * arrowVector[0];
            vector[1] = SpaceDistance * arrowVector[1];
            vector[2] = SpaceDistance * arrowVector[2];
            SpaceVector.Set = SpaceVector.Plus(vector);
        }

        public static float CtrlAngle = (float)Math.PI / 12;

        private static RotationMatrix R = new RotationMatrix();
        private static void UpdateCtrlMatrix(int[] arrowVector)
        {
            if (arrowVector[0] + arrowVector[1] + arrowVector[2] == 0)
            {
                if (DigitKey[Key.D0]) CtrlMatrix[0].SetIdentity();
                else if (DigitKey[Key.D1]) CtrlMatrix[1].SetIdentity();
                else if (DigitKey[Key.D2]) CtrlMatrix[2].SetIdentity();
                else if (DigitKey[Key.D3]) CtrlMatrix[3].SetIdentity();
                else if (DigitKey[Key.D4]) CtrlMatrix[4].SetIdentity();
                else if (DigitKey[Key.D5]) CtrlMatrix[5].SetIdentity();
                else if (DigitKey[Key.D6]) CtrlMatrix[6].SetIdentity();
                else if (DigitKey[Key.D7]) CtrlMatrix[7].SetIdentity();
                else if (DigitKey[Key.D8]) CtrlMatrix[8].SetIdentity();
                else if (DigitKey[Key.D9]) CtrlMatrix[9].SetIdentity();
            }
            else
            {
                float[] axis = new float[3] { -arrowVector[1], arrowVector[0], arrowVector[2] };
                R.SetRn(CtrlAngle, axis);

                if (DigitKey[Key.D0]) CtrlMatrix[0].Set = R.Times(CtrlMatrix[0]);
                else if (DigitKey[Key.D1]) CtrlMatrix[1].Set = R.Times(CtrlMatrix[1]);
                else if (DigitKey[Key.D2]) CtrlMatrix[2].Set = R.Times(CtrlMatrix[2]);
                else if (DigitKey[Key.D3]) CtrlMatrix[3].Set = R.Times(CtrlMatrix[3]);
                else if (DigitKey[Key.D4]) CtrlMatrix[4].Set = R.Times(CtrlMatrix[4]);
                else if (DigitKey[Key.D5]) CtrlMatrix[5].Set = R.Times(CtrlMatrix[5]);
                else if (DigitKey[Key.D6]) CtrlMatrix[6].Set = R.Times(CtrlMatrix[6]);
                else if (DigitKey[Key.D7]) CtrlMatrix[7].Set = R.Times(CtrlMatrix[7]);
                else if (DigitKey[Key.D8]) CtrlMatrix[8].Set = R.Times(CtrlMatrix[8]);
                else if (DigitKey[Key.D9]) CtrlMatrix[9].Set = R.Times(CtrlMatrix[9]);
            }
        }

        private static int mashCounter = 0;
        private static Key KeyOld;
        internal static void KeyDownEvent(object sender, KeyEventArgs e)
        {
            if (e.Key == KeyOld) mashCounter++;
            else mashCounter = 1;

            KeyOld = e.Key;

            if (ModifierKey[Key.LeftShift] || ModifierKey[Key.RightShift])
            {
                if (ArrowKey.ContainsKey(e.Key))
                {
                    UpdateShiftVector(ArrowKey[e.Key], mashCounter);
                }
            }
            else if (ModifierKey[Key.LeftCtrl] || ModifierKey[Key.RightCtrl])
            {
                if (ArrowKey.ContainsKey(e.Key))
                {
                    UpdateCtrlMatrix(ArrowKey[e.Key]);
                }
            }
            else if (ModifierKey[Key.Space])
            {
                if (ArrowKey.ContainsKey(e.Key))
                {
                    UpdateSpaceVector(ArrowKey[e.Key]);
                }
            }
            else if (ModifierKey.ContainsKey(e.Key))
            {
                ModifierKey[e.Key] = true;
            }
            else if (DigitKey.ContainsKey(e.Key))
            {
                DigitKey[digitKeyOld] = false;
                DigitKey[e.Key] = true;
                digitKeyOld = e.Key;
            }
            else if (KeyEvent.ContainsKey(e.Key))
            {
                KeyEvent[e.Key]();
            }
            else if (BoolKey.ContainsKey(e.Key))    // hayato
            {
                BoolKey[e.Key] = true;
            }
        }

        internal static void KeyUpEvent(object sender, KeyEventArgs e)
        {
            if (ModifierKey.ContainsKey(e.Key))
            {
                ModifierKey[e.Key] = false;
            }
            else if (BoolKey.ContainsKey(e.Key))    // hayato
            {
                BoolKey[e.Key] = false;
            }
        }

    }

}
