using OpenTK.Input;
using System;

namespace OpenRCF
{
    public class GamePad
    {
        private GamePadState State = new GamePadState();

        public bool A { get { return State.Buttons.A.Equals(ButtonState.Pressed); } }
        public bool B { get { return State.Buttons.B.Equals(ButtonState.Pressed); } }
        public bool X { get { return State.Buttons.X.Equals(ButtonState.Pressed); } }
        public bool Y { get { return State.Buttons.Y.Equals(ButtonState.Pressed); } }
        public bool R { get { return State.Buttons.RightShoulder.Equals(ButtonState.Pressed); } }
        public bool L { get { return State.Buttons.LeftShoulder.Equals(ButtonState.Pressed); } }
        public float RT { get { return State.Triggers.Right; } }
        public float LT { get { return State.Triggers.Left; } }
        public bool Up { get { return State.DPad.IsUp; } }
        public bool Down { get { return State.DPad.IsDown; } }
        public bool Left { get { return State.DPad.IsLeft; } }
        public bool Right { get { return State.DPad.IsRight; } }
        public float LeftStickX { get { return State.ThumbSticks.Left.X; } }
        public float LeftStickY { get { return State.ThumbSticks.Left.Y; } }
        public bool LeftStickDown { get { return State.Buttons.LeftStick.Equals(ButtonState.Pressed); } }
        public float RightStickX { get { return State.ThumbSticks.Right.X; } }
        public float RightStickY { get { return State.ThumbSticks.Right.Y; } }
        public bool RightStickDown { get { return State.Buttons.RightStick.Equals(ButtonState.Pressed); } }

        public void GetState(int padID = 0)
        {
            State = OpenTK.Input.GamePad.GetState(padID);
            if (!State.IsConnected) Console.WriteLine("GamePad is not connected.");
        }

        public void ConsoleWriteState()
        {
            Console.WriteLine("A:{0}, B:{1}, X:{2}, Y:{3}", A, B, X, Y);
            Console.WriteLine("R:{0}, L:{1}, RT:{2}, LT:{3}", R, L, RT, LT);
            Console.WriteLine("Up:{0}, Down:{1}, Left:{2}, Right:{3}", Up, Down, Left, Right);
            Console.WriteLine("LeftStickX:{0}, LeftStickY:{1}, LeftStickDown:{2}", LeftStickX, LeftStickY, LeftStickDown);
            Console.WriteLine("RightStickX:{0}, RightStickY:{1}, RightStickDown:{2}", RightStickX, RightStickY, RightStickDown);
            Console.WriteLine();
        }
    }

}
