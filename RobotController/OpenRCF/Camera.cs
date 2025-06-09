using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using System;
using System.Windows.Forms;

namespace OpenRCF
{
    static class Camera
    {
        private static int GraphicsMode_AntialiasLevel = 6; // Low quality 0  ~  12 High quality
        private static GraphicsMode mode = new GraphicsMode(
            GraphicsMode.Default.ColorFormat,
            GraphicsMode.Default.Depth,
            GraphicsMode.Default.Stencil,
            GraphicsMode_AntialiasLevel,
            GraphicsMode.Default.AccumulatorFormat,
            GraphicsMode.Default.Buffers,
            GraphicsMode.Default.Stereo
        );

        internal static GLControl GLControl = new GLControl(mode);

        static Camera()
        {
            GLControl.Load += GL_Load;
            GLControl.Resize += GL_Resize;

            GLControl.MouseMove += new MouseEventHandler(MouseMoveEvent);
            GLControl.MouseDown += new MouseEventHandler((object sender, MouseEventArgs e) => { eOld = e.Location; });
            GLControl.MouseWheel += new MouseEventHandler((object sender, MouseEventArgs e) => { Distance += 0 < e.Delta ? 0.5f : -0.5f; });
        }

        private static System.Drawing.Point eOld = new System.Drawing.Point();
        private static float gain = 0.005f;
        private static float Ca, Sa;
        private static void MouseMoveEvent(object sender, MouseEventArgs e)
        {
            if (!GLControl.MouseButtons.Equals(MouseButtons.None))
            {
                if (GLControl.MouseButtons.Equals(MouseButtons.Left))
                {
                    Angle += -gain * (e.Location.X - eOld.X);
                }
                else if (GLControl.MouseButtons.Equals(MouseButtons.Right))
                {
                    Ca = (float)Math.Cos(Angle);
                    Sa = (float)Math.Sin(Angle);
                    viewedPosition[0] += gain * (-(e.Location.X - eOld.X) * Ca - (e.Location.Y - eOld.Y) * Sa);
                    viewedPosition[1] += gain * (-(e.Location.X - eOld.X) * Sa + (e.Location.Y - eOld.Y) * Ca);
                    SettingUpdate();
                }
                else if (GLControl.MouseButtons.Equals(MouseButtons.Middle))
                {
                    viewedPosition[2] += gain * (e.Location.Y - eOld.Y);
                    SettingUpdate();
                }

                eOld = e.Location;
            }
        }


        public static readonly float[] PositionInit = new float[] { 0, 0, 0 };    // hayato：カメラの初期位置. 引数をいじることでカメラの初期位置を変えられる
        private static Vector3 viewedPosition = new Vector3(PositionInit[0], PositionInit[1], PositionInit[2]);   // hayato：初期化子を追加
        private static Vector3 viewPosition;
        private static Matrix4 CameraMatrix;

        public const float DistanceInit = 2;   // hayato：default 3
        public const float AngleInit = -(float)Math.PI / 4;    // hayato：default 0
        public const float HeightInit = 1;    // hayato：default 1.2
        private static float distance = DistanceInit;
        private static float angle = AngleInit;
        private static float height = HeightInit;

        private static Light[] light = new Light[8];

        private class Light
        {
            private float[] ambient = new float[4] { 0.03f, 0.03f, 0.03f, 0 };
            private float[] position = new float[4] { 0, 0, 3, 0 };
            private LightName LightName;
            private EnableCap EnableCap;

            public void SetPosition(float x, float y, float z)
            {
                position[0] = x;
                position[1] = y;
                position[2] = z;
                GL.Light(LightName, LightParameter.Position, position);
            }

            public void SetAmbient(float ambient)
            {
                this.ambient[0] = ambient;
                this.ambient[1] = ambient;
                this.ambient[2] = ambient;
                GL.Light(LightName, LightParameter.Ambient, this.ambient);
            }

            public Light(int i)
            {
                if (i == 0) LightName = LightName.Light0;
                else if (i == 1) LightName = LightName.Light1;
                else if (i == 2) LightName = LightName.Light2;
                else if (i == 3) LightName = LightName.Light3;
                else if (i == 4) LightName = LightName.Light4;
                else if (i == 5) LightName = LightName.Light5;
                else if (i == 6) LightName = LightName.Light6;
                else if (i == 7) LightName = LightName.Light7;

                GL.Light(LightName, LightParameter.Ambient, ambient);
                GL.Light(LightName, LightParameter.Diffuse, System.Drawing.Color.Gray);
                GL.Light(LightName, LightParameter.Position, position);

                if (i == 0) EnableCap = EnableCap.Light0;
                else if (i == 1) EnableCap = EnableCap.Light1;
                else if (i == 2) EnableCap = EnableCap.Light2;
                else if (i == 3) EnableCap = EnableCap.Light3;
                else if (i == 4) EnableCap = EnableCap.Light4;
                else if (i == 5) EnableCap = EnableCap.Light5;
                else if (i == 6) EnableCap = EnableCap.Light6;
                else if (i == 7) EnableCap = EnableCap.Light7;
            }

            public void Enable() { GL.Enable(EnableCap); }

            public void Disable() { GL.Disable(EnableCap); }

            public void UpdatePosition()
            {
                GL.Light(LightName, LightParameter.Position, position);
            }
        }

        public static float Distance
        {
            get { return distance; }
            set
            {
                if (0.001f < value) distance = value;
                else distance = 0.001f;

                SettingUpdate();
            }
        }

        public static float Angle
        {
            get { return angle; }
            set
            {
                angle = value;
                SettingUpdate();
            }
        }

        public static float Height
        {
            get { return height; }
            set
            {
                height = value;
                SettingUpdate();
            }
        }

        public static void SetSubjectPosition(float[] position)
        {
            viewedPosition[0] = position[0];
            viewedPosition[1] = position[1];
            viewedPosition[2] = position[2];
            SettingUpdate();
        }

        public static void SetAmbient(float ambient)
        {
            for (int i = 0; i < light.Length; i++)
            {
                if (1 < ambient) light[i].SetAmbient(1);
                else if (ambient < 0) light[i].SetAmbient(0);
                else light[i].SetAmbient(ambient);
            }
        }

        private static void GL_Load(object sender, EventArgs e)
        {
            GL.ClearColor(Color4.White);

            GL.Viewport(0, 0, GLControl.Width, GLControl.Height);

            viewPosition[0] = viewedPosition[0] + distance * (float)Math.Cos(angle - 0.5 * Math.PI);
            viewPosition[1] = viewedPosition[1] + distance * (float)Math.Sin(angle - 0.5 * Math.PI);
            viewPosition[2] = viewedPosition[2] + height;

            GL.MatrixMode(MatrixMode.Projection);
            Matrix4 proj = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver3, GLControl.AspectRatio, 0.2f, 15);
            GL.LoadMatrix(ref proj);

            GL.MatrixMode(MatrixMode.Modelview);
            CameraMatrix = Matrix4.LookAt(viewPosition, viewedPosition, Vector3.UnitZ);
            GL.LoadMatrix(ref CameraMatrix);

            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.Lighting);
            GL.ShadeModel(ShadingModel.Smooth);
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

            for (int i = 0; i < light.Length; i++)
            {
                light[i] = new Light(i);
            }

            light[0].SetPosition(12, 9, 3);
            light[1].SetPosition(9, -12, 12);
            light[2].SetPosition(-12, 9, 12);
            light[3].SetPosition(-9, -12, -3);
            light[4].SetPosition(0, 0, -12);

            light[0].Enable();
            light[1].Enable();
            light[2].Enable();
            light[3].Enable();
            light[4].Enable();

            GL.EnableClientState(ArrayCap.VertexArray);
        }

        private static void GL_Resize(object sender, EventArgs e)
        {
            GL.Viewport(0, 0, GLControl.Width, GLControl.Height);
        }

        private static void SettingUpdate()
        {
            viewPosition[0] = viewedPosition[0] + distance * (float)Math.Cos(angle - 0.5 * Math.PI);
            viewPosition[1] = viewedPosition[1] + distance * (float)Math.Sin(angle - 0.5 * Math.PI);
            viewPosition[2] = viewedPosition[2] + height;

            CameraMatrix = Matrix4.LookAt(viewPosition, viewedPosition, Vector3.UnitZ);
            GL.LoadMatrix(ref CameraMatrix);

            for (int i = 0; i < light.Length; i++)
            {
                light[i].UpdatePosition();
            }
        }

        internal static void DisplayUpdate()
        {
            GLControl.SwapBuffers();
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
        }

    }

}
