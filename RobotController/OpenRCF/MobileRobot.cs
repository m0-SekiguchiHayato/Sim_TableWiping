using System;

namespace OpenRCF
{
    public static class MobileRobot
    {
        public class ObjectBase
        {
            protected PrickleBall Effector = new PrickleBall();
            protected bool isKinematicsEnabled = false;

            public Vector Position
            {
                get { return Effector.Position; }
                set { Effector.Position = value; }
            }

            public RotationMatrix Rotate
            {
                get { return Effector.Rotate; }
                set { Effector.Rotate = value; }
            }

            private uint samplingTimeMs = 50;
            public uint SamplingTimeMs
            {
                get { return samplingTimeMs; }
                set
                {
                    if (isKinematicsEnabled)
                    {
                        Console.WriteLine("Error: SamplingTime must be set before StartKinematics().");
                    }
                    else if (value < 1)
                    {
                        Console.WriteLine("Error: SamplingTime is too small.");
                    }
                    else
                    {
                        samplingTimeMs = value;
                    }
                }
            }

            protected float SamplingTimeS { get { return 0.001f * samplingTimeMs; } }

        }

        public class TwoWheel : ObjectBase
        {
            public Vector Velocity = new Vector(2);
            public Vector WheelVelocity = new Vector(2);
            public Vector WheelAngle = new Vector(2);
            public Matrix J = new Matrix(2, 2);
            private Matrix J1 = new Matrix(2, 2);
            private Matrix J2 = new Matrix(2, 2);
            private Pillar[] Wheel = new Pillar[2];

            private float wheelRadius = 0.1f;
            public float WheelRadius
            {
                get { return wheelRadius; }
                set
                {
                    wheelRadius = value;
                    Effector.Diameter = 0.4f * wheelRadius;
                    Wheel[0].Radius = wheelRadius;
                    Wheel[0].Height = 0.4f * wheelRadius;
                    Wheel[1].Radius = wheelRadius;
                    Wheel[1].Height = 0.4f * wheelRadius;
                    Position[2] = wheelRadius;
                }
            }

            private float treadWidth = 0.4f;
            public float TreadWidth
            {
                get { return treadWidth; }
                set
                {
                    treadWidth = value;
                    Wheel[0].SetPositionOffset(-distance, -0.5f * treadWidth, 0);
                    Wheel[1].SetPositionOffset(-distance, 0.5f * treadWidth, 0);
                }
            }

            private float distance = 0.35f;
            public float Distance
            {
                get { return distance; }
                set
                {
                    distance = value;
                    Wheel[0].SetPositionOffset(-distance, -0.5f * treadWidth, 0);
                    Wheel[1].SetPositionOffset(-distance, 0.5f * treadWidth, 0);
                }
            }

            public byte Transparency
            {
                get { return Effector.Color.Alpha; }
                set
                {
                    Wheel[0].Color.Alpha = value;
                    Wheel[1].Color.Alpha = value;
                    Effector.Color.Alpha = value;
                }
            }

            public TwoWheel(float wheelRadius = 0.1f, float treadWidth = 0.4f, float distance = 0.3f)
            {
                for (int i = 0; i < Wheel.Length; i++)
                {
                    Wheel[i] = new Pillar();
                    Wheel[i].SetRotateOffset(0.5f * (float)Math.PI, 0, 0);
                    Wheel[i].Position = Effector.Position;
                    Wheel[i].Rotate = Effector.Rotate;
                }

                WheelRadius = wheelRadius;
                TreadWidth = treadWidth;
                Distance = distance;
            }

            public void Draw()
            {
                Effector.Draw();
                Wheel[0].Draw();
                Wheel[0].DrawStripes();
                Wheel[1].Draw();
                Wheel[1].DrawStripes();
            }

            private void CalcJacobiMatrix()
            {
                J1[0, 0] = Rotate[0, 0];
                J1[0, 1] = Rotate[0, 1];
                J1[1, 0] = Rotate[1, 0];
                J1[1, 1] = Rotate[1, 1];

                J2[0, 0] = 0.5f * WheelRadius;
                J2[0, 1] = 0.5f * WheelRadius;
                J2[1, 0] = WheelRadius * Distance / TreadWidth;
                J2[1, 1] = -WheelRadius * Distance / TreadWidth;

                J = J1 * J2;
            }

            private RotationMatrix Rz = new RotationMatrix();
            private void AddSmallDisplacement()
            {
                Position[0] += SamplingTimeS * Velocity[0];
                Position[1] += SamplingTimeS * Velocity[1];
                Rz.SetRz(SamplingTimeS * (WheelRadius / TreadWidth * (WheelVelocity[0] - WheelVelocity[1])));
                Rotate.Set = Rotate.Times(Rz);
                WheelAngle += SamplingTimeS * WheelVelocity;
                Wheel[0].SetRotateOffset(0.5f * (float)Math.PI, 0, -WheelAngle[0]);
                Wheel[1].SetRotateOffset(0.5f * (float)Math.PI, 0, -WheelAngle[1]);
            }

            public void InverseKinematics()
            {
                CalcJacobiMatrix();
                WheelVelocity = J.Inv * Velocity;
                AddSmallDisplacement();
            }

            public void ForwardKinematics()
            {
                CalcJacobiMatrix();
                Velocity = J * WheelVelocity;
                AddSmallDisplacement();
            }

            public void StartInverseKinematics()
            {
                if (isKinematicsEnabled)
                {
                    Console.WriteLine("Warning: StartKinematics() is already enabled.");
                }
                else
                {
                    Parallel.RunEndless(InverseKinematics, SamplingTimeMs);
                    isKinematicsEnabled = true;
                }
            }

            public void StartForwardKinematics()
            {
                if (isKinematicsEnabled)
                {
                    Console.WriteLine("Warning: StartKinematics() is already enabled.");
                }
                else
                {
                    Parallel.RunEndless(ForwardKinematics, SamplingTimeMs);
                    isKinematicsEnabled = true;
                }
            }

        }

        public class Mecanum : ObjectBase
        {
            public Vector Velocity = new Vector(3);
            public Vector WheelVelocity = new Vector(4);
            public Vector WheelAngle = new Vector(4);
            public Matrix Jinv = new Matrix(4, 3);
            private Pillar[] Wheel = new Pillar[4];

            private float wheelRadius = 0.08f;
            public float WheelRadius
            {
                get { return wheelRadius; }
                set
                {
                    wheelRadius = value;
                    Effector.Diameter = 0.6f * wheelRadius;
                    Position[2] = wheelRadius;

                    for (int i = 0; i < Wheel.Length; i++)
                    {
                        Wheel[i].Radius = wheelRadius;
                        Wheel[i].Height = wheelRadius;
                    }
                }
            }

            private float treadWidth = 0.5f;
            public float TreadWidth
            {
                get { return treadWidth; }
                set
                {
                    treadWidth = value;
                    Wheel[0].SetPositionOffset(0.5f * distance, -0.5f * treadWidth, 0);
                    Wheel[1].SetPositionOffset(0.5f * distance, 0.5f * treadWidth, 0);
                    Wheel[2].SetPositionOffset(-0.5f * distance, 0.5f * treadWidth, 0);
                    Wheel[3].SetPositionOffset(-0.5f * distance, -0.5f * treadWidth, 0);
                }
            }

            private float distance = 0.5f;
            public float Distance
            {
                get { return distance; }
                set
                {
                    distance = value;
                    Wheel[0].SetPositionOffset(0.5f * distance, -0.5f * treadWidth, 0);
                    Wheel[1].SetPositionOffset(0.5f * distance, 0.5f * treadWidth, 0);
                    Wheel[2].SetPositionOffset(-0.5f * distance, 0.5f * treadWidth, 0);
                    Wheel[3].SetPositionOffset(-0.5f * distance, -0.5f * treadWidth, 0);
                }
            }

            public byte Transparency
            {
                get { return Effector.Color.Alpha; }
                set
                {
                    Wheel[0].Color.Alpha = value;
                    Wheel[1].Color.Alpha = value;
                    Wheel[2].Color.Alpha = value;
                    Wheel[3].Color.Alpha = value;
                    Effector.Color.Alpha = value;
                }
            }

            public Mecanum(float wheelRadius = 0.08f, float treadWidth = 0.5f, float distance = 0.5f)
            {
                for (int i = 0; i < Wheel.Length; i++)
                {
                    Wheel[i] = new Pillar();
                    Wheel[i].SetRotateOffset(0.5f * (float)Math.PI, 0, 0);
                    Wheel[i].Position = Effector.Position;
                    Wheel[i].Rotate = Effector.Rotate;
                }

                WheelRadius = wheelRadius;
                TreadWidth = treadWidth;
                Distance = distance;
            }

            public void Draw()
            {
                Effector.Draw();

                for (int i = 0; i < Wheel.Length; i++)
                {
                    Wheel[i].Draw();
                    Wheel[i].DrawStripes();
                }
            }

            private void CalcJacobiMatrix()
            {
                Jinv[0, 0] = 1;
                Jinv[0, 1] = 1;
                Jinv[0, 2] = 0.5f * (treadWidth + distance);

                Jinv[1, 0] = 1;
                Jinv[1, 1] = -1;
                Jinv[1, 2] = -0.5f * (treadWidth + distance);

                Jinv[2, 0] = 1;
                Jinv[2, 1] = 1;
                Jinv[2, 2] = -0.5f * (treadWidth + distance);

                Jinv[3, 0] = 1;
                Jinv[3, 1] = -1;
                Jinv[3, 2] = 0.5f * (treadWidth + distance);

                Jinv = (1 / wheelRadius) * Jinv;
            }

            private RotationMatrix Rz = new RotationMatrix();
            private void AddSmallDisplacement()
            {
                Position[0] += SamplingTimeS * Velocity[0];
                Position[1] += SamplingTimeS * Velocity[1];
                Rz.SetRz(SamplingTimeS * Velocity[2]);
                Rotate.Set = Rotate.Times(Rz);
                WheelAngle += SamplingTimeS * WheelVelocity;

                for (int i = 0; i < Wheel.Length; i++)
                {
                    Wheel[i].SetRotateOffset(0.5f * (float)Math.PI, 0, -WheelAngle[i]);
                }
            }

            public void InverseKinematics()
            {
                CalcJacobiMatrix();
                WheelVelocity = Jinv * (Rotate.Trans * Velocity);
                AddSmallDisplacement();
            }

            public void StartInverseKinematics()
            {
                if (isKinematicsEnabled)
                {
                    Console.WriteLine("Warning: StartKinematics() is already enabled.");
                }
                else
                {
                    Parallel.RunEndless(InverseKinematics, SamplingTimeMs);
                    isKinematicsEnabled = true;
                }
            }

        }

    }

}
