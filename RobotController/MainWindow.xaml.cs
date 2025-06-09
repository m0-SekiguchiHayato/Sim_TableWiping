using OpenRCF;
using System;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using static OpenRCF.HAoiTrajectry;
using static OpenRCF.HDataOutputter;
using static OpenRCF.HDynamixelManager;
using static OpenRCF.HGlobal;

namespace RobotController
{
    public partial class MainWindow : Window
    {

        // hayato：グラフィック関係ここから

        public MainWindow()
        {

            InitializeComponent();
            glHost.Child = Camera.GLControl;
            Loaded += InitializeOpenRCF;
            Setup();

        }

        private void InitializeOpenRCF(object sender, RoutedEventArgs e)
        {

            Core.SetFPS(30);
            Core.SetDrawFunction = Draw;

        }

        private void KeyDownHandler(object sender, KeyEventArgs e)
        {

            OpenRCF.Keyboard.KeyDownEvent(sender, e);
            if (OpenRCF.Keyboard.IsSpaceDowned) Camera.SetSubjectPosition(OpenRCF.Keyboard.SpaceVector.Get);

        }

        private void KeyUpHandler(object sender, KeyEventArgs e)
        {

            OpenRCF.Keyboard.KeyUpEvent(sender, e);

        }

        // hayato：ここまで



        private HTable Table1 = new HTable();
        private HTable Table2 = new HTable();
        private HVirtualAoi Virtual = new HVirtualAoi();
        private HGhostAoi Ghost = new HGhostAoi();


        private void UpdateDataTable()      // シミュレーションに表示する表などの更新をする関数
        {

            if (IsPortOpen && IsCurrentDisplayMode)
            {

                Dynamixel_2229.RequestCurrentReply(ID_2229);

                TB_Current22.Text = (currentScaringFactor * Dynamixel_2229.Current(22)).ToString();
                TB_Current23.Text = (currentScaringFactor * Dynamixel_2229.Current(23)).ToString();
                TB_Current24.Text = (currentScaringFactor * Dynamixel_2229.Current(24)).ToString();
                TB_Current25.Text = (currentScaringFactor * Dynamixel_2229.Current(25)).ToString();
                TB_Current26.Text = (currentScaringFactor * Dynamixel_2229.Current(26)).ToString();
                TB_Current27.Text = (currentScaringFactor * Dynamixel_2229.Current(27)).ToString();
                TB_Current28.Text = (currentScaringFactor * Dynamixel_2229.Current(28)).ToString();
                TB_Current29.Text = (currentScaringFactor * Dynamixel_2229.Current(29)).ToString();

                if (IsCurrentSaveMode)
                {
                    WriteFile_txt("Current23.txt", (currentScaringFactor * Dynamixel_2229.Current(23)).ToString());
                    WriteFile_txt("Current25.txt", (currentScaringFactor * Dynamixel_2229.Current(25)).ToString());
                    Console.WriteLine("Saved.");
                }

            }
            else
            {

                TB_Current22.Text = "No data";
                TB_Current23.Text = "No data";
                TB_Current24.Text = "No data";
                TB_Current25.Text = "No data";
                TB_Current26.Text = "No data";
                TB_Current27.Text = "No data";
                TB_Current28.Text = "No data";
                TB_Current29.Text = "No data";

            }

        }

        private void Setup()
        {

            Table2.Tabletop.Rotate.SetRz(Deg2Rad(30));

            // 並列処理
            //ParallelUI.RunEndless(UpdateDataTable, 1000);
            Parallel.RunEndless(Virtual.MoveVirtualAoiForLoop, PeriodMs);

        }

        private void Draw()
        {

            //Table1.Draw();
            //Table2.Draw();

            Virtual.Draw();
            Ghost.Draw();

            DrawTrajectory(Virtual.AoiArm);

        }



        private OpenRCF.Vector[] startAndEnd_Arm = new OpenRCF.Vector[2];
        private OpenRCF.Vector[] startAndEnd_Base = new OpenRCF.Vector[2];
        private int[] timeSpanMs = new int[StepNumMax];

        private void Button1_Click(object sender, RoutedEventArgs e)
        {

            Ghost.AoiArm.Kinematics.Target[1].Position.SetValue(0.56f, 0, 0.88f);
            Ghost.AoiArm.Kinematics.Target[1].Rotate.SetRy(Deg2Rad(180));
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();

        }

        private void Button2_Click(object sender, RoutedEventArgs e)
        {

            Ghost.AoiArm.Kinematics.Target[1].Position.SetValue(0.8f, 0, 0.73f);
            Ghost.AoiArm.Kinematics.Target[1].Rotate.SetRy(Deg2Rad(180));
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();

        }

        private void Button3_Click(object sender, RoutedEventArgs e)
        {

            for (int i = 0; i < 2; i++)
            {
                startAndEnd_Arm[i] = new OpenRCF.Vector(3);
                startAndEnd_Base[i] = new OpenRCF.Vector(3);
            }

            startAndEnd_Arm[0].SetValue(0.8f, 0, 0.73f);
            startAndEnd_Arm[1].SetValue(0.8f, -1.0f, 0.73f);

            startAndEnd_Base[0].SetValue(Ghost.AoiArm[0, 0].q, Ghost.AoiArm[0, 1].q + 1, 0);
            startAndEnd_Base[1].SetValue(Ghost.AoiArm[0, 0].q, Ghost.AoiArm[0, 1].q - 2, 0);

            for (int i = 0; i < timeSpanMs.Length; i++) timeSpanMs[i] = 200;
            float[] angleArray = new float[1] { 0 };

            SetTableWipingtrajectory(Virtual.AoiArm, Ghost.AoiArm, startAndEnd_Arm, startAndEnd_Base, timeSpanMs, angleArray);

        }

        private void Button4_Click(object sender, RoutedEventArgs e)
        {

            // ゴーストのリセット
            Ghost.AoiArm.Kinematics.ReturnHomePosition();
            Ghost.AoiArm.Kinematics.BaseRotate.SetRz(0);
            Ghost.AoiArm.Kinematics.ForwardKinematics();
            Ghost.AoiArm.Kinematics.SetTargetsToEffector();

            Ghost.AoiArm.Kinematics.Target[1].Position.SetValue(0.56f, 0, 0.88f);
            Ghost.AoiArm.Kinematics.Target[1].Rotate.SetRy(Deg2Rad(180));
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();

            Ghost.AoiArm.Kinematics.Target[1].Position.SetValue(0.8f, 0, 0.73f);
            Ghost.AoiArm.Kinematics.Target[1].Rotate.SetRy(Deg2Rad(180));
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();

        }

        private void Button5_Click(object sender, RoutedEventArgs e)
        {

            Virtual.Base.StartTask();
            Virtual.AoiArm.Trajectory.MoveToJointTargets(timeSpanMs);

        }



        private RotationMatrix Ry180 = new RotationMatrix(0, Deg2Rad(180), 0);
        private RotationMatrix Rz30 = new RotationMatrix(0, 0, Deg2Rad(30));
        private OpenRCF.Vector basePosition = new OpenRCF.Vector(3);
        private OpenRCF.Vector startPosition0 = new OpenRCF.Vector(3);
        private OpenRCF.Vector startPosition1 = new OpenRCF.Vector(3);
        private OpenRCF.Vector endPosition = new OpenRCF.Vector(3);
        private OpenRCF.Vector[] nodePosition_Arm = new OpenRCF.Vector[2];
        private float start_x = 0.7f;
        private void Button6_Click(object sender, RoutedEventArgs e)
        {

            basePosition.SetValue(0, 0, 0);
            basePosition.Set = (Rz30 * (basePosition - Table2.Position) + Table2.Position).Get;

            startPosition0.SetValue(0.56f, 0, 0.88f);
            startPosition0.Set = (Rz30 * (startPosition0 - Table2.Position) + Table2.Position).Get;

            startPosition1.SetValue(start_x, 0, 0.73f);
            startPosition1.Set = (Rz30 * (startPosition1 - Table2.Position) + Table2.Position).Get;

            endPosition.SetValue(start_x, -1.0f, 0.73f);
            endPosition.Set = (Rz30 * (endPosition - Table2.Position) + Table2.Position).Get;


            // ゴーストの再配置
            Ghost.AoiArm.Kinematics.ReturnHomePosition();
            Ghost.AoiArm.Kinematics.BaseRotate.SetRz(0);
            Ghost.AoiArm.Kinematics.ForwardKinematics();
            Ghost.AoiArm.Kinematics.SetTargetsToEffector();

            Ghost.AoiArm[0, 0].q = basePosition[0];
            Ghost.AoiArm[0, 1].q = basePosition[1];
            Ghost.AoiArm[0, 5].q = Deg2Rad(30);


            Ghost.AoiArm.Kinematics.Target[1].Position.Set = startPosition0.Get;
            Ghost.AoiArm.Kinematics.Target[1].Rotate.Set = (Rz30 * Ry180).Get;
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();


            Ghost.AoiArm.Kinematics.Target[1].Position.Set = startPosition1.Get;
            Ghost.AoiArm.Kinematics.Target[1].Rotate.Set = (Rz30 * Ry180).Get;
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();


            for (int i = 0; i < nodePosition_Arm.Length; i++)
            {
                nodePosition_Arm[i] = new OpenRCF.Vector(3);
            }

            nodePosition_Arm[0].Set = startPosition1.Get;
            nodePosition_Arm[1].Set = endPosition.Get;

        }

        private OpenRCF.Vector[] nodePosition_Base = new OpenRCF.Vector[3];
        private float norm = 0.8f;
        private void Button7_Click(object sender, RoutedEventArgs e)
        {

            for (int i = 0; i < nodePosition_Base.Length; i++)
            {
                nodePosition_Base[i] = new OpenRCF.Vector(3);
            }

            nodePosition_Base[0].SetValue(Ghost.AoiArm[0, 0].q, Ghost.AoiArm[0, 1].q, 0);
            nodePosition_Base[1].SetValue(Ghost.AoiArm[0, 0].q + norm * 0.5f, Ghost.AoiArm[0, 1].q - norm * (float)Math.Sqrt(3) / 2, 0);
            nodePosition_Base[2].SetValue(Ghost.AoiArm[0, 0].q + norm * 0.5f, Ghost.AoiArm[0, 1].q - norm * (float)Math.Sqrt(3) / 2 - 2, 0);

            for (int i = 0; i < timeSpanMs.Length; i++) timeSpanMs[i] = 250;

            float[] baseAngleTargets = new float[2];
            baseAngleTargets[0] = Deg2Rad(30);
            baseAngleTargets[1] = Deg2Rad(30);

            SetTableWipingtrajectory(Virtual.AoiArm, Ghost.AoiArm, nodePosition_Arm, nodePosition_Base, timeSpanMs, baseAngleTargets);

        }

        private OpenRCF.Vector[] sinPosition_Base = new OpenRCF.Vector[10];
        private float baseDeg = 30;
        private float amplitude = 0.15f;
        private void Button8_Click(object sender, RoutedEventArgs e)
        {

            for (int i = 0; i < sinPosition_Base.Length; i++)
            {
                sinPosition_Base[i] = new OpenRCF.Vector(3);
            }

            RotationMatrix R = new RotationMatrix(0, 0, Deg2Rad(baseDeg));
            OpenRCF.Vector v_sin = new OpenRCF.Vector(3);
            float[] baseAngleTargets = new float[sinPosition_Base.Length - 1];

            for (int i = 0; i < sinPosition_Base.Length; i++)
            {

                v_sin.SetValue(amplitude * (float)Math.Sin(2 * (float)Math.PI / (sinPosition_Base.Length - 1) * i), -(float)i / (sinPosition_Base.Length - 1), 0);
                sinPosition_Base[i].SetValue(Ghost.AoiArm[0, 0].q, Ghost.AoiArm[0, 1].q, 0);

                sinPosition_Base[i] = sinPosition_Base[0] + R * v_sin;

                if (i < sinPosition_Base.Length - 1) baseAngleTargets[i] = Deg2Rad(baseDeg);

            }

            for (int i = 0; i < timeSpanMs.Length; i++) timeSpanMs[i] = 300;

            SetTableWipingtrajectory(Virtual.AoiArm, Ghost.AoiArm, nodePosition_Arm, sinPosition_Base, timeSpanMs, baseAngleTargets);

        }

        private OpenRCF.Vector[] avoidancePosition_Base = new OpenRCF.Vector[4];
        private void Button9_Click(object sender, RoutedEventArgs e)
        {

            for (int i = 0; i < avoidancePosition_Base.Length; i++)
            {
                avoidancePosition_Base[i] = new OpenRCF.Vector(3);
            }

            RotationMatrix R = new RotationMatrix(0, 0, Deg2Rad(baseDeg));
            OpenRCF.Vector vTmp = new OpenRCF.Vector(3);
            float[] baseAngleTargets = new float[avoidancePosition_Base.Length - 1];

            avoidancePosition_Base[0].SetValue(Ghost.AoiArm[0, 0].q, Ghost.AoiArm[0, 1].q, 0);

            for (int i = 0; i < avoidancePosition_Base.Length; i++)
            {
                if (i == 1) vTmp.SetValue(0.2f, -0.1f, 0);
                else if (i == 2) vTmp.SetValue(0, -0.8f, 0);
                else vTmp.SetValue(-0.2f, -0.1f, 0);

                if (i != 0) avoidancePosition_Base[i] = avoidancePosition_Base[i - 1] + R * vTmp;

                if (i < avoidancePosition_Base.Length - 1) baseAngleTargets[i] = Deg2Rad(baseDeg);
            }

            for (int i = 0; i < timeSpanMs.Length; i++)
            {
                if (i == 4 || i == 5 || i == 38 || i == 39) timeSpanMs[i] = 500;
                else timeSpanMs[i] = 250;

                //timeSpanMs[i] = 250;
            }

            SetTableWipingtrajectory(Virtual.AoiArm, Ghost.AoiArm, nodePosition_Arm, avoidancePosition_Base, timeSpanMs, baseAngleTargets);

        }

        private void Button10_Click(object sender, RoutedEventArgs e)
        {

            // ゴーストの再配置
            Ghost.AoiArm.Kinematics.ReturnHomePosition();
            Ghost.AoiArm.Kinematics.BaseRotate.SetRz(0);
            Ghost.AoiArm.Kinematics.ForwardKinematics();
            Ghost.AoiArm.Kinematics.SetTargetsToEffector();

            Ghost.AoiArm[0, 0].q = basePosition[0];
            Ghost.AoiArm[0, 1].q = basePosition[1];
            Ghost.AoiArm[0, 5].q = Deg2Rad(30);


            Ghost.AoiArm.Kinematics.Target[1].Position.Set = startPosition0.Get;
            Ghost.AoiArm.Kinematics.Target[1].Rotate.Set = (Rz30 * Ry180).Get;
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();


            Ghost.AoiArm.Kinematics.Target[1].Position.Set = startPosition1.Get;
            Ghost.AoiArm.Kinematics.Target[1].Rotate.Set = (Rz30 * Ry180).Get;
            Ghost.SetVirtualJointSpringDefault();

            Ghost.AoiArm.Kinematics.NewInverseKinematics();

        }



        private void Button11_Click(object sender, RoutedEventArgs e)
        {

        }

        private void Button12_Click(object sender, RoutedEventArgs e)
        {

        }

        private void Button13_Click(object sender, RoutedEventArgs e)
        {

        }

        private void Button14_Click(object sender, RoutedEventArgs e)
        {

        }

        private void Button15_Click(object sender, RoutedEventArgs e)
        {

        }






        private void VirtualDrawButton_Click(object sender, RoutedEventArgs e)
        {

            Virtual.SwitchDrawMode();

            if (Virtual.IsDrawMode)
            {
                VirtualDrawStatus.Text = "Visible";
                VirtualDrawButton.Background = Brushes.Lime;
            }
            else
            {
                VirtualDrawStatus.Text = "Invisible";
                VirtualDrawButton.Background = Brushes.LightSlateGray;
            }

        }

        private void GhostDrawButton_Click(object sender, RoutedEventArgs e)
        {

            Ghost.SwitchDrawMode();

            if (Ghost.IsDrawMode)
            {
                GhostDrawStatus.Text = "Visible";
                GhostDrawButton.Background = Brushes.Lime;
            }
            else
            {
                GhostDrawStatus.Text = "Invisible";
                GhostDrawButton.Background = Brushes.LightSlateGray;
            }

        }

        private void TrajectoryDrawButton_Click(object sender, RoutedEventArgs e)
        {

            SwitchTrajectoryDrawMode();

            if (IsTrajectoryDrawMode)
            {
                TrajectoryDrawStatus.Text = "Visible";
                TrajectoryDrawButton.Background = Brushes.Lime;
            }
            else
            {
                TrajectoryDrawStatus.Text = "Invisible";
                TrajectoryDrawButton.Background = Brushes.LightSlateGray;
            }

        }

        private void WiperButton_Click(object sender, RoutedEventArgs e)
        {

            Virtual.Arm.SwitchWiperMode();
            Ghost.SwitchWiperMode();

            if (Virtual.Arm.IsWiperEnabled && Ghost.IsWiperEnabled)
            {
                WiperStatus.Text = "ON";
                WiperButton.Background = Brushes.Lime;
            }
            else
            {
                WiperStatus.Text = "OFF";
                WiperButton.Background = Brushes.LightSlateGray;
            }

        }





        private void ManualButton_Click(object sender, RoutedEventArgs e)
        {

            Virtual.SwitchManualMode();

            if (Virtual.IsManualMode)
            {
                ManualStatus.Text = "ON";
                ManualButton.Background = Brushes.Lime;
            }
            else
            {
                ManualStatus.Text = "OFF";
                ManualButton.Background = Brushes.LightSlateGray;
            }

        }

        private void PantomimeButtonContent()   // パントマイムボタンの中身. 他のボタンにも流用したかったので作成
        {

            Virtual.Arm.SwitchPantomimeMode();

            if (Virtual.Arm.IsPantomimeMode)
            {
                PantomimeStatus.Text = "ON";
                PantomimeButton.Background = Brushes.Lime;
            }
            else
            {
                PantomimeStatus.Text = "OFF";
                PantomimeButton.Background = Brushes.LightSlateGray;
            }

        }

        private void PantomimeButton_Click(object sender, RoutedEventArgs e)
        {

            PantomimeButtonContent();

        }

        private void AutoVelocityButton_Click(object sender, RoutedEventArgs e)
        {

            Virtual.SwitchAutoTaskMode();

            if (Virtual.IsAutoVelocityMode)
            {
                AutoVelocityStatus.Text = "ON";
                AutoVelocityButton.Background = Brushes.Lime;
            }
            else
            {
                AutoVelocityStatus.Text = "OFF";
                AutoVelocityButton.Background = Brushes.LightSlateGray;
            }

        }

        private void HandButton_Click(object sender, RoutedEventArgs e)
        {

            Virtual.Arm.SwitchHandMode();

            if (Virtual.Arm.IsHandOpen)
            {
                HandStatus.Text = "Open";
                HandButton.Background = Brushes.Lime;
            }
            else
            {
                HandStatus.Text = "Closed";
                HandButton.Background = Brushes.LightSlateGray;
            }

        }





        private void PortButton_Click(object sender, RoutedEventArgs e)
        {

            SwitchPortMode();

            if (IsPortOpen)
            {
                PortStatus.Text = "Open";
                PortButton.Background = Brushes.Lime;
            }
            else
            {
                PortStatus.Text = "Closed";
                PortButton.Background = Brushes.LightSlateGray;
            }

        }

        private void TorqueButton_Click(object sender, RoutedEventArgs e)
        {

            ResetReflectStatus();


            if (!IsPortOpen)
            {
                Console.WriteLine("\nSerial port is not open.");
                return;
            }


            SwitchTorqueMode();

            if (IsTorqueON)
            {

                TorqueStatus.Text = "ON";
                TorqueButton.Background = Brushes.Lime;

            }
            else
            {

                TorqueStatus.Text = "OFF";
                TorqueButton.Background = Brushes.LightSlateGray;

                if (IsRinkMode)
                {
                    SwitchRinkMode();
                    RinkStatus.Text = "OFF";
                    RinkButton.Background = Brushes.LightSlateGray;
                }

            }

        }

        // ReflectVirtualToReal()関数を引数がない状態にする
        private void ReflectVTR()
        {
            ReflectVirtualToReal(Virtual);
        }

        private void RinkButton_Click(object sender, RoutedEventArgs e)
        {

            if (!IsPortOpen)
            {
                Console.WriteLine("\nSerial port is not open.");
                return;
            }

            if (!IsTorqueON)
            {
                Console.WriteLine("\nTorque is not enabled. Please press the Torque button.");
                return;
            }

            if (!IsReflect)
            {
                Console.WriteLine("\nDANGER: Reflection has not completed. Please press the Reflect button.");
                return;
            }


            SwitchRinkMode();

            if (IsRinkMode)
            {
                Parallel.Run(ReflectVTR, () => !IsRinkMode, PeriodMs);
                RinkStatus.Text = "ON";
                RinkButton.Background = Brushes.Lime;
            }
            else
            {
                RinkStatus.Text = "OFF";
                RinkButton.Background = Brushes.LightSlateGray;
                ResetReflectStatus();
            }

        }





        // ReflectRealPositionToVirtual()を引数がない状態にする
        private void ReflectRPTV()
        {
            ReflectRealPositionToVirtual(Virtual.AoiArm);
        }

        private void ReflectButton_Click(object sender, RoutedEventArgs e)
        {

            if (IsReflect) ResetReflectStatus();

            if (!IsRinkMode && IsPortOpen)
            {
                if (Virtual.Arm.IsPantomimeMode) PantomimeButtonContent();
                Parallel.Run(ReflectRPTV, () => IsReflect, PeriodMs);
            }
            else if (IsRinkMode)
            {
                Console.WriteLine("\nDANGER: Rink mode is active. If you want to reflect, plese turn rink mode off.");
            }
            else Console.WriteLine("\nSerial port is not open.");

        }

        private void TrackButton_Click(object sender, RoutedEventArgs e)
        {

            if (Virtual.Arm.IsPantomimeMode) PantomimeButtonContent();
            MoveVirtualArmToGhost(Virtual.AoiArm, Ghost.AoiArm, 1500);

            Virtual.Base.BaseTarget.Position.Set = Ghost.AoiArm.Torso.Position.Get;
            Virtual.Base.BaseTarget.Rotate.Set = Ghost.AoiArm.Torso.Rotate.Get;

        }





        private void GamePadButton_Click(object sender, RoutedEventArgs e)
        {

            Virtual.SwitchGamePadMode();

            if (Virtual.IsGamePadMode)
            {
                GamePadStatus.Text = "ON";
                GamePadButton.Background = Brushes.Lime;
            }
            else
            {
                GamePadStatus.Text = "OFF";
                GamePadButton.Background = Brushes.LightSlateGray;
            }

        }





        // ロボットのリセットボタンの中身
        private void ResetRobotButtonContent()
        {

            if (!IsRinkMode) ResetReflectStatus();

            Ghost.AoiArm.Kinematics.ReturnHomePosition();
            Ghost.AoiArm.Kinematics.BaseRotate.SetRz(0);
            Ghost.AoiArm.Kinematics.ForwardKinematics();
            Ghost.AoiArm.Kinematics.SetTargetsToEffector();

            Virtual.AoiBase.Position.SetValue(AoiHomePosition);
            Virtual.AoiBase.Rotate.SetRz(0);

            Virtual.Base.BaseTarget.Position.SetValue(AoiHomePosition);
            Virtual.Base.BaseTarget.Position[2] = 0;
            Virtual.Base.BaseTarget.Rotate.SetRz(0);

            if (Virtual.Arm.IsPantomimeMode) PantomimeButtonContent();
            MoveVirtualArmToGhost(Virtual.AoiArm, Ghost.AoiArm, 1500);

        }

        private void ResetViewButtonCotent()    // 視点のリセットボタンの中身
        {

            Camera.SetSubjectPosition(Camera.PositionInit);
            Camera.Distance = Camera.DistanceInit;
            Camera.Angle = Camera.AngleInit;
            Camera.Height = Camera.HeightInit;

        }

        private void ResetRobotButton_Click(object sender, RoutedEventArgs e)
        {

            ResetRobotButtonContent();

        }

        private void ResetViewButton_Click(object sender, RoutedEventArgs e)
        {

            ResetViewButtonCotent();

        }

        private void ResetAllButton_Click(object sender, RoutedEventArgs e)
        {

            ResetRobotButtonContent();
            ResetViewButtonCotent();

        }





        private void CurrentDisplayButton_Click(object sender, RoutedEventArgs e)
        {

            SwitchCurrentDisplayMode();

            if (IsCurrentDisplayMode)
            {
                CurrentStatus.Text = "Shown";
                CurrentDisplayButton.Background = Brushes.Lime;
            }
            else
            {
                CurrentStatus.Text = "Hidden";
                CurrentDisplayButton.Background = Brushes.LightSlateGray;
            }

        }

    }

}
