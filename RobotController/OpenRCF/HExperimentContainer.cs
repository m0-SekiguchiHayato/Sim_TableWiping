using System;
using static OpenRCF.HGlobal;

namespace OpenRCF
{
    public class HExperimentContainer     // 実験条件などを保存しておくためのクラス
    {

        // 西村さんの修論の実験条件を再現したもの. 目標値2つのうち1つ目
        public void MasterThesisExperient1(Robot ghost_AoiArm)
        {

            ghost_AoiArm.Kinematics.BaseRotate.SetRz((float)Math.PI / 2);
            ghost_AoiArm.Kinematics.ForwardKinematics();

            ghost_AoiArm.Kinematics.Target[0].Priority = false;
            ghost_AoiArm.Kinematics.Target[0].Position.SetValue(1.05f, 0.10f, 0);
            ghost_AoiArm.Kinematics.Target[0].Rotate.SetRz(Deg2Rad(93.566f));
            ghost_AoiArm.Kinematics.Target[1].Position.SetValue(1.5f, 0.3f, 0.7f);
            ghost_AoiArm.Kinematics.Target[1].Rotate.SetRx(-(float)Math.PI);

            ghost_AoiArm.Kinematics.JointTarget[1].SetValue(0, 0, 0, Deg2Rad(-90), 0, 0, 0);


            //ghost_AoiArm.Kinematics.ObserveOldIK(100);
            //ghost_AoiArm.Kinematics.OldInverseKinematics();
            ghost_AoiArm.Kinematics.ObserveNewIK(100);
            //ghost_AoiArm.Kinematics.NewInverseKinematics();

        }

        // 西村さんの修論の実験条件を再現したもの. 目標値2つのうち2つ目
        public void MasterThesisExperient2(Robot ghost_AoiArm)
        {

            ghost_AoiArm.Kinematics.Target[0].Priority = false;
            ghost_AoiArm.Kinematics.Target[0].Position.SetValue(1.05f, 0.10f, 0);
            ghost_AoiArm.Kinematics.Target[0].Rotate.SetRz(Deg2Rad(93.566f));
            ghost_AoiArm.Kinematics.Target[1].Position.SetValue(0.5f, -0.3f, 0.35f);
            ghost_AoiArm.Kinematics.Target[1].Rotate.SetRx(-(float)Math.PI);

            ghost_AoiArm.Kinematics.JointTarget[1].SetValue(0, 0, 0, Deg2Rad(-90), 0, 0, 0);


            //ghost_AoiArm.Kinematics.ObserveOldIK(100);
            //ghost_AoiArm.Kinematics.OldInverseKinematics();
            ghost_AoiArm.Kinematics.ObserveNewIK(100);
            //ghost_AoiArm.Kinematics.NewInverseKinematics();

        }


        // トルク測定実験用
        public void TorqueExperiment(Robot ghost_AoiArm)
        {

            ghost_AoiArm.Kinematics.Target[0].Priority = false;
            ghost_AoiArm.Kinematics.Target[0].Position.SetValue(0.5f, -1.5f, 0);
            ghost_AoiArm.Kinematics.Target[1].Position.SetValue(1.15f, -1.5f, 0.5f);
            ghost_AoiArm.Kinematics.Target[1].Rotate.SetRy(-(float)Math.PI);

            ghost_AoiArm.Kinematics.JointTarget[1].SetValue(0, 0, 0, Deg2Rad(-90f), 0, 0, 0);


            //ghost_AoiArm.Kinematics.OldInverseKinematics();
            ghost_AoiArm.Kinematics.NewInverseKinematics();

        }


        // ランダムな目標値を与えた時の挙動を見る実験
        private Random RandInt_x = new Random(6);
        private Random RandInt_y = new Random(7);
        private Random RandInt_z = new Random(8);
        public void PreRandTargetExperiment(Robot ghost_AoiArm)
        {

            float rand_x = RandInt_x.Next(-1500, 1501);
            float rand_y = RandInt_y.Next(-1500, 1501);
            float rand_z = RandInt_z.Next(200, 1201);

            rand_x = rand_x / 1000;
            rand_y = rand_y / 1000;
            rand_z = rand_z / 1000;

            Console.WriteLine("\nxyz: " + rand_x + " " + rand_y + " " + rand_z);



            ghost_AoiArm.Kinematics.ReturnHomePosition();
            ghost_AoiArm.Kinematics.BaseRotate.SetRz(0);
            ghost_AoiArm.Kinematics.ForwardKinematics();

            ghost_AoiArm.Kinematics.Target[0].SetDOF0();
            ghost_AoiArm.Kinematics.JointTarget[1].SetValue(0, 0, 0, Deg2Rad(-90), 0, 0, 0);
            ghost_AoiArm.Kinematics.Target[1].Position.SetValue(rand_x, rand_y, rand_z);

            //ghost_AoiArm.Kinematics.Target[1].Rotate.SetRy(0);
            ghost_AoiArm.Kinematics.Target[1].Rotate.SetRy(Deg2Rad(90));
            //ghost_AoiArm.Kinematics.Target[1].Rotate.SetRy(Deg2Rad(180));


            //ghost_AoiArm.Kinematics.ObserveOldIK(100);
            ghost_AoiArm.Kinematics.ObserveNewIK(100);
            //ghost_AoiArm.Kinematics.ObserveAltIK(100);
            //ghost_AoiArm.Kinematics.NewInverseKinematics();
            //Console.WriteLine(Rad2Deg(ghost_AoiArm[1, 0].q));
            //Console.WriteLine(Rad2Deg(ghost_AoiArm[1, 3].q));

        }

        // ランダムな目標値を与える実験を行い, データをcsv形式のファイルで出力する関数
        public void RandTargetExperiment_csv(Robot ghost_AoiArm, string fileName, float deg_Ry, string NEW_or_OLD)
        {

            float[] lineTmp = new float[11];
            float normTmp;

            for (int i = 0; i < 1000; i++)      // n数：1000
            {

                // 単位：mm
                float rand_x = RandInt_x.Next(-1500, 1501);
                float rand_y = RandInt_y.Next(-1500, 1501);
                float rand_z = RandInt_z.Next(200, 1201);

                // 単位：mm → m
                rand_x = rand_x / 1000;
                rand_y = rand_y / 1000;
                rand_z = rand_z / 1000;


                ghost_AoiArm.Kinematics.ReturnHomePosition();
                ghost_AoiArm.Kinematics.BaseRotate.SetRz(0);
                ghost_AoiArm.Kinematics.ForwardKinematics();

                ghost_AoiArm.Kinematics.Target[0].SetDOF0();
                ghost_AoiArm.Kinematics.JointTarget[1].SetValue(0, 0, 0, Deg2Rad(-90), 0, 0, 0);
                ghost_AoiArm.Kinematics.Target[1].Position.SetValue(rand_x, rand_y, rand_z);

                ghost_AoiArm.Kinematics.Target[1].Rotate.SetRy(Deg2Rad(deg_Ry));


                if (NEW_or_OLD == "NEW")
                {
                    ghost_AoiArm.Kinematics.NewInverseKinematics(true);
                }
                else if (NEW_or_OLD == "OLD")
                {
                    ghost_AoiArm.Kinematics.OldInverseKinematics(true);
                }
                else
                {
                    Console.WriteLine("Error: The fourth parameter must be \"NEW\" or \"OLD\".");
                    return;
                }


                normTmp = (ghost_AoiArm.Kinematics.Target[1].Position - ghost_AoiArm.Kinematics.Chain[1].pe).Norm;



                // ・(目標姿勢として与えたy軸周りの回転角) ⇒ ファイル名で管理
                // ・目標位置(0, 1, 2)
                // ・減衰更新までの各ステップ数(3, 4, 5, 6)
                // ・合計のステップ数(7)
                // ・目標値と逆運動学解との間の距離が閾値を超えているか否か(8, true = 1 false = 0)
                // ・根本・肘の角度(9, 10)

                lineTmp[0] = rand_x;
                lineTmp[1] = rand_y;
                lineTmp[2] = rand_z;

                for (int tag = 3; tag < ghost_AoiArm.Kinematics.StepsContainerTmp.Length + 3; tag++)
                {
                    lineTmp[tag] = ghost_AoiArm.Kinematics.StepsContainerTmp[tag - 3];
                }

                if (0.05f < normTmp) lineTmp[8] = 1;
                else lineTmp[8] = 0;

                lineTmp[9] = Rad2Deg(ghost_AoiArm[1, 0].q);
                lineTmp[10] = Rad2Deg(ghost_AoiArm[1, 3].q);


                HDataOutputter.WriteFile_csv(fileName, lineTmp);

            }

        }

    }
}
