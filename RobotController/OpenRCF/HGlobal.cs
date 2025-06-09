using System;

namespace OpenRCF
{
    public static class HGlobal     // グローバルに用いたい定数・関数などを格納する静的クラス
    {

        public static GamePad GPad = new GamePad();


        public const int ExtendedDOF_MobileBase = 6;   // 拡張した移動台車の自由度
        public const int DOF_CRX7 = 7;   // CRANEX7の自由度

        public const float ArmLengthMax = 1.32f;   // 地面からの距離も含めたアームの全長. バネ定数の計算に使用する
        public const float Kf = 1;  // 並進バネ定数
        public const float Km = ArmLengthMax * ArmLengthMax / (float)Math.PI;   // 回転バネ定数

        public const int WheelNum = 4;  // 車輪の数
        public const float WheelRadius = 0.05f; // 車輪の半径
        public const float TreadWidth = 0.27f;  // 車輪のトレッド幅
        public const float Distance = 0.25f;    // 前輪と後輪の軸間距離


        // 移動台車の速度について
        // 車輪用Dynamixelの回転速度の最大値は, 実測値で28.00 rev/min
        // 車輪の角速度の最大値は ωMax = 28.00 * 2PI / 60 rad/s
        // 車輪が発生する速度(移動台車の速度)の最大値は　VMax = r * ωmax ≒ 0.147 m/s
        // 並進と回転を同時にすると最大速度を維持できない(実機とのズレが大きくなる)ので基本は分離
        // Dynamixelの速度指令値は ±122 くらいが限界

        public const float VMax = 0.147f;  // 移動台車の並進速度の最大値
        public const float OmegaMax_z = 0.56f;     // 移動台車の回転角速度(z軸周り)の最大値


        public const uint PeriodMs = 50;   // 描画などの周期. MobileRobot.ObjectBaseのSampringTimeMs
        public const float Epsilon_d = VMax * PeriodMs / 1000;    // 位置の偏差が十分に小さいかを確かめるための, 微小な値
        public const float Epsilon_theta = (float)Math.PI / 180;    // 角度の偏差が十分に小さいかを確かめるための, 微小な値

        public static readonly float[] AoiHomePosition = new float[3] { 0, 0, WheelRadius };    // Aoiのホームポジション



        // radをdegに変換する関数
        public static float Rad2Deg(float rad)
        {
            return rad * 180 / (float)Math.PI;
        }

        // degをradに変換する関数
        public static float Deg2Rad(float deg)
        {
            return deg * (float)Math.PI / 180;
        }

        // Dynamixelの角度指令値ををradに変換する関数
        public static float DyAngle2Rad(int dyAngle)
        {
            return -(float)Math.PI + dyAngle * (float)Math.PI / 2048;    //openRCFは0 deg基準、Dynamixelは180 deg基準だから
        }

        // radをDynamixelの角度指令値に変換する関数
        public static int Rad2DyAngle(float rad)
        {
            return (int)((2048 + rad * 2048 / (float)Math.PI) % 4096);
        }

        // 角速度をDynamixelの速度指令値に変換する関数
        public static int Omega2DyVelocity(float omega)
        {
            //omega = 2πN/60
            //Dynamixel velocity：-1023~1023
            //Dynamixel rpm：-234.27~234.27 [rpm] → N

            //omega = -24.53~24.53 [rad/s]

            //ratio = 1023/24.53 = 41.7


            return (int)(41.7f * omega);
        }



        public class HTable      // 机のクラス
        {

            public Cuboid Tabletop = new Cuboid(0.7f, 1.4f, 0.02f);     // 天板
            public Cuboid[] Legs = new Cuboid[4];   // 脚


            // 定数
            private float sizeXHalf;
            private float sizeYHalf;
            private float z_Offset;

            // クラス外読み取り用
            public float SizeXHalf { get { return sizeXHalf; } }
            public float SizeYHalf { get { return sizeYHalf; } }
            public float Z_Offset { get { return z_Offset; } }



            // コンストラクタ
            public HTable()
            {

                // 定数の設定
                sizeXHalf = Tabletop.SizeX / 2;
                sizeYHalf = Tabletop.SizeY / 2;
                z_Offset = 0.73f - Tabletop.SizeZ / 2;


                // 天板の設定
                Tabletop.Position.SetValue(0.6f + sizeXHalf, -0.5f, z_Offset);   // 位置
                Tabletop.Color.SetBrown();  // 色


                for (int i = 0; i < Legs.Length; i++)   // 脚の設定
                {

                    Legs[i] = new Cuboid(0.03f, 0.03f, 0.71f);      // サイズ

                    // 天板の位置・姿勢を追従
                    Legs[i].Position.Follow(Tabletop.Position);
                    Legs[i].Rotate.Follow(Tabletop.Rotate);

                    Legs[i].Color.SetBrown();   // 色

                }

                // 脚のオフセット
                Legs[0].SetPositionOffset(Tabletop.SizeX / 2 - Legs[0].SizeX / 2 - 0.05f, Tabletop.SizeY / 2 - Legs[0].SizeY / 2, -Tabletop.SizeZ / 2 - Legs[0].SizeZ / 2);
                Legs[1].SetPositionOffset(-Tabletop.SizeX / 2 + Legs[1].SizeX / 2 + 0.05f, Tabletop.SizeY / 2 - Legs[1].SizeY / 2, -Tabletop.SizeZ / 2 - Legs[1].SizeZ / 2);
                Legs[2].SetPositionOffset(-Tabletop.SizeX / 2 + Legs[2].SizeX / 2 + 0.05f, -Tabletop.SizeY / 2 + Legs[2].SizeY / 2, -Tabletop.SizeZ / 2 - Legs[2].SizeZ / 2);
                Legs[3].SetPositionOffset(Tabletop.SizeX / 2 - Legs[3].SizeX / 2 - 0.05f, -Tabletop.SizeY / 2 + Legs[3].SizeY / 2, -Tabletop.SizeZ / 2 - Legs[3].SizeZ / 2);


            }


            // 天板の位置へのインデクサ
            public Vector Position
            {
                get { return Tabletop.Position; }
                set { Tabletop.Position = value; }
            }


            // 机の描画を行う関数
            public void Draw()
            {

                Tabletop.Draw();

                for (int i = 0; i < Legs.Length; i++)
                {
                    Legs[i].Draw();
                }

            }

        }

    }
}
