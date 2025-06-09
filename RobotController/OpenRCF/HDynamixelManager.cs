using System.Threading;
using static OpenRCF.HGlobal;

namespace OpenRCF
{
    public static class HDynamixelManager   // Dynamixelとの通信に関する変数・関数を格納する静的クラス
    {

        // Dynamixelのインスタンス化
        public static SerialDevice.Dynamixel Dynamixel_2229 = new SerialDevice.Dynamixel(1000000);     // アーム
        public static SerialDevice.Dynamixel Dynamixel_1114 = new SerialDevice.Dynamixel(1000000);     // メカナム

        // DynamixelのID
        public static byte[] ID_2229 = new byte[8] { 22, 23, 24, 25, 26, 27, 28, 29 };     // アーム
        public static byte[] ID_1114 = new byte[4] { 14, 11, 12, 13 };     // メカナム, 配置の関係で14を先頭に


        // Dynamixelの電流の返り値を mA に変換するための値. 出典：https://www.besttechnology.co.jp/modules/knowledge/?Dynamixel%20XM540-W270#y2cc93fd
        public static readonly float currentScaringFactor = 2.69f;


        private static bool isTorqueON = false;     // トルクが入っているかどうか
        private static bool isReflect = false;      // シミュレーションへの反映が行われたかどうか
        private static bool isRinkMode = false;         // シミュレーションと実機とのリンクがされているか

        // クラス外読み取り用
        public static bool IsTorqueON { get { return isTorqueON; } }
        public static bool IsReflect { get { return isReflect; } }
        public static bool IsRinkMode { get { return isRinkMode; } }
        public static bool IsPortOpen   // ポートが開いているかどうか
        {

            get
            {
                bool result;

                if (!Dynamixel_2229.IsPortOpen && !Dynamixel_1114.IsPortOpen) result = false;
                else result = true;

                return result;
            }

        }



        // ポートの開閉を切り替える関数
        public static void SwitchPortMode()
        {

            if (!IsPortOpen)
            {
                Dynamixel_2229.PortOpen("COM8");
                Dynamixel_1114.PortOpen("COM10");
            }
            else
            {
                Dynamixel_2229.PortClose();
                Dynamixel_1114.PortClose();
            }

        }


        // トルクのON・OFFを切り替える関数
        public static void SwitchTorqueMode()
        {

            if (!isTorqueON && IsPortOpen)
            {
                Dynamixel_2229.TorqueEnable(ID_2229);
                Dynamixel_1114.TorqueEnable(ID_1114);
                isTorqueON = true;
            }
            else
            {
                Dynamixel_2229.TorqueDisable(ID_2229);
                Dynamixel_1114.TorqueDisable(ID_1114);
                isTorqueON = false;
            }

        }


        // シミュレーションと実機をリンクするかどうかを切り替える関数
        public static void SwitchRinkMode()
        {

            if (!isRinkMode && IsPortOpen) isRinkMode = true;
            else isRinkMode = false;

        }


        // isReflectを初期値falseに戻す関数
        public static void ResetReflectStatus()
        {
            isReflect = false;
        }


        // 実機の角度をシミュレーションに反映する関数
        public static void ReflectRealPositionToVirtual(Robot virtual_AoiArm)
        {

            Dynamixel_2229.RequestPositionReply(ID_2229);

            Thread.Sleep(50);

            for (int i = 0; i < DOF_CRX7; i++)
            {

                if (i == 5)
                {
                    virtual_AoiArm[1, 5].q = DyAngle2Rad(Dynamixel_2229.Position(ID_2229[5]) + 168);   // +168は実機とのズレを補正するためのもの
                }
                else
                {
                    virtual_AoiArm[1, i].q = DyAngle2Rad(Dynamixel_2229.Position(ID_2229[i]));
                }

            }

            virtual_AoiArm.Kinematics.ForwardKinematics();

            isReflect = true;

        }


        // シミュレーションを実機に反映する関数
        public static void ReflectVirtualToReal(HVirtualAoi Virtual)
        {

            int[] DyangleTmp = new int[DOF_CRX7 + 1];   // +1はハンドの分
            int[] DyVelocityTmp = new int[WheelNum];


            for (int i = 0; i < DOF_CRX7; i++)      // アーム
            {

                if (i == 5)
                {
                    DyangleTmp[5] = Rad2DyAngle(Virtual.AoiArm[1, 5].q) - 168;     // -168はズレ補正
                }
                else
                {
                    DyangleTmp[i] = Rad2DyAngle(Virtual.AoiArm[1, i].q);
                }

            }
            DyangleTmp[7] = Virtual.Arm.HandDyAngle;   // ハンド


            for (int i = 0; i < WheelNum; i++)      // メカナム
            {
                DyVelocityTmp[i] = Omega2DyVelocity(Virtual.AoiBase.WheelVelocity[i]);
            }


            Dynamixel_2229.WritePosition(ID_2229, DyangleTmp);
            Dynamixel_1114.WriteVelocity(ID_1114, DyVelocityTmp);

        }

    }
}
