using System;
using static OpenRCF.HGlobal;

namespace OpenRCF
{
    public class HArmBase    // アームに関する基底クラス
    {

        public Robot AoiArm = new Robot(ExtendedDOF_MobileBase, DOF_CRX7);    // アームのインスタンス化
        protected Cuboid Frame = new Cuboid(0.34f, 0.34f, 0.05f);     // フレームのインスタンス化
        protected HWiper Wiper;     // ワイパーの宣言


        // CRANEX7のリンクベクトルの成分(西村さんの論文からL10をx成分に変更)
        protected const float CRX7_L10_x = 0.18f;
        protected const float CRX7_L10_z = 0.525f;
        protected const float CRX7_L11_z = 0.05f;
        protected const float CRX7_L12_z = 0.05f;
        protected const float CRX7_L13_z = 0.20f;
        protected const float CRX7_L14_z = 0.135f;
        protected const float CRX7_L15_z = 0.115f;
        protected const float CRX7_L16_z = 0.02f;
        protected const float CRX7_L17_z = 0.045f;   // 元は0.02. 実機は0.045


        // ワイパーのリンクベクトルの成分
        protected const float CRX7_Wiper_z = 0.01f;
        protected const float CRX7_Wiper_x = 0.063f;


        protected bool isWiperEnabled = false;    // ワイパーの有無



        // コンストラクタ
        protected HArmBase(bool isBaseOffsetEnabled, byte transparency = 255)
        {

            Wiper = new HWiper(AoiArm, transparency);     // ワイパーのインスタンス化
            AoiArm.SetFloatingJoint6DOF(0.20f, 0.24f, 0.32f);     // 逆運動学の計算の都合上, 移動台車を6自由度で生成(本来は3自由度)


            // 移動台車は3自由度(x, y直進対偶・z回転対偶)で動いてほしいので, z直進対偶・x, y回転対偶をロック
            AoiArm[0, 2].IsLocked = true;
            AoiArm[0, 3].IsLocked = true;
            AoiArm[0, 4].IsLocked = true;


            // ボディのオフセット
            if (isBaseOffsetEnabled) AoiArm.Torso.SetPositionOffset(0, 0, WheelRadius + AoiArm.Torso.SizeZ / 2 + Frame.SizeZ);
            else AoiArm.Torso.SetPositionOffset(0, 0, 2 * WheelRadius + AoiArm.Torso.SizeZ / 2 + Frame.SizeZ);


            // フレームはボディを追従
            Frame.Position.Follow(AoiArm.Torso.Position);
            Frame.Rotate.Follow(AoiArm.Torso.Rotate);


            // フレームのオフセット
            if (isBaseOffsetEnabled) Frame.SetPositionOffset(0, 0, WheelRadius + Frame.SizeZ / 2);
            else Frame.SetPositionOffset(0, 0, 2 * WheelRadius + Frame.SizeZ / 2);


            // 見栄えを良くするための設定
            for (int i = 0; i < ExtendedDOF_MobileBase; i++) AoiArm[0, i].JointObj.Radius = 0;

            Frame.Color.SetRGB(0, 0, 0);
            Frame.Color.Alpha = transparency;
            AoiArm.Torso.Color.SetRGB(229, 138, 55);


            // 基準姿勢時における各関節のリンクベクトルと回転軸ベクトル(西村さんの論文ではy軸正方向が正面だったが, x軸正方向を正面として作り直した)
            if (isBaseOffsetEnabled) AoiArm[1, 0].LinkInit.Set = new float[3] { CRX7_L10_x, 0, CRX7_L10_z - WheelRadius };    // オフセットの分だけリンクを短くする
            else AoiArm[1, 0].LinkInit.Set = new float[3] { CRX7_L10_x, 0, CRX7_L10_z };    // オフセットは設けない
            AoiArm[1, 0].AxisInit.SetUnitVectorZ();   // z軸正方向

            AoiArm[1, 1].LinkInit.Set = new float[3] { 0, 0, CRX7_L11_z };
            AoiArm[1, 1].AxisInit.SetUnitVectorY(-1);     // y軸負方向

            AoiArm[1, 2].LinkInit.Set = new float[3] { 0, 0, CRX7_L12_z };
            AoiArm[1, 2].AxisInit.SetUnitVectorZ();

            AoiArm[1, 3].LinkInit.Set = new float[3] { 0, 0, CRX7_L13_z };
            AoiArm[1, 3].AxisInit.SetUnitVectorY(-1);

            AoiArm[1, 4].LinkInit.Set = new float[3] { 0, 0, CRX7_L14_z };
            AoiArm[1, 4].AxisInit.SetUnitVectorZ();

            AoiArm[1, 5].LinkInit.Set = new float[3] { 0, 0, CRX7_L15_z };
            AoiArm[1, 5].AxisInit.SetUnitVectorY(-1);

            AoiArm[1, 6].LinkInit.Set = new float[3] { 0, 0, CRX7_L16_z };
            AoiArm[1, 6].AxisInit.SetUnitVectorZ();

            AoiArm[1, 7].LinkInit.Set = new float[3] { 0, 0, CRX7_L17_z };


            // 各関節における角度限界値の設定, 角度限界値のデフォは-135 deg～135 deg
            AoiArm[1, 0].JointRangeMin = DyAngle2Rad(500);
            AoiArm[1, 0].JointRangeMax = DyAngle2Rad(3600);

            AoiArm[1, 1].JointRangeMin = DyAngle2Rad(1024);
            AoiArm[1, 1].JointRangeMax = DyAngle2Rad(3072);

            AoiArm[1, 2].JointRangeMin = DyAngle2Rad(500);
            AoiArm[1, 2].JointRangeMax = DyAngle2Rad(3600);

            AoiArm[1, 3].JointRangeMin = DyAngle2Rad(400);
            AoiArm[1, 3].JointRangeMax = DyAngle2Rad(2048);

            AoiArm[1, 4].JointRangeMin = DyAngle2Rad(300);
            AoiArm[1, 4].JointRangeMax = DyAngle2Rad(3800);

            AoiArm[1, 5].JointRangeMin = DyAngle2Rad(1030);
            AoiArm[1, 5].JointRangeMax = DyAngle2Rad(3068);

            AoiArm[1, 6].JointRangeMin = DyAngle2Rad(200);
            AoiArm[1, 6].JointRangeMax = DyAngle2Rad(3072);


            // 関節・リンクの色とサイズ
            for (int i = 0; i < DOF_CRX7 + 1; i++)  // +1は手先の分
            {

                AoiArm[1, i].JointObj.Color.SetRGB(0, 144, 148);
                AoiArm[1, i].LinkObj.Color.SetRGB(0, 0, 0);

                AoiArm[1, i].LinkObj.Radius = 0.03f;
                AoiArm[1, i].JointObj.Radius = 0.03f;

            }
            AoiArm[1, 6].JointObj.Color.SetRGB(135, 206, 235);
            AoiArm.Transparency = transparency;

            AoiArm.Kinematics.Chain[1].EndPoint.Diameter = 0.04f;     // 手先の姿勢を見やすくするため


            // ホームポジション時の関節変位
            AoiArm[1, 0].qHome = DyAngle2Rad(2048);
            AoiArm[1, 1].qHome = DyAngle2Rad(1720);
            AoiArm[1, 2].qHome = DyAngle2Rad(2048);
            AoiArm[1, 3].qHome = DyAngle2Rad(720);
            AoiArm[1, 4].qHome = DyAngle2Rad(2048);
            AoiArm[1, 5].qHome = DyAngle2Rad(2048);
            AoiArm[1, 6].qHome = DyAngle2Rad(2048);


            // 関節変位の初期値はホームポジション準拠
            for (int i = 0; i < DOF_CRX7; i++) AoiArm[1, i].q = AoiArm[1, i].qHome;


            AoiArm.Kinematics.ForwardKinematics();    // 順運動学を解く
            AoiArm.Kinematics.SetTargetsToEffector();     // 目標の位置・姿勢を効果器の位置・姿勢に一致

        }


        // アーム・フレーム・ワイパーを描画する関数. オーバーライド用
        protected void draw(bool isDrawMode)
        {

            if (isDrawMode)
            {
                AoiArm.Draw();
                Frame.Draw();
                if (isWiperEnabled) Wiper.Draw();
            }

        }


        // ワイパーの有無の切り替えを行う関数. オーバーライド用
        protected void switchWiperMode()
        {

            if (!isWiperEnabled)      // 無効⇒有効
            {
                AoiArm.Kinematics.Chain[1].SetPositionOffsetToEffector(CRX7_Wiper_x, 0, CRX7_Wiper_z);    // 終端位置のオフセットを有効化
                AoiArm.Kinematics.Chain[1].SetRotateOffsetToEffector(0, (float)Math.PI / 2, 0);    // 終端姿勢のオフセットを有効化
                AoiArm.Kinematics.ForwardKinematics();      // シミュレーションに反映
                AoiArm.Kinematics.SetTargetsToEffector();  // 目標を効果器に一致
                isWiperEnabled = true;   // 有効化チェック
            }
            else    // 有効⇒無効
            {
                AoiArm.Kinematics.Chain[1].SetPositionOffsetToEffector();    // 終端位置のオフセットを無効化
                AoiArm.Kinematics.Chain[1].SetRotateOffsetToEffector();    // 終端姿勢のオフセットを無効化
                AoiArm.Kinematics.ForwardKinematics();      // シミュレーションに反映
                AoiArm.Kinematics.SetTargetsToEffector();  // 目標を効果器に一致
                isWiperEnabled = false;  // 無効化チェック
            }

        }

    }


    public class HWiper      // ワイパーのクラス
    {

        public Pillar Handle = new Pillar(0.02f, 0.083f);      // 持ち手
        public Cuboid Head = new Cuboid(0.08f, 0.16f, 0.013f);     // ヘッド


        // コンストラクタ
        public HWiper(Robot aoiArm, byte transparency = 255)
        {

            Head.Position.Follow(aoiArm.Kinematics.Chain[1].pe);  // ヘッドは終端位置を追従
            Head.SetPositionOffset(0, 0, -Head.SizeZ / 2);    // 追従のオフセット
            Head.Rotate.Follow(aoiArm.Kinematics.Chain[1].Re);    // ヘッドは終端姿勢を追従
            SetColorRGB(50, 50, 50, transparency);   // 色

            // 持ち手の位置・姿勢はヘッドを追従(オフセットあり)
            Handle.Position.Follow(Head.Position);
            Handle.Rotate.Follow(Head.Rotate);
            Handle.SetPositionOffset(0, 0, -(Handle.Height / 2 + Head.SizeZ / 2));

        }


        // ワイパーの描画を行う関数
        public void Draw()
        {
            Handle.Draw();
            Head.Draw();
        }


        // ワイパーの色を指定する関数
        public void SetColorRGB(byte R, byte G, byte B, byte A = 255)
        {
            Handle.Color.SetRGB(R, G, B);
            Head.Color.SetRGB(R, G, B);
            Handle.Color.Alpha = A;
            Head.Color.Alpha = A;
        }

    }
}
