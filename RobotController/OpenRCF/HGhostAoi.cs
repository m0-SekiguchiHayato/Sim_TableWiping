using static OpenRCF.HGlobal;

namespace OpenRCF
{
    public class HGhostAoi : HArmBase   // Aoiのゴースト(運動学担当)のクラス
    {

        private bool isDrawMode = true;    // ゴーストの描画の有無を支配する変数

        // クラス外読み取り用
        public bool IsDrawMode { get { return isDrawMode; } }
        public bool IsWiperEnabled { get { return isWiperEnabled; } }   // from 基底クラス



        // 仮想関節バネを規定値で設定する関数
        public void SetVirtualJointSpringDefault()
        {

            AoiArm.Kinematics.Target[0].SetDOF0();  // 台車は「仮想関節バネ」の方で管理したいので, 台車の「仮想バネ」は無効化

            float[] baseDiag = new float[ExtendedDOF_MobileBase] { 0, 0, 0, 0, 0, 0 };      // 移動台車にはバネをつけない
            float[] armDiag = new float[DOF_CRX7] { Km, 0, 0, Km, 0, 0, 0 };   // アームの1軸, 4軸にバネをつけたい
            AoiArm.Kinematics.Chain[0].SetVirtualJointSpringDiag(baseDiag);
            AoiArm.Kinematics.Chain[1].SetVirtualJointSpringDiag(armDiag);

            AoiArm.Kinematics.JointTarget[0].Priority = false;
            AoiArm.Kinematics.JointTarget[1].SetValue(0, 0, 0, Deg2Rad(-90f), 0, 0, 0);

        }


        // コンストラクタ. 基底クラスのコンストラクタも実行
        public HGhostAoi() : base(false, 128)
        {
            SetVirtualJointSpringDefault();
        }


        // ワイパーの有無の切り替えを行う関数
        public void SwitchWiperMode()
        {
            switchWiperMode();
        }


        // 描画の有無を切り替える関数
        public void SwitchDrawMode()
        {
            isDrawMode = !isDrawMode;
        }

        // ゴーストを描画する関数 from 基底クラス
        public void Draw()
        {
            draw(isDrawMode);
        }

    }

}
