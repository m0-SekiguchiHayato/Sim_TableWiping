using System;
using static OpenRCF.HAoiTrajectry;
using static OpenRCF.HGlobal;

namespace OpenRCF
{
    public class HVirtualAoi     // シミュレーション上の仮想Aoiに関するクラス
    {

        // 移動台車・仮想アームの宣言
        public HVirtualBase Base;
        public HVirtualArm Arm;


        private bool isDrawMode = true;     // 仮想Aoiを描画するかどうか
        private bool isManualMode = true;   // 仮想Aoiを手動で操作するかどうか
        private bool isAutoVelocityMode = false;   // 速度を自動で更新するかどうか
        private bool isGamePadMode = false;     // ゲームパッドで操作するかどうか

        // クラス外読み取り用
        public bool IsDrawMode { get { return isDrawMode; } }
        public bool IsManualMode { get { return isManualMode; } }
        public bool IsAutoVelocityMode { get { return isAutoVelocityMode; } }
        public bool IsGamePadMode { get { return isGamePadMode; } }



        // コンストラクタ
        public HVirtualAoi()
        {
            Base = new HVirtualBase();
            Arm = new HVirtualArm(Base.AoiBase);
        }


        // インデクサ
        public MobileRobot.Mecanum AoiBase { get { return Base.AoiBase; } }
        public Robot AoiArm { get { return Arm.AoiArm; } }


        // 描画の有無を切り替える関数
        public void SwitchDrawMode()
        {
            isDrawMode = !isDrawMode;
        }

        // 仮想Aoiを描画する関数
        public void Draw()
        {
            Base.Draw(isDrawMode);
            Arm.Draw(isDrawMode);
        }


        // 手動操作モードの切り替えを行う関数
        public void SwitchManualMode()
        {
            isManualMode = !isManualMode;
        }

        // 自動作業モードの切り替えを行う関数
        public void SwitchAutoTaskMode()
        {
            isAutoVelocityMode = !isAutoVelocityMode;
        }

        // ゲームパッドモードの切り替えを行う関数
        public void SwitchGamePadMode()
        {
            isGamePadMode = !isGamePadMode;
        }


        // 仮想Aoi全体の動きを司る関数. 並列処理でループさせる
        public void MoveVirtualAoiForLoop()
        {
            
            if (isGamePadMode) GPad.GetState();

            Base.CalcAoiBaseVelocity(isManualMode, isAutoVelocityMode, isGamePadMode);
            Arm.ManageVirtualArmMotion(isManualMode, isGamePadMode, AoiBase);
            Base.AoiBase.InverseKinematics();
            Arm.AoiArm.Kinematics.ForwardKinematics();

        }

    }


    public class HVirtualBase    // 移動台車のクラス
    {

        // 車輪のインスタンス化
        public MobileRobot.Mecanum AoiBase = new MobileRobot.Mecanum(WheelRadius, TreadWidth, Distance);    // 車輪の半径, トレッド幅, 前輪と後輪の軸間距離


        // 移動台車の目標のインスタンス化
        public PrickleBall BaseTarget = new PrickleBall();


        private bool isTaskStarted = false;    // タスクが開始されたかどうか
        private bool isTaskCompleted = true;    // タスクが完了したかどうか
        private int taskStep = 0;   // タスクの現在のステップ

        private float vNorm = VMax;     // 台車の速度. 初期値は最大値に設定


        // コンストラクタ
        public HVirtualBase()
        {
            AoiBase.Position.SetValue(AoiHomePosition);     // 車輪の初期位置(姿勢はデフォルト, x軸正方向が正面)
            BaseTarget.Color.SetDeepPink();
        }


        // 移動台車を描画する関数
        public void Draw(bool isDrawMode)
        {

            if (isDrawMode)
            {
                AoiBase.Draw();
                BaseTarget.Draw();
            }

        }


        // タスク実行時に用いる関数
        public void StartTask()
        {
            isTaskStarted = true;
        }


        // ゲームパッド入力に応じて移動台車の速度を計算する関数. CalcAoiBaseVelocity()で使用
        private void CalcVelocityByGamePadInputs()
        {

            // 入力
            float threshold = 0.5f;
            bool LXP = threshold < GPad.LeftStickX;
            bool LXN = -threshold > GPad.LeftStickX;
            bool LYP = threshold < GPad.LeftStickY;
            bool LYN = -threshold > GPad.LeftStickY;
            bool L = GPad.L;
            bool R = GPad.R;

            // 移動する方向を決めるための変数
            int direction_x = 0;
            int direction_y = 0;
            int direction_theta = 0;


            if (LYP || LXN || LYN || LXP)   // 並進
            {
                if (LYP) direction_x += 1;    // x正方向
                if (LXN) direction_y += 1;    // y正方向
                if (LYN) direction_x -= 1;    // x負方向
                if (LXP) direction_y -= 1;    // y負方向
            }
            else if (L || R)    // 回転
            {
                if (L) direction_theta += 1;    // 左回転
                if (R) direction_theta -= 1;    // 右回転
            }
            else    // 停止
            {
                direction_x = 0;
                direction_y = 0;
                direction_theta = 0;
            }


            // 速度を更新
            float vNormTmp = VMax;
            if (direction_x != 0 && direction_y != 0) vNormTmp = VMax / (float)Math.Sqrt(2);     // どちらも0でないときはVMaxを調整

            AoiBase.Velocity[0] = direction_x * vNormTmp;
            AoiBase.Velocity[1] = direction_y * vNormTmp;
            AoiBase.Velocity[2] = direction_theta * OmegaMax_z;


            // 移動台車の目標は現在位置を追従
            BaseTarget.Position[0] = AoiBase.Position[0];
            BaseTarget.Position[1] = AoiBase.Position[1];
            BaseTarget.Rotate.Set = AoiBase.Rotate.Get;

        }

        // キーボード入力に応じて移動台車の速度を計算する関数. CalcAoiBaseVelocity()で使用
        private void CalcVelocityByKeyboadInputs()
        {

            // キー入力
            bool W = Keyboard.KeyEventW;
            bool A = Keyboard.KeyEventA;
            bool S = Keyboard.KeyEventS;
            bool D = Keyboard.KeyEventD;
            bool Q = Keyboard.KeyEventQ;
            bool E = Keyboard.KeyEventE;

            // 移動する方向を決めるための変数
            int direction_x = 0;
            int direction_y = 0;
            int direction_theta = 0;


            if (W || A || S || D)   // 並進
            {
                if (W) direction_x += 1;    // x正方向
                if (A) direction_y += 1;    // y正方向
                if (S) direction_x -= 1;    // x負方向
                if (D) direction_y -= 1;    // y負方向
            }
            else if (Q || E)    // 回転
            {
                if (Q) direction_theta += 1;    // 左回転
                if (E) direction_theta -= 1;    // 右回転
            }
            else    // 停止
            {
                direction_x = 0;
                direction_y = 0;
                direction_theta = 0;
            }


            // 速度を更新
            float vNormTmp = VMax;
            if (direction_x != 0 && direction_y != 0) vNormTmp = VMax / (float)Math.Sqrt(2);     // どちらも0でないときはVMaxを調整

            AoiBase.Velocity[0] = direction_x * vNormTmp;
            AoiBase.Velocity[1] = direction_y * vNormTmp;
            AoiBase.Velocity[2] = direction_theta * OmegaMax_z;


            // 移動台車の目標は現在位置を追従
            BaseTarget.Position[0] = AoiBase.Position[0];
            BaseTarget.Position[1] = AoiBase.Position[1];
            BaseTarget.Rotate.Set = AoiBase.Rotate.Get;

        }

        // 移動台車の目標位置や速さを設定する関数. CalcAoiBaseVelocity()で使用
        private void SetBaseTargetAndVNorm()
        {

            if (isTaskStarted) isTaskCompleted = false;


            // 台車目標や速さの更新が必要かどうか
            Vector basePosition_ZEquals0 = new Vector(3);
            basePosition_ZEquals0.SetValue(AoiBase.Position[0], AoiBase.Position[1], 0);    // z座標だけ0に
            bool isUpdateNeeded = AoiBase.Velocity.Norm == 0 || (basePosition_ZEquals0 - BaseTarget.Position).Norm < Epsilon_d;   // 速度が0, もしくは台車目標との誤差が十分に小さければtrue


            if (!isTaskCompleted && isUpdateNeeded)   // タスクが完了するまでの間, 更新が必要になったら
            {

                if (taskStep == StepNumMax)     // すべてのステップが完了したら
                {
                    isTaskStarted = false;
                    isTaskCompleted = true;
                    taskStep = 0;
                    vNorm = VMax;
                    return;
                }


                // 台車目標・速度の大きさ・ステップ数を更新
                BaseTarget.Position.SetValue(BaseTars[taskStep][0], BaseTars[taskStep][1], 0);
                BaseTarget.Rotate.SetRz(BaseTars[taskStep][2]);
                vNorm = VNormsAmongBaseTars[taskStep];
                taskStep += 1;

            }

        }

        // 目標位置に向かうように移動台車の速度を計算をする関数. CalcAoiBaseVelocity()で使用
        private void CalcVelocityToTarget()
        {

            // 目標の位置とAoiの位置の偏差を計算
            float targetDist_x = BaseTarget.Position[0] - AoiBase.Position[0];
            float targetDist_y = BaseTarget.Position[1] - AoiBase.Position[1];
            float targetDist = (float)Math.Sqrt(targetDist_x * targetDist_x + targetDist_y * targetDist_y);


            float theta_v = (float)Math.Atan2(targetDist_y, targetDist_x);  // 速度の角度(方向)
            float targetTheta = (float)Math.Atan2(BaseTarget.Rotate[1, 0], BaseTarget.Rotate[1, 1]); // 目標の角度. 移動台車の目標の回転行列からsin, cosを抜き出す
            float aoiTheta = (float)Math.Atan2(AoiBase.Rotate[1, 0], AoiBase.Rotate[1, 1]); // 移動台車の角度. 現在の移動台車の回転行列からsin, cosを抜き出す


            // 目標の角度とAoiの角度の偏差
            float thetaError = targetTheta - aoiTheta;

            if (thetaError > Math.PI)    // 偏差が180 degよりも大きいとき
            {
                thetaError = (float)(-2 * Math.PI + thetaError); // 逆回りに変更
            }
            else if (thetaError < -Math.PI)   // 偏差が-180 degよりも小さいとき
            {
                thetaError = (float)(2 * Math.PI + thetaError);   // 逆回りに変更
            }


            AoiBase.Velocity[2] = Math.Sign(thetaError) * OmegaMax_z;  // 偏差の符号で回転方向を決め, Aoiに一定の角速度を設定

            // 回転している間は並進を停止
            AoiBase.Velocity[0] = 0;
            AoiBase.Velocity[1] = 0;


            if (Math.Abs(thetaError) < Epsilon_theta)    // 角度の偏差が十分に小さければ
            {

                AoiBase.Velocity[2] = 0;   // 回転を停止

                if (targetDist < Epsilon_d)  // 位置の偏差が十分に小さければ
                {
                    // Aoiを停止
                    AoiBase.Velocity[0] = 0;
                    AoiBase.Velocity[1] = 0;
                }
                else
                {
                    // Aoiの速度が目標の方向になるように設定
                    AoiBase.Velocity[0] = (float)(vNorm * Math.Cos(theta_v));
                    AoiBase.Velocity[1] = (float)(vNorm * Math.Sin(theta_v));
                }

            }

        }


        // 移動台車の速度を計算する関数
        public void CalcAoiBaseVelocity(bool isManualMode, bool isAutoVelocityMode, bool isGamePadMode)
        {

            if (isManualMode)   // 手動モードのとき
            {
                if (isGamePadMode) CalcVelocityByGamePadInputs();   // ゲームパッドモードのときは，それにより速度を決定
                else CalcVelocityByKeyboadInputs();  // それ以外はキーボードにより速度を決定
            }
            else
            {
                if (isAutoVelocityMode) SetBaseTargetAndVNorm();    // 自動でタスクを実行させるときは, 目標位置と速さを設定
                else vNorm = VMax;  // それ以外は最大の速さに設定

                CalcVelocityToTarget();     // 目標に向かって速度を指定
            }

        }

    }


    public class HVirtualArm : HArmBase   // 仮想アームのクラス
    {

        private float movementUnit = 0.005f;   // 手動で動かすときの目標位置の単位移動量


        private bool isPantomimeMode = false;    // パントマイムをするかどうか
        private bool isPantomimeSetuped = false;       // パントマイムの初期設定が行われたかどうか


        protected bool isHandOpen = false;  // ハンドが開いているかどうか
        protected int handDyAngle = 2048;     // ハンドの角度指令値


        // クラス外読み取り用
        public bool IsPantomimeMode { get { return isPantomimeMode; } }
        public bool IsWiperEnabled { get { return isWiperEnabled; } }   // from 基底クラス
        public bool IsHandOpen { get { return isHandOpen; } }
        public int HandDyAngle { get { return handDyAngle; } }



        // コンストラクタ. 基底クラスのコンストラクタも実行
        public HVirtualArm(MobileRobot.Mecanum aoiBase) : base(true)
        {

            // アームの根本の位置・姿勢は, 移動台車の位置・姿勢を追従
            AoiArm.Kinematics.BasePosition.Follow(aoiBase.Position);
            AoiArm.Kinematics.BaseRotate.Follow(aoiBase.Rotate);


            // アームの目標値関連
            AoiArm.Kinematics.Target[0].Position.Follow(aoiBase.Position);     // 根元の位置目標は移動台車を追従
            AoiArm.Kinematics.Target[0].Rotate.Follow(aoiBase.Rotate);     // 根元の姿勢目標は移動台車を追従
            AoiArm.Kinematics.Target[1].SetDOF0();    // 手先の目標値は基本なし
            AoiArm.Kinematics.Target[1].Priority = false;   // 手先の優先度は低くしておく. 根本が移動台車から離れるのを防ぐため
            AoiArm.Kinematics.Target[0].Diameter = 0;  // 台車の目標を見えなくする
            AoiArm.Kinematics.Target[1].Diameter = 0;  // 手先の目標を見えなくする

        }


        // 仮想アームを描画する関数 from 基底クラス
        public void Draw(bool isDrawMode)   
        {
            draw(isDrawMode);
        }


        // ワイパーの有無の切り替えを行う関数 from 基底クラス
        public void SwitchWiperMode()   
        {
            switchWiperMode();
        }


        // ハンドの開閉を切り替える関数
        public void SwitchHandMode()
        {

            if (!isHandOpen)    // 閉⇒開
            {
                handDyAngle = 3072;
                isHandOpen = true;
            }
            else　   // 開⇒閉
            {
                handDyAngle = 2000;
                isHandOpen = false;
            }

        }


        // パントマイムモードの切り替えを行う関数
        public void SwitchPantomimeMode()
        {
            isPantomimeMode = !isPantomimeMode;
            isPantomimeSetuped = false;   // 初期設定の履歴をリセット
        }

        // パントマイムの初期設定を行う関数
        private void PantomimeSetup()
        {

            if (isPantomimeMode)
            {
                AoiArm.Kinematics.Target[1].SetDOF6();     // 手先の目標値を6自由度で指定
                AoiArm.Kinematics.SetTargetsToEffector();  // 現在の手先位置に目標を指定
                AoiArm.Kinematics.Target[1].Diameter = 0.05f;  // 手先の目標をデフォルトサイズに
                isPantomimeSetuped = true;    // 設定済チェック
            }
            else
            {
                AoiArm.Kinematics.Target[1].SetDOF0();     // 手先の目標値無し
                AoiArm.Kinematics.Target[1].Diameter = 0;  // 手先の目標を見えなくする
                isPantomimeSetuped = true;    // 設定済チェック
            }

        }


        // ゲームパッド入力に応じて手先の目標位置を移動する関数. ManageVirtualArmMotion()で使用
        private void MoveArmByGamePadInputs(MobileRobot.Mecanum aoiBase)
        {

            // 入力
            bool Front = GPad.Up;
            bool Back = GPad.Down;
            bool Left = GPad.Left;
            bool Right = GPad.Right;
            bool Up = GPad.Y;
            bool Down = GPad.A;

            // 進む方向を決めるための変数. sinθ, cosθの符号決めに用いる
            int directionFB = 0;    // +1:Front -1:Back
            int directionLR = 0;    // +1:Left  -1:Right
            int directionUD = 0;    // +1:Up    -1:Down

            // 移動台車の回転に基づいたsinθ, cosθ
            float sin = aoiBase.Rotate[1, 0];
            float cos = aoiBase.Rotate[1, 1];


            if (Front) directionFB += 1;     // 前. (x, y) = (movementUnit * cosθ, movementUnit * sinθ)
            if (Left) directionLR += 1;     // 左. (x, y) = (movementUnit * (-sinθ), movementUnit * cosθ)
            if (Back) directionFB -= 1;     // 後. (x, y) = (movementUnit * (-cosθ), movementUnit * (-sinθ))
            if (Right) directionLR -= 1;     // 右. (x, y) = (movementUnit * sinθ, movementUnit * (-cosθ))
            if (Up) directionUD += 1;     // 上. +z方向に目標位置をずらす
            if (Down) directionUD -= 1;     // 下. -z方向に目標位置をずらす


            // 目標位置を更新
            float unitTmp = movementUnit;
            bool isFBZero = directionFB == 0;    // directionFBが0ならtrue, それ以外ならfalse
            bool isLRZero = directionLR == 0;    // directionLRが0ならtrue, それ以外ならfalse
            bool isUDZero = directionUD == 0;    // directionUDが0ならtrue, それ以外ならfalse

            if (!isFBZero && !isLRZero && !isUDZero) unitTmp = movementUnit / (float)Math.Sqrt(3);    // すべて0でないときは1/√3倍
            else if ((!isFBZero && !isLRZero) || (!isLRZero && !isUDZero) || (!isUDZero && !isFBZero)) unitTmp = movementUnit / (float)Math.Sqrt(2);    // いずれか1つだけ0のときは1/√2倍

            AoiArm.Kinematics.Target[1].Position[0] += unitTmp * (directionFB * cos + directionLR * (-sin));
            AoiArm.Kinematics.Target[1].Position[1] += unitTmp * (directionFB * sin + directionLR * cos);
            AoiArm.Kinematics.Target[1].Position[2] += unitTmp * directionUD;

        }

        // キーボード入力に応じて手先の目標位置を移動する関数. ManageVirtualArmMotion()で使用
        private void MoveArmByKeybordInputs(MobileRobot.Mecanum aoiBase)
        {

            // キー入力
            bool I = Keyboard.KeyEventI;
            bool J = Keyboard.KeyEventJ;
            bool K = Keyboard.KeyEventK;
            bool L = Keyboard.KeyEventL;
            bool U = Keyboard.KeyEventU;
            bool O = Keyboard.KeyEventO;

            // 進む方向を決めるための変数. sinθ, cosθの符号決めに用いる
            int directionFB = 0;    // +1:Front -1:Back
            int directionLR = 0;    // +1:Left  -1:Right
            int directionUD = 0;    // +1:Up    -1:Down

            // 移動台車の回転に基づいたsinθ, cosθ
            float sin = aoiBase.Rotate[1, 0];
            float cos = aoiBase.Rotate[1, 1];


            if (I) directionFB += 1;     // 前. (x, y) = (movementUnit * cosθ, movementUnit * sinθ)
            if (J) directionLR += 1;     // 左. (x, y) = (movementUnit * (-sinθ), movementUnit * cosθ)
            if (K) directionFB -= 1;     // 後. (x, y) = (movementUnit * (-cosθ), movementUnit * (-sinθ))
            if (L) directionLR -= 1;     // 右. (x, y) = (movementUnit * sinθ, movementUnit * (-cosθ))
            if (U) directionUD += 1;     // 上. +z方向に目標位置をずらす
            if (O) directionUD -= 1;     // 下. -z方向に目標位置をずらす


            // 目標位置を更新
            float unitTmp = movementUnit;
            bool isFBZero = directionFB == 0;    // directionFBが0ならtrue, それ以外ならfalse
            bool isLRZero = directionLR == 0;    // directionLRが0ならtrue, それ以外ならfalse
            bool isUDZero = directionUD == 0;    // directionUDが0ならtrue, それ以外ならfalse

            if (!isFBZero && !isLRZero && !isUDZero) unitTmp = movementUnit / (float)Math.Sqrt(3);    // すべて0でないときは1/√3倍
            else if ((!isFBZero && !isLRZero) || (!isLRZero && !isUDZero) || (!isUDZero && !isFBZero)) unitTmp = movementUnit / (float)Math.Sqrt(2);    // いずれか1つだけ0のときは1/√2倍

            AoiArm.Kinematics.Target[1].Position[0] += unitTmp * (directionFB * cos + directionLR * (-sin));
            AoiArm.Kinematics.Target[1].Position[1] += unitTmp * (directionFB * sin + directionLR * cos);
            AoiArm.Kinematics.Target[1].Position[2] += unitTmp * directionUD;

        }


        // 仮想アームの動きを管理する関数
        public void ManageVirtualArmMotion(bool isManualMode, bool isGamePadMode, MobileRobot.Mecanum aoiBase)
        {

            float threshold = 0.5f;
            bool LXP = threshold < GPad.LeftStickX;
            bool LXN = -threshold > GPad.LeftStickX;
            bool LYP = threshold < GPad.LeftStickY;
            bool LYN = -threshold > GPad.LeftStickY;
            bool L = GPad.L;
            bool R = GPad.R;

            bool isMove = LXP || LXN || LYP || LYN || L || R;


            if (!isPantomimeSetuped) PantomimeSetup();  // パントマイムの初期設定をしていなければ実行

            if (isManualMode)   // 手動モードのとき
            {

                if (isGamePadMode)  // ゲームパッドモードのとき
                {

                    if (isMove)
                    {
                        isPantomimeMode = false;
                        isPantomimeSetuped = false;
                    }
                    else
                    {
                        isPantomimeMode = true;
                        isPantomimeSetuped = false;
                        MoveArmByGamePadInputs(aoiBase);     // 手先を移動
                    }
                    
                }
                else MoveArmByKeybordInputs(aoiBase);   // それ以外はキーボードにより移動

            }

            if (isPantomimeMode) AoiArm.Kinematics.InverseKinematics(); // パントマイムモードの時は逆運動学を解く

        }

    }

}
