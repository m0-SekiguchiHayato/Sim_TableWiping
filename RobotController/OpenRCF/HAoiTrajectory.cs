using System;
using static OpenRCF.HGlobal;

namespace OpenRCF
{
    public static class HAoiTrajectry   // Aoiの軌道生成に関する静的クラス
    {

        private const float Amplitude = 0.1f;   // 机拭き動作の振幅
        private const int RoundTripsNum = 10;   // 机拭き動作の往復回数
        private const int SplitNum = 4;  // 各目標値間の分割数
        private const int StepNum_OneTable = SplitNum * (RoundTripsNum + 1); // 一つのテーブルに対するステップ数
        public const int StepNumMax = SplitNum * (RoundTripsNum + 1);  // ステップ数の最大値


        // 格納用
        private static RotationMatrix Rz_tv;     // 目標のベクトルをx軸と平行にする回転行列. 各要素の回転補正に使用する
        private static Vector initialBasePosition = new Vector(3);   // 台車の初期位置
        private static Vector targetVector = new Vector(3);  // 台車目標となるベクトル
        private static Vector[] nodePositionRead_Arm;   // 手先目標の節の位置
        private static Vector[] nodePositionRead_Base;  // 台車目標の節の位置
        private static Vector referencePosition = new Vector(3);     // 並進の基準になるベクトル
        private static float angleToTargetVector;   // 台車目標となるベクトルがx軸となす角
        private static float[] baseAngleTargetsRead;    // 台車回転目標. 1つの線分につき1つ
        private static int nearestNodeTag;   // 最近傍点の番号
        private static bool isBackTarget, isForthTarget;    // どちらのベクトルが目標か
        private static bool isBackDomainOld, isForthDomainOld, isDoubleDomainOld, isNormalDomainOld;   // 領域記録用

        public static Vector[] ballPosition = new Vector[StepNumMax + 1];


        // 関数の引数を記録する変数
        private static Vector initialBasePositionRead = new Vector(3);  // 台車の初期位置


        // 軌道表示用のゴースト
        private const int TrajectoryGhostsNum = StepNumMax;  // ステップ数と同じ数だけゴーストを用意
        private static HGhostAoi TrajectoryGhost = new HGhostAoi();


        private static bool isTrajectoryDrawMode = false;   // 軌道を表示するかどうか
        private static Vector[] baseTars;   // 移動台車の目標位置・角度. x・y・thetaを格納
        private static Vector vNormsAmongBaseTars; // 移動台車の目標間毎の速度を格納

        // クラス外読み取り用
        public static bool IsTrajectoryDrawMode { get { return isTrajectoryDrawMode; } }
        public static Vector[] BaseTars { get { return baseTars; } }
        public static Vector VNormsAmongBaseTars { get { return vNormsAmongBaseTars; } }



        // 仮想Aoiをゴーストに向かって(アームのみ)動かす関数
        public static void MoveVirtualArmToGhost(Robot virtual_AoiArm, Robot ghost_AoiArm, int timeMs)
        {

            if (!virtual_AoiArm.Trajectory.IsMoveCompleted)    // 動いていたら警告・終了
            {
                Console.WriteLine("Error: The object is moving now.");
                return;
            }


            Vector qTarTmp = new Vector(DOF_CRX7);

            for (int i = 0; i < DOF_CRX7; i++)      // 目標を格納
            {
                qTarTmp[i] = virtual_AoiArm[1, i].qTar;
                virtual_AoiArm[1, i].qTar = ghost_AoiArm[1, i].q;
            }


            // 動作を再生し, 動かし終わったら目標値をもとに戻す
            virtual_AoiArm.Trajectory.MoveToJointTargets(timeMs, 1);
            Parallel.RunWait(() => { for (int i = 0; i < DOF_CRX7; i++) virtual_AoiArm[1, i].qTar = qTarTmp[i]; }, virtual_AoiArm.Trajectory.IsMoveCompleted, timeMs + 200);

        }


        private static void SetBaseTarget_LineSegment(Robot ghost_AoiArm)
        {

            Vector nearestNeighborPoint = new Vector(3);     // 最近傍点(節)
            Vector nearestNeighborPoint_Back = new Vector(3);    // 最近傍点後方側の節
            Vector nearestNeighborPoint_Forth = new Vector(3);   // 最近傍点前方側の節
            Vector route_Back = new Vector(3);   // 最近傍点後方側のベクトル
            Vector route_Forth = new Vector(3);  // 最近傍点前方側のベクトル
            Vector basePositionFromNNPoint = new Vector(3);  // 最近傍点から台車位置へのベクトル

            float distMin = 0;
            bool isFirstTag, isLastTag;
            bool isBackDomain, isForthDomain, isDoubleDomain, isNormalDomain;   // どの領域にいるか

            isFirstTag = isLastTag = false;
            isForthDomain = isBackDomain = isDoubleDomain = isNormalDomain = false;


            // 初期の座標系で最近傍点を走査
            for (int i = 0; i < nodePositionRead_Base.Length; i++)
            {

                float distTmp = (nodePositionRead_Base[i] - initialBasePosition).Norm;

                if (i == 0)
                {
                    distMin = distTmp;
                }
                else if (distTmp < distMin)
                {
                    distMin = distTmp;
                    nearestNodeTag = i;
                }

            }

            if (nearestNodeTag == 0) isFirstTag = true;
            else if (nearestNodeTag == nodePositionRead_Base.Length - 1) isLastTag = true;


            // 最近傍点関連の記録と領域判断
            float dotProduct_Back = 0, dotProduct_Forth = 0;

            nearestNeighborPoint.Set = nodePositionRead_Base[nearestNodeTag].Get;
            basePositionFromNNPoint.Set = (initialBasePosition - nearestNeighborPoint).Get;

            if (!isFirstTag)
            {
                nearestNeighborPoint_Back.Set = nodePositionRead_Base[nearestNodeTag - 1].Get;
                route_Back.Set = (nearestNeighborPoint - nearestNeighborPoint_Back).Get;
                dotProduct_Back = (-route_Back).Trans * basePositionFromNNPoint;    // 最近傍点後方側の逆ベクトルと, その始点から台車位置へのベクトルの内積
            }
            if (!isLastTag)
            {
                nearestNeighborPoint_Forth.Set = nodePositionRead_Base[nearestNodeTag + 1].Get;
                route_Forth.Set = (nearestNeighborPoint_Forth - nearestNeighborPoint).Get;
                dotProduct_Forth = route_Forth.Trans * basePositionFromNNPoint;   // 最近傍点前方側のベクトルと, その始点から台車位置へのベクトルの内積
            }


            if (isFirstTag)
            {
                isBackDomain = dotProduct_Forth < 0;
                isForthDomain = 0 <= dotProduct_Forth;
            }
            else if (isLastTag)
            {
                isBackDomain = 0 <= dotProduct_Back;
                isForthDomain = dotProduct_Back < 0;
            }
            else
            {
                isBackDomain = 0 <= dotProduct_Back && dotProduct_Forth < 0;       // 後方側の領域のみ
                isForthDomain = dotProduct_Back < 0 && 0 <= dotProduct_Forth;      // 前方側の領域のみ
                isDoubleDomain = 0 <= dotProduct_Back && 0 <= dotProduct_Forth;    // 上二つの領域が重なる領域
                isNormalDomain = dotProduct_Back < 0 && dotProduct_Forth < 0;     // どこにも属さない領域
            }


            // 台車のy, theta_zにバネをつける
            float[] baseDiag = new float[ExtendedDOF_MobileBase] { 0, Kf, 0, 0, 0, Km };
            ghost_AoiArm.Kinematics.Chain[0].SetVirtualJointSpringDiag(baseDiag);

            isBackTarget = isForthTarget = false;
            if (isBackDomain)   // 初期位置が後方側の領域のとき
            {

                // 台車y座標の目標値は0(x軸)に設定し, 優先度を消去
                ghost_AoiArm.Kinematics.JointTarget[0][1] = 0;
                ghost_AoiArm.Kinematics.JointTarget[0].Priority = false;


                if (isFirstTag)     // 最近傍点が始点の場合
                {

                    // 前方側のベクトルを目標ベクトルに設定
                    targetVector.Set = route_Forth.Get;
                    isForthTarget = true;
                    referencePosition.Set = nearestNeighborPoint.Get;   // 基準は最近傍点

                    // 連続でこの領域でなければ, 台車のx, y, theta_zにバネをつけて目標を始点にし, 優先度を付与
                    if (!isBackDomainOld)
                    {
                        baseDiag = new float[ExtendedDOF_MobileBase] { Kf, Kf, 0, 0, 0, Km };
                        ghost_AoiArm.Kinematics.Chain[0].SetVirtualJointSpringDiag(baseDiag);
                        ghost_AoiArm.Kinematics.JointTarget[0][0] = 0;
                        ghost_AoiArm.Kinematics.JointTarget[0].Priority = true;
                    }

                }
                else
                {
                    // 後方側のベクトルを目標ベクトルに設定
                    targetVector.Set = route_Back.Get;
                    isBackTarget = true;
                    referencePosition.Set = nearestNeighborPoint_Back.Get;  // 基準は後方側の節
                }
                Console.WriteLine(nearestNodeTag + "B");

            }
            else if (isForthDomain)  // 初期位置が前方側の領域のとき
            {

                // 台車y座標の目標値は0(x軸)に設定し, 優先度を消去
                ghost_AoiArm.Kinematics.JointTarget[0][1] = 0;
                ghost_AoiArm.Kinematics.JointTarget[0].Priority = false;


                if (isLastTag)      // 最近傍点が終点の場合
                {

                    // 後方側のベクトルを目標ベクトルに設定
                    targetVector.Set = route_Back.Get;
                    isBackTarget = true;
                    referencePosition.Set = nearestNeighborPoint_Back.Get;  // 基準は後方側の節

                    // 連続でこの領域でなければ, 台車のx, y, theta_zにバネをつけて目標を終点にし, 優先度を付与
                    if (!isForthDomainOld)
                    {
                        baseDiag = new float[ExtendedDOF_MobileBase] { Kf, Kf, 0, 0, 0, Km };
                        ghost_AoiArm.Kinematics.Chain[0].SetVirtualJointSpringDiag(baseDiag);
                        ghost_AoiArm.Kinematics.JointTarget[0][0] = targetVector.Norm;
                        ghost_AoiArm.Kinematics.JointTarget[0].Priority = true;
                    }

                }
                else
                {
                    // 前方側のベクトルを目標ベクトルに設定
                    targetVector.Set = route_Forth.Get;
                    isForthTarget = true;
                    referencePosition.Set = nearestNeighborPoint.Get;   // 基準は最近傍点
                }
                Console.WriteLine(nearestNodeTag + "F");

            }
            else if (isDoubleDomain)  // 初期位置が両方の領域のとき
            {

                RotationMatrix Rz90 = new RotationMatrix(0, 0, Deg2Rad(90));
                Vector normalVector_Back = new Vector(3);
                Vector normalVector_Forth = new Vector(3);

                normalVector_Back.Set = (Rz90 * route_Back).Get;
                normalVector_Forth.Set = (Rz90 * route_Forth).Get;

                float distToBack = (float)Math.Abs(normalVector_Back.Trans * basePositionFromNNPoint) / normalVector_Back.Norm;
                float distToForth = (float)Math.Abs(normalVector_Forth.Trans * basePositionFromNNPoint) / normalVector_Back.Norm;


                // 近い方のベクトルを目標ベクトルに設定
                if (distToBack < distToForth)
                {
                    targetVector.Set = route_Back.Get;
                    isBackTarget = true;
                    referencePosition.Set = nearestNeighborPoint_Back.Get;  // 基準は後方側の節
                }
                else
                {
                    targetVector.Set = route_Forth.Get;
                    isForthTarget = true;
                    referencePosition.Set = nearestNeighborPoint.Get; // 基準は最近傍点
                }


                // 台車y座標の目標値は0(x軸)に設定し, 優先度を消去
                ghost_AoiArm.Kinematics.JointTarget[0][1] = 0;
                ghost_AoiArm.Kinematics.JointTarget[0].Priority = false;
                Console.WriteLine(nearestNodeTag + "D");

            }
            else if (isNormalDomain)    // 初期位置がどちらの領域でもないとき
            {

                // ここだけ台車のx, y, theta_zにバネをつける
                baseDiag = new float[ExtendedDOF_MobileBase] { Kf, Kf, 0, 0, 0, Km };
                ghost_AoiArm.Kinematics.Chain[0].SetVirtualJointSpringDiag(baseDiag);


                // 前方側のベクトルを目標ベクトルに設定
                targetVector.Set = route_Forth.Get;
                referencePosition.Set = nearestNeighborPoint_Forth.Get; // 基準は最近傍点


                // 台車の目標値は原点に設定し, 優先度を付与
                ghost_AoiArm.Kinematics.JointTarget[0][0] = 0;
                ghost_AoiArm.Kinematics.JointTarget[0][1] = 0;
                ghost_AoiArm.Kinematics.JointTarget[0].Priority = true;


                // 2回連続でこの領域のときは優先度を消去
                if (isNormalDomainOld) ghost_AoiArm.Kinematics.JointTarget[0].Priority = false;
                Console.WriteLine(nearestNodeTag + "D");

            }

            angleToTargetVector = (float)Math.Atan2(targetVector.Normalize[1], targetVector.Normalize[0]);
            Rz_tv = new RotationMatrix(0, 0, -angleToTargetVector);


            // 領域を記憶
            isBackDomainOld = isBackDomain;
            isForthDomainOld = isForthDomain;
            isDoubleDomainOld = isDoubleDomain;
            isNormalDomainOld = isNormalDomain;

        }

        private static void KinematicsSetup(Robot ghost_AoiArm)
        {

            // 目標の設定
            SetBaseTarget_LineSegment(ghost_AoiArm);


            // 移動台車の回転角
            float baseAngleTarget;
            if (isBackTarget) baseAngleTarget = baseAngleTargetsRead[nearestNodeTag - 1];
            else if (isForthTarget) baseAngleTarget = baseAngleTargetsRead[nearestNodeTag];
            else baseAngleTarget = (baseAngleTargetsRead[nearestNodeTag - 1] + baseAngleTargetsRead[nearestNodeTag]) / 2;

            RotationMatrix Rz_Base = new RotationMatrix(0, 0, baseAngleTarget);   // 台車のz軸周りの回転を指定する回転行列


            // 目標の設定
            RotationMatrix Ry180 = new RotationMatrix(0, Deg2Rad(180), 0);

            float[] armDiag = new float[DOF_CRX7] { Km, 0, 0, 0, 0, 0, 0 };   // アームの1軸にバネをつける
            ghost_AoiArm.Kinematics.Chain[1].SetVirtualJointSpringDiag(armDiag);

            ghost_AoiArm.Kinematics.Target[0].SetDOF0();   // 台車は「仮想関節バネ」の方で管理したいので, 台車の「仮想バネ」は無効化
            ghost_AoiArm.Kinematics.Target[1].Rotate.Set = (Rz_Base * Ry180).Get;  // 手先目標は下向きにしてから, 台車の回転で補正
            ghost_AoiArm.Kinematics.JointTarget[0][5] = baseAngleTarget - angleToTargetVector;     // z軸周りの回転角度の目標
            ghost_AoiArm.Kinematics.JointTarget[1][0] = 0;     // 1軸の目標は0 rad


            // 並進・回転補正
            Vector correctedBasePosition = new Vector(3)    // 補正後の台車位置
            {
                Set = (Rz_tv * (ghost_AoiArm.Kinematics.Chain[0].pe - referencePosition)).Get
            };


            // 補正後の値に上書き
            ghost_AoiArm[0, 0].q = correctedBasePosition[0];
            ghost_AoiArm[0, 1].q = correctedBasePosition[1];
            ghost_AoiArm[0, 5].q -= angleToTargetVector;
            ghost_AoiArm.Kinematics.Target[1].Rotate.Set = (Rz_tv * ghost_AoiArm.Kinematics.Target[1].Rotate).Get;

        }

        private static void MakeTableWipingTrajectory(Robot virtual_AoiArm, Robot ghost_AoiArm, int wipingVectorTag = 0, int firstStep = 0)
        {

            KinematicsSetup(ghost_AoiArm);

            bool isOddTurning = true;   // 切り返しが奇数回目かどうか
            Vector armTargetPositionOld = new Vector(3);       // 手先目標の位置を保存するための変数
            Vector vectorToNextCorner = new Vector(3);   // 次の角の方向を示すベクトルを保存するための変数


            for (int k = 0; k < StepNum_OneTable + 1; k++) ballPosition[k] = new Vector(3);

            // 軌道生成
            for (int k = firstStep; k < StepNum_OneTable + firstStep; k++)
            {

                if (k == 0) ballPosition[0].Set = nodePositionRead_Arm[0].Get;

                // 机拭きの目標関連
                Vector correctedStartPosition_Arm = Rz_tv * (nodePositionRead_Arm[wipingVectorTag] - referencePosition);    // 始点(並進・回転補正済)
                Vector correctedEndPosition_Arm = Rz_tv * (nodePositionRead_Arm[wipingVectorTag + 1] - referencePosition);      // 終点(並進・回転補正済)
                Vector wipingVector = correctedEndPosition_Arm - correctedStartPosition_Arm;    // 机拭き軌道の中心線

                float cornerDist = wipingVector.Norm / RoundTripsNum;     // ある角から次の角までに進む距離
                float angleToWipingVector = (float)Math.Atan2(wipingVector.Normalize[1], wipingVector.Normalize[0]);  // 中心線がx軸となす角
                RotationMatrix Rz_wv = new RotationMatrix(0, 0, angleToWipingVector);  // 軌道の中心線に向かう回転行列


                int offset_k = k - firstStep;   // 内部処理は0番からスタート

                if (offset_k < SplitNum)   // 最初の折り返しまでの間
                {

                    if (offset_k == 0) armTargetPositionOld.Set = correctedStartPosition_Arm.Get;    // 最初, 手先目標の基準位置はスタートに

                    vectorToNextCorner.SetValue(cornerDist / (2 * SplitNum), Amplitude / (2 * SplitNum), 0);      // デフォルト(SplitNumで割るだけ)の半分の長さに
                    vectorToNextCorner = Rz_wv * vectorToNextCorner;     // 方向補正

                }
                else if (SplitNum <= offset_k && offset_k < SplitNum * RoundTripsNum)     // 最初の折り返しから最後の折り返しまでの間
                {

                    if (offset_k % SplitNum == 0)      // 折り返し時だけ
                    {
                        if ((offset_k / SplitNum) % 2 == 1) isOddTurning = true;    // 奇数回目の折り返し
                        else isOddTurning = false;   // 偶数回目の折り返し
                    }

                    if (isOddTurning) vectorToNextCorner.SetValue(cornerDist / SplitNum, -Amplitude / SplitNum, 0);   // 奥⇒手前
                    else vectorToNextCorner.SetValue(cornerDist / SplitNum, Amplitude / SplitNum, 0);     // 手前⇒奥
                    vectorToNextCorner = Rz_wv * vectorToNextCorner;     // 方向補正

                }
                else if (SplitNum * RoundTripsNum <= offset_k && offset_k < StepNumMax)   // 最後の折り返し以降
                {

                    if (offset_k == SplitNum * RoundTripsNum)  // 最初だけ
                    {
                        if (RoundTripsNum % 2 == 1) isOddTurning = true;    // 奇数回目の折り返し
                        else isOddTurning = false;   // 偶数回目の折り返し
                    }

                    if (isOddTurning) vectorToNextCorner.SetValue(cornerDist / (2 * SplitNum), -Amplitude / (2 * SplitNum), 0);      // デフォルト(SplitNumで割るだけ)の半分の長さに
                    else vectorToNextCorner.SetValue(cornerDist / (2 * SplitNum), Amplitude / (2 * SplitNum), 0);      // デフォルト(SplitNumで割るだけ)の半分の長さに
                    vectorToNextCorner = Rz_wv * vectorToNextCorner;     // 方向補正

                }


                // 次の角に向かって少しずつ目標を移動し, 目標値を保存
                ghost_AoiArm.Kinematics.Target[1].Position.Set = (armTargetPositionOld + vectorToNextCorner).Get;
                armTargetPositionOld.Set = ghost_AoiArm.Kinematics.Target[1].Position.Get;


                float distTmp = 1;

                Vector basePositionNow = new Vector(3);
                Vector basePositionOriginal = new Vector(3);
                Vector armTargetPositionOriginal = new Vector(3);

                while (0.05f < distTmp)     // 計算は手先目標に十分近づくまで行う
                {

                    ghost_AoiArm.Kinematics.NewInverseKinematics();
                    distTmp = (ghost_AoiArm.Kinematics.Target[1].Position - ghost_AoiArm.Kinematics.Chain[1].pe).Norm;
                    //Console.WriteLine(distTmp);


                    // 台車の位置・回転角を元に戻す
                    basePositionNow.Set = ghost_AoiArm.Kinematics.Chain[0].pe.Get;
                    basePositionOriginal.Set = (Rz_tv.Inv * basePositionNow + referencePosition).Get;

                    ghost_AoiArm[0, 0].q = basePositionOriginal[0];
                    ghost_AoiArm[0, 1].q = basePositionOriginal[1];
                    ghost_AoiArm[0, 5].q += angleToTargetVector;


                    // 台車の位置・回転角を記録
                    baseTars[k] = new Vector(3);
                    baseTars[k][0] = ghost_AoiArm[0, 0].q;
                    baseTars[k][1] = ghost_AoiArm[0, 1].q;
                    baseTars[k][2] = ghost_AoiArm[0, 5].q;


                    // 手先目標の位置をもとに戻す
                    armTargetPositionOriginal.Set = (Rz_tv.Inv * armTargetPositionOld + referencePosition).Get;

                    ballPosition[k + 1].Set = armTargetPositionOriginal.Get;


                    // 初期位置を更新して領域判断
                    ghost_AoiArm.Kinematics.ForwardKinematics();
                    initialBasePosition.Set = ghost_AoiArm.Kinematics.Chain[0].pe.Get;
                    initialBasePosition[2] = 0;
                    KinematicsSetup(ghost_AoiArm);


                    // 手先目標を再び補正
                    armTargetPositionOld.Set = (Rz_tv * (armTargetPositionOriginal - referencePosition)).Get;
                    ghost_AoiArm.Kinematics.Target[1].Position.Set = armTargetPositionOld.Get;

                }


                // 軌道用の関節目標を格納
                for (int i = 1; i < virtual_AoiArm.Kinematics.Chain.Length; i++)
                {

                    for (int j = 0; j < virtual_AoiArm.Kinematics.Chain[i].DOF; j++)
                    {
                        virtual_AoiArm[i, j].qTars[k] = ghost_AoiArm[i, j].q;
                    }

                }

            }

            ghost_AoiArm.Kinematics.SetTargetsToEffector();

        }

        private static void ContainVNorms(int[] timeSpanMs, int firstStep = 0)
        {

            for (int i = firstStep; i < StepNum_OneTable + firstStep; i++)
            {

                float dist_x, dist_y, dist;

                // 距離を計算
                if (i == 0)
                {
                    dist_x = baseTars[0][0] - initialBasePositionRead[0];
                    dist_y = baseTars[0][1] - initialBasePositionRead[1];
                }
                else
                {
                    dist_x = baseTars[i][0] - baseTars[i - 1][0];
                    dist_y = baseTars[i][1] - baseTars[i - 1][1];
                }

                dist = (float)Math.Sqrt(dist_x * dist_x + dist_y * dist_y);


                vNormsAmongBaseTars[i] = 1000 * dist / timeSpanMs[i];      // 速度の大きさを計算
                //Console.WriteLine(vNormsAmongBaseTars[i]);

                if (vNormsAmongBaseTars[i] > VMax)
                {
                    vNormsAmongBaseTars[i] = VMax;
                    Console.WriteLine("\"vNormsAmongBaseTars[" + i + "]\" exceeded VMax. Please set longer time value.");
                }

            }

            float totalTime = 0;
            for (int i = 0; i < timeSpanMs.Length; i++) totalTime += timeSpanMs[i];
            totalTime = totalTime / 1000f;

            Console.WriteLine("Total time of this task is " + totalTime + "s.");

        }

        public static void SetTableWipingtrajectory(Robot virtual_AoiArm, Robot ghost_AoiArm, Vector[] nodePosition_Arm, Vector[] nodePosition_Base,
            int[] timeSpanMs, float[] baseAngleTargets)
        {

            baseTars = new Vector[StepNumMax];
            vNormsAmongBaseTars = new Vector(StepNumMax);


            // 台車の位置を記録(xy平面上の点にしたいので, z座標は0に上書き)
            initialBasePositionRead.Set = ghost_AoiArm.Kinematics.Chain[0].pe.Get;
            initialBasePositionRead[2] = 0;
            initialBasePosition.Set = initialBasePositionRead.Get;


            // 引数を記録
            nodePositionRead_Arm = new Vector[nodePosition_Arm.Length];
            nodePositionRead_Base = new Vector[nodePosition_Base.Length];
            baseAngleTargetsRead = new float[baseAngleTargets.Length];
            baseAngleTargetsRead = baseAngleTargets;

            for (int i = 0; i < nodePosition_Arm.Length; i++)
            {

                nodePositionRead_Arm[i] = new Vector(3)
                {
                    Set = nodePosition_Arm[i].Get
                };

            }
            for (int i = 0; i < nodePosition_Base.Length; i++)
            {

                nodePositionRead_Base[i] = new Vector(3)
                {
                    Set = nodePosition_Base[i].Get
                };

            }


            MakeTableWipingTrajectory(virtual_AoiArm, ghost_AoiArm);
            ContainVNorms(timeSpanMs);

        }



        // 軌道の描画の有無を切り替える関数
        public static void SwitchTrajectoryDrawMode()
        {
            isTrajectoryDrawMode = !isTrajectoryDrawMode;
        }

        private static PrickleBall Ball = new PrickleBall();
        private static RotationMatrix Ry180 = new RotationMatrix(0, Deg2Rad(180), 0);
        private static RotationMatrix Rz_Base = new RotationMatrix(0, 0, Deg2Rad(30));
        // 軌道表示用のゴーストを軌道上に設置・表示する関数
        public static void DrawTrajectory(Robot virtual_AoiArm)
        {

            if (baseTars == null || !isTrajectoryDrawMode) return;  // 軌道を生成した覚えがないか, 表示モードでなければ以下スキップ


            //Ball.Rotate.Set = Ry180.Get;
            Ball.Rotate.Set = (Rz_Base * Ry180).Get;
            Ball.Diameter = 0.03f;


            for (int k = 0; k < TrajectoryGhostsNum; k++)
            {

                if (!TrajectoryGhost.IsWiperEnabled) TrajectoryGhost.SwitchWiperMode();     // ワイパーを有効化

                TrajectoryGhost.AoiArm[0, 0].q = BaseTars[k][0];     // x
                TrajectoryGhost.AoiArm[0, 1].q = BaseTars[k][1];     // y
                TrajectoryGhost.AoiArm[0, 5].q = BaseTars[k][2];     // theta_z

                for (int i = 1; i < TrajectoryGhost.AoiArm.Kinematics.Chain.Length; i++)
                {

                    for (int j = 0; j < TrajectoryGhost.AoiArm.Kinematics.Chain[i].DOF; j++)
                    {
                        TrajectoryGhost.AoiArm[i, j].q = virtual_AoiArm[i, j].qTars[k];     // ゴーストに軌道の関節変位を書き込む
                    }

                }

                TrajectoryGhost.AoiArm.Kinematics.ForwardKinematics();
                TrajectoryGhost.AoiArm.Kinematics.SetTargetsToEffector();
                TrajectoryGhost.AoiArm.Kinematics.Chain[1].EndPoint.Diameter = 0;

                TrajectoryGhost.Draw();


                //Ball.Position.Set = ballPosition[k].Get;
                //Ball.Draw();

            }

            //Ball.Position.Set = ballPosition[StepNumMax - 1].Get;
            //Ball.Draw();

        }

    }
}
