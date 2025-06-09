


// hayato
// 追加したものの主なリスト
// 関数：OldInverseKinematics(), NewInverseKinamatics(), AltInversekinematics(), ObserveOldIK(), ObserveNewIK(), ObserveAltIK(), SetVirtualJointSpringDiag()
// クラス：JointTarget, JointErrorVector, VirtualJointSpringMatrix
//
// いじったクラス
// Kinematics, Chain
//
// その他追加した変数等は「hayato」で検索



using System;

namespace OpenRCF
{
    public class Robot
    {
        public Kinematics Kinematics;
        public Trajectory Trajectory;
        public Cuboid Torso = new Cuboid(0, 0, 0);
        private int[] DOF;

        public Robot(params int[] n)
        {
            DOF = n;
            Kinematics = new Kinematics(n);
            Trajectory = new Trajectory(ref Kinematics);
        }

        public Pair this[int i, int j]
        {
            get
            {
                if (i < DOF.Length && j <= DOF[i])
                {
                    return Kinematics.Chain[i].Pair[j];
                }
                else
                {
                    Console.WriteLine("Error : i or j of Robot[i, j] is incorrect.");
                    return new Pair();
                }
            }
        }

        public void Draw()
        {
            Kinematics.Draw();
            Torso.Draw();
        }

        public void SetFloatingJoint6DOF(float sizeX, float sizeY, float sizeZ)
        {
            try
            {
                Kinematics[0, 0].SetPrismatic();
                Kinematics[0, 0].AxisInit.SetUnitVectorX();
                Kinematics[0, 0].LinkInit.SetZeroVector();

                Kinematics[0, 1].SetPrismatic();
                Kinematics[0, 1].AxisInit.SetUnitVectorY();
                Kinematics[0, 1].LinkInit.SetZeroVector();

                Kinematics[0, 2].SetPrismatic();
                Kinematics[0, 2].AxisInit.SetUnitVectorZ();
                Kinematics[0, 2].LinkInit.SetZeroVector();

                Kinematics[0, 3].AxisInit.SetUnitVectorX();
                Kinematics[0, 3].LinkInit.SetZeroVector();
                Kinematics[0, 3].SetJointRangeInfinite();

                Kinematics[0, 4].AxisInit.SetUnitVectorY();
                Kinematics[0, 4].LinkInit.SetZeroVector();
                Kinematics[0, 4].SetJointRangeInfinite();

                Kinematics[0, 5].AxisInit.SetUnitVectorZ();
                Kinematics[0, 5].LinkInit.SetZeroVector();
                Kinematics[0, 5].SetJointRangeInfinite();

                Kinematics[0, 6].LinkInit.SetZeroVector();

                Torso.Position = Kinematics.Chain[0].pe;
                Torso.Rotate = Kinematics.Chain[0].Re;
                Torso.SetSize(sizeX, sizeY, sizeZ);
            }
            catch
            {
                Console.WriteLine("Error : Degrees of Freedom is Not enough in SetFloatingJoint().");
            }
        }

        public void SetPlanarJoint3DOF(float sizeX, float sizeY, float sizeZ)
        {
            try
            {
                Kinematics[0, 0].SetPrismatic();
                Kinematics[0, 0].AxisInit.SetUnitVectorX();
                Kinematics[0, 0].LinkInit.SetZeroVector();

                Kinematics[0, 1].SetPrismatic();
                Kinematics[0, 1].AxisInit.SetUnitVectorY();
                Kinematics[0, 1].LinkInit.SetZeroVector();

                Kinematics[0, 2].AxisInit.SetUnitVectorZ();
                Kinematics[0, 2].LinkInit.SetZeroVector();
                Kinematics[0, 2].SetJointRangeInfinite();

                Kinematics[0, 3].LinkInit[2] = sizeZ;
                Torso.Position = Kinematics[0, 3].pc;
                Torso.Rotate = Kinematics.Chain[0].Re;
                Torso.SetSize(sizeX, sizeY, sizeZ);
            }
            catch
            {
                Console.WriteLine("Error : Degrees of Freedom is Not enough in SetPlanarJoint().");
            }
        }

        public bool IsCollision(params ICollision[] obstacle)
        {
            if (Kinematics.IsCollisionBody(obstacle)) return true;
            if (Kinematics.IsCollisionEffc(obstacle)) return true;
            if (Kinematics.IsCollisionSelf()) return true;
            return false;
        }

        public byte Transparency
        {
            get { return Torso.Color.Alpha; }
            set
            {
                for (int i = 0; i < DOF.Length; i++)
                {
                    for (int j = 0; j <= DOF[i]; j++)
                    {
                        Kinematics.Chain[i].Pair[j].JointObj.Color.Alpha = value;
                        Kinematics.Chain[i].Pair[j].LinkObj.Color.Alpha = value;
                    }

                    Kinematics.Target[i].Color.Alpha = value;
                    Kinematics.Chain[i].EndPoint.Color.Alpha = value;
                }

                Torso.Color.Alpha = value;
            }
        }

        public void CopyJointAngles(Robot robot)
        {
            for (int i = 0; i < DOF.Length; i++)
            {
                for (int j = 0; j <= DOF[i]; j++)
                {
                    this[i, j].q = robot[i, j].q;
                }
            }
        }

        public void CopyJointTrajectory(Robot robot)
        {
            for (int i = 0; i < DOF.Length; i++)
            {
                for (int j = 0; j <= DOF[i]; j++)
                {
                    for (int k = 0; k < this[i, j].qTars.Length; k++)
                    {
                        this[i, j].qTars[k] = robot[i, j].qTars[k];
                    }
                }
            }
        }

        public void CopyJointLinkStructure(Robot robot)
        {
            for (int i = 0; i < DOF.Length; i++)
            {
                for (int j = 0; j <= DOF[i]; j++)
                {
                    for (int k = 0; k < this[i, j].qTars.Length; k++)
                    {
                        this[i, j].LinkInit.Set = robot[i, j].LinkInit.Get;
                        this[i, j].AxisInit.Set = robot[i, j].AxisInit.Get;
                        this[i, j].JointRange[0] = robot[i, j].JointRange[0];
                        this[i, j].JointRange[1] = robot[i, j].JointRange[1];
                    }
                }
            }
        }

    }

    public class Trajectory
    {
        private Kinematics kinematics;

        public Trajectory(ref Kinematics kinematics)
        {
            this.kinematics = kinematics;
        }

        private int qTarNum, qTarNumMax;
        private int step, stepNum, stepNumSum;
        private float[] ratio;

        private float[] RatioAmongJointTargets(int targetNum)
        {
            float[] result = new float[targetNum + 1];
            float sum = 0;

            for (int k = 0; k < targetNum; k++)
            {
                for (int i = 0; i < kinematics.Chain.Length; i++)
                {
                    for (int j = 0; j < kinematics.Chain[i].DOF; j++)
                    {
                        if (k == 0) result[k] += Math.Abs(kinematics[i, j].qTars[k] - kinematics[i, j].q);
                        else result[k] += Math.Abs(kinematics[i, j].qTars[k] - kinematics[i, j].qTars[k - 1]);
                    }
                }

                sum += result[k];
            }

            if (0 < sum)
            {
                for (int k = 0; k < targetNum; k++)
                {
                    result[k] = result[k] / sum;
                }
            }

            return result;
        }

        public void MoveToJointTargets(int timeMs, int targetNum)
        {
            ratio = RatioAmongJointTargets(targetNum);
            qTarNumMax = targetNum;
            qTarNum = 0;
            step = 1;
            stepNumSum = timeMs / 25;
            System.Threading.Tasks.Task.Run(ProceedToTargetsLoop);
        }

        private float[] RatioAmongJointTargets(int[] timeMs, float timeMsSum)
        {
            float[] result = new float[timeMs.Length];
            for (int k = 0; k < timeMs.Length; k++) { result[k] = timeMs[k] / timeMsSum; }
            return result;
        }

        public void MoveToJointTargets(int[] timeMs)
        {
            float timeMsSum = 0;
            for (int k = 0; k < timeMs.Length; k++) { timeMsSum += timeMs[k]; }
            ratio = RatioAmongJointTargets(timeMs, timeMsSum);
            qTarNumMax = timeMs.Length + 1;
            qTarNum = 0;
            step = 1;
            stepNumSum = (int)(timeMsSum / 25);
            System.Threading.Tasks.Task.Run(ProceedToTargetsLoop);
        }

        private void ProceedToTargets()
        {
            stepNum = (int)(ratio[qTarNum] * stepNumSum);

            if (0 < stepNum)
            {
                float t = (float)step / stepNum;
                float val0to1;

                if (qTarNum == 0 && t < 0.5f) val0to1 = -2 * t * t * t + 3 * t * t;
                else if (qTarNum == qTarNumMax - 1 && 0.5f < t) val0to1 = -2 * t * t * t + 3 * t * t;
                else val0to1 = t;

                for (int i = 0; i < kinematics.Chain.Length; i++)
                {
                    for (int j = 0; j < kinematics.Chain[i].DOF; j++)
                    {
                        kinematics.Chain[i].Pair[j].ProceedToTargets(qTarNum, val0to1, step == 1);
                    }
                }
            }

            kinematics.ForwardKinematics();

            if (step < stepNum)
            {
                step++;
            }
            else if (qTarNum < qTarNumMax)
            {
                qTarNum++;
                step = 1;
            }

        }

        public bool IsMoveCompleted { get; private set; } = true;

        private void ProceedToTargetsLoop()
        {
            IsMoveCompleted = false;
            int k = 0;

            while (k < stepNumSum)
            {
                ProceedToTargets();

                if (ratio.Length - 1 < qTarNum) break;  // hayato

                System.Threading.Thread.Sleep(25);
                k++;
            }

            IsMoveCompleted = true;
        }

        public void SetCurrentJointToTarget(int qTarsNum)
        {
            for (int i = 0; i < kinematics.Chain.Length; i++)
            {
                for (int j = 0; j < kinematics.Chain[i].DOF; j++)
                {
                    kinematics[i, j].qTars[qTarsNum] = kinematics[i, j].q;
                }
            }
        }

        public bool IsCollisionAmongJointTargets(int qTarNum1, int qTarNum2, params ICollision[] iCollision)
        {
            kinematics.RecordCurrentJointAngles();

            for (int k = qTarNum1; k < qTarNum2; k++)
            {
                float d = 0;
                int n = 0;

                for (int i = 0; i < kinematics.Chain.Length; i++)
                {
                    for (int j = 0; j < kinematics.Chain[i].DOF; j++)
                    {
                        d += Math.Abs(kinematics[i, j].qTars[k] - kinematics[i, j].qTars[k + 1]);
                        n++;
                    }
                }

                int divNum = (int)(20 * d / (n + 1));
                if (divNum == 0) divNum = 1;

                for (int div = 0; div <= divNum; div++)
                {
                    for (int i = 0; i < kinematics.Chain.Length; i++)
                    {
                        for (int j = 0; j < kinematics.Chain[i].DOF; j++)
                        {
                            kinematics.Chain[i].Pair[j].ProceedToTargets(k + 1, (float)div / divNum, div == 0);
                        }
                    }

                    kinematics.ForwardKinematics();

                    if (kinematics.IsCollisionEffc(iCollision) || kinematics.IsCollisionBody(iCollision))
                    {
                        kinematics.ReadRecordJointAngles();
                        return true;
                    }
                }

            }

            kinematics.ReadRecordJointAngles();
            return false;
        }

    }

    public class Kinematics
    {
        public Chain[] Chain;
        public Target[] Target;
        public JointTarget[] JointTarget;   // hayato

        public ErrorVector e;
        public JointErrorVector e_J;    // hayato
        public VirtualSpringMatrix K;
        public VirtualJointSpringMatrix K_J;    //hayato
        public JacobianMatrix J;
        public BlockMatrix JT;
        public BlockVector tau;

        private float Vk, VkOld, delta;
        private BlockVector tauOld, tauTilde, dq;
        private BlockMatrix JTKJ, D, T, E;
        private int[] n;
        private int nSum = 0;

        public Vector BasePosition = new Vector(3);
        public RotationMatrix BaseRotate = new RotationMatrix();

        private static float g = 9.80665f;
        private BlockVector tauSelf, mg;
        private BlockMatrix Y;

        private static int[] Int6(int n)
        {
            int[] result = new int[n];
            for (int i = 0; i < n; i++) result[i] = 6;
            return result;
        }

        private static int[] IntChainDOF(Chain[] chain)     // hayato：直鎖ごとの自由度を配列で取得する関数
        {
            int[] result = new int[chain.Length];
            for (int i = 0; i < chain.Length; i++) result[i] = chain[i].DOF;
            return result;
        }

        public Kinematics(int[] n)
        {
            this.n = n;
            Chain = new Chain[n.Length];
            Target = new Target[n.Length];
            JointTarget = new JointTarget[n.Length];    // hayato

            for (int i = 0; i < n.Length; i++)
            {
                Chain[i] = new Chain(n[i]);
                Target[i] = new Target(0.05f);
                Target[i].Position.Set = new float[3] { 1, 0, 1 };
                JointTarget[i] = new JointTarget(Chain[i].DOF);     // hayato
                nSum += n[i];
            }

            e = new ErrorVector(Int6(n.Length));
            e_J = new JointErrorVector(IntChainDOF(Chain));     // hayato
            K = new VirtualSpringMatrix(Int6(n.Length), Int6(n.Length));
            K_J = new VirtualJointSpringMatrix(IntChainDOF(Chain), IntChainDOF(Chain));     // hayato
            K.FixBlockDiagonal();
            tau = new BlockVector(n);
            tauOld = new BlockVector(n);
            tauTilde = new BlockVector(n);
            dq = new BlockVector(n);
            J = new JacobianMatrix(Int6(n.Length), n);
            J.FixBlockLowerTriangle();
            JTKJ = new BlockMatrix(n, n);
            D = new BlockMatrix(n, n);
            T = new BlockMatrix(n, n);
            T.FixBlockDiagonal();
            E = new BlockMatrix(n, n);
            E.FixBlockDiagonal();

            tauSelf = new BlockVector(n);
            Y = new BlockMatrix(n, n);
            Y.FixBlockUpperTriangle();
            mg = new BlockVector(n);
        }

        public Pair this[int i, int j] { get { return Chain[i].Pair[j]; } }

        public void Draw()
        {
            for (int i = 0; i < Chain.Length; i++)
            {
                Chain[i].Draw();
                Target[i].Draw();
            }
        }

        public void SetJointLinkRadiusAuto(float ratio = 0.2f)
        {
            float sum = 0;
            int n = 0;

            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF + 1; j++)
                {
                    if (0 < Chain[i].Pair[j].LinkInit.AbsSum)
                    {
                        sum += Chain[i].Pair[j].LinkInit.Norm;
                        n++;
                    }
                }
            }

            float jointRadius;

            if (0 < n) jointRadius = ratio * sum / n;
            else jointRadius = 0;

            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF + 1; j++)
                {
                    Chain[i].Pair[j].JointObj.Radius = jointRadius;
                    Chain[i].Pair[j].LinkObj.Radius = jointRadius;
                }

                Target[i].Diameter = jointRadius;
            }
        }

        public void SetTargetsToEffector()
        {
            for (int i = 0; i < Chain.Length; i++)
            {
                Target[i].Position.Set = Chain[i].pe.Get;
                Target[i].Rotate.Set = Chain[i].Re.Get;
            }
        }

        public void ReturnHomePosition()
        {
            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF; j++)
                {
                    Chain[i].Pair[j].q = Chain[i].Pair[j].qHome;
                }
            }

            ForwardKinematics();
        }

        public int RedundantDOF
        {
            get
            {
                int sum = 0;
                for (int i = 0; i < Target.Length; i++) { sum += Target[i].DOF; }
                if (sum < nSum) return nSum - sum;
                else return 0;
            }
        }

        private RotationMatrix Rn = new RotationMatrix();
        private RotationMatrix originalRe = new RotationMatrix();   // hayato
        public void ForwardKinematics()
        {
            for (int i = 0; i < Chain.Length; i++)
            {
                if (i == 0) Chain[i].Pair[0].R.Set = BaseRotate.Get;
                else Chain[i].Pair[0].R.Set = Chain[0].PairEnd.R.Get;

                for (int j = 0; j < Chain[i].DOF; j++)
                {
                    if (Chain[i].Pair[j].IsRevolute || Chain[i].Pair[j].IsParallelA)
                    {
                        Rn.SetRn(Chain[i].Pair[j].q, Chain[i].Pair[j].AxisInit);
                        Chain[i].Pair[j + 1].R.Set = Chain[i].Pair[j].R.Times(Rn);
                    }
                    else if (Chain[i].Pair[j].IsPrismatic)
                    {
                        Chain[i].Pair[j + 1].R.Set = Chain[i].Pair[j].R.Get;
                    }
                    else if (Chain[i].Pair[j].IsParallelB)
                    {
                        Chain[i].Pair[j + 1].R.Set = Chain[i].Pair[j - 1].R.Get;
                    }
                    else
                    {
                        Console.WriteLine("Error : JointType is Not defined.");
                    }
                }
            }

            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF + 1; j++)
                {
                    Chain[i].Pair[j].Axis.Set = Chain[i].Pair[j].R.Times(Chain[i].Pair[j].AxisInit);
                    Chain[i].Pair[j].Link.Set = Chain[i].Pair[j].R.Times(Chain[i].Pair[j].LinkInit);
                }
            }

            for (int i = 0; i < Chain.Length; i++)
            {
                if (i == 0) Chain[i].Pair[0].p.Set = Chain[i].Pair[0].Link.Plus(BasePosition);
                else Chain[i].Pair[0].p.Set = Chain[i].Pair[0].Link.Plus(Chain[0].PairEnd.p);

                for (int j = 0; j < Chain[i].DOF; j++)
                {
                    Chain[i].Pair[j + 1].p.Set = Chain[i].Pair[j].p.Plus(Chain[i].Pair[j + 1].Link);

                    if (Chain[i].Pair[j].IsPrismatic)
                    {
                        Chain[i].Pair[j + 1].p.Set = Chain[i].Pair[j + 1].p.Plus(Chain[i].Pair[j].q * Chain[i].Pair[j].Axis);
                    }
                }
            }

            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF + 1; j++)
                {
                    Chain[i].Pair[j].pt.Set = Chain[i].Pair[j].p.Plus(-0.25f * Chain[i].Pair[j].Link);
                    Chain[i].Pair[j].pc.Set = Chain[i].Pair[j].p.Plus(-0.50f * Chain[i].Pair[j].Link);
                    Chain[i].Pair[j].pb.Set = Chain[i].Pair[j].p.Plus(-0.75f * Chain[i].Pair[j].Link);
                }


                // hayato：original
                //Chain[i].pe.Set = Chain[i].PairEnd.p.Get;
                //Chain[i].Re.Set = Chain[i].PairEnd.R.Times(Chain[i].PairEnd.LinkInit.ConvertRotationMatrix);


                // hayato：オフセットあり
                originalRe = Chain[i].PairEnd.R.Times(Chain[i].PairEnd.LinkInit.ConvertRotationMatrix);

                Chain[i].pe.Set = (Chain[i].PairEnd.p + originalRe * Chain[i].peOffset).Get;
                Chain[i].Re.Set = (originalRe * Chain[i].ReOffset).Get;
            }
        }

        private float e2Tmp = -1, epsilon = 0.000001f;
        private float e_J2Tmp = -1;     // hayato
        public void InverseKinematics()
        {
            int k = 0;
            bool IsVkStop = false;

            ForwardKinematics();
            e.Update(Chain, Target);

            if (Math.Abs(e2Tmp - e.SquareSum) < epsilon) return;
            else
            {
                K.ZetaReset(Chain, Target);
                tauTilde.SetZeroVector();
                tau.SetZeroVector();
                Vk = e.Trans * K * e;
            }

            while (k < 100)
            {
                ForwardKinematics();
                e.Update(Chain, Target);

                VkOld = Vk;
                Vk = 0.5f * e.Trans * K * e;

                if (VkOld <= Vk || Math.Abs(VkOld - Vk) < epsilon)
                {
                    if (K.IsZetaConverged(Target))
                    {
                        if (IsVkStop) break;
                        else IsVkStop = true;
                    }
                    else K.ZetaUpdate(Chain, Target);
                }

                J.Update(Chain);
                tauOld.Set = tau.Get;
                tau = J.Trans * K * e;
                J.ReflectJointMobility(Chain, tau);
                JTKJ = J.Trans * K * J;
                delta = (RedundantDOF + 1) * JTKJ.Trace / (100f * nSum);
                E.SetDiagonal(delta + 0.2f * Vk);
                T.SetDiagonal(tauTilde + tau.Abs);
                CalcTauTilde();
                D = JTKJ + T + E;
                dq = D.Inv * tau;
                q = dq + q;

                k++;
            }

            e2Tmp = e.SquareSum;
        }


        // hayato：ここから



        private int kOld;   // 収束時のステップ数kを記録する変数
        private int stepTag;    // stepsContainerTmpの要素を呼ぶための値
        private int[] stepsContainerTmp;    // ステップ数kを格納するための仮の場所. 計算の都度書き換えられる
        public int[] StepsContainerTmp { get { return stepsContainerTmp; } }     // クラス外読み取り用

        // 以下のOldInverseKinematics(), NewInverseKinematics()における引数は実験のために追加した
        public void OldInverseKinematics(bool isStepsSaveMode = false)  // 2018の関口さんの論文の解法を用いた, 逆運動学を計算する関数
        {
            if (isStepsSaveMode)    // 実験用
            {
                kOld = 0;
                stepTag = 0;
                stepsContainerTmp = new int[5];
            }

            int k = 0;
            bool IsVkStop = false;

            ForwardKinematics();
            e.Update(Chain, Target);

            if (Math.Abs(e2Tmp - e.SquareSum) < epsilon) return;
            else
            {
                K.ZetaReset(Chain, Target);
                //tauTilde.SetZeroVector();     // 削除
                tau.SetZeroVector();
                Vk = e.Trans * K * e;
            }

            while (k < 100)
            {
                ForwardKinematics();
                e.Update(Chain, Target);

                VkOld = Vk;
                Vk = 0.5f * e.Trans * K * e;

                if (VkOld <= Vk || Math.Abs(VkOld - Vk) < epsilon)
                {
                    if (K.IsZetaConverged(Target))
                    {
                        if (IsVkStop)
                        {
                            if (isStepsSaveMode) stepsContainerTmp[4] = k;     // 実験用
                            break;
                        }
                        else IsVkStop = true;
                    }
                    else
                    {
                        K.ZetaUpdate(Chain, Target);

                        if (isStepsSaveMode)    // 実験用
                        {
                            if (stepTag == 0) stepsContainerTmp[stepTag] = k + 1;
                            else stepsContainerTmp[stepTag] = k - kOld;
                            kOld = k;
                            stepTag++;
                        }
                    }
                }

                J.Update(Chain);
                //tauOld.Set = tau.Get;     // 削除
                tau = J.Trans * K * e;
                J.ReflectJointMobility(Chain, tau);
                JTKJ = J.Trans * K * J;
                //delta = (RedundantDOF + 1) * K_J.lengthMax * K_J.lengthMax / 1000f;     // 論文にて示されている値だが, 大体Dの逆行列が計算できない
                delta = JTKJ.Trace / (1000f * nSum);    // 2023の関口さんの論文にて示されている値. こっちならうまくいく
                E.SetDiagonal(0.5f * Vk + delta);   // 変更
                //T.SetDiagonal(tauTilde + tau.Abs);    // 削除
                //CalcTauTilde();   // 削除
                D = JTKJ + E;   // 変更
                dq = D.Inv * tau;
                q = dq + q;

                k++;
            }

            e2Tmp = e.SquareSum;
        }

        public void NewInverseKinematics(bool isStepsSaveMode = false)  // 仮想関節バネを用いて逆運動学を計算する関数
        {
            if (isStepsSaveMode)    // 実験用
            {
                kOld = 0;
                stepTag = 0;
                stepsContainerTmp = new int[5];
            }

            int k = 0;
            bool IsVkStop = false;

            ForwardKinematics();
            e.Update(Chain, Target);
            e_J.Update(Chain, JointTarget);     // 追加

            if (Math.Abs(e2Tmp - e.SquareSum) < epsilon && (Math.Abs(e_J2Tmp - e_J.SquareSum) < epsilon)) return;   // 一部追加
            else
            {
                K.ZetaReset(Chain, Target);
                K_J.Zeta_JReset(Chain);     // 追加
                //tauTilde.SetZeroVector();     // 削除
                tau.SetZeroVector();
                Vk = e.Trans * K * e + e_J.Trans * K_J * e_J;   // 一部追加
            }

            while (k < 100)
            {
                ForwardKinematics();
                e.Update(Chain, Target);
                e_J.Update(Chain, JointTarget);     // 追加

                VkOld = Vk;
                Vk = 0.5f * e.Trans * K * e + 0.5f * e_J.Trans * K_J * e_J;     // 一部追加

                if (VkOld <= Vk || Math.Abs(VkOld - Vk) < epsilon)
                {
                    // 以下大きく変更
                    if (K.IsZetaConverged(Target) && K_J.IsZeta_JConverged(JointTarget))
                    {
                        if (IsVkStop)
                        {
                            if (isStepsSaveMode) stepsContainerTmp[4] = k;     // 実験用
                            break;
                        }
                        else IsVkStop = true;
                    }
                    else if (!K.IsZetaConverged(Target) && !K_J.IsZeta_JConverged(JointTarget))
                    {
                        K.ZetaUpdate(Chain, Target);
                        K_J.Zeta_JUpdate(Chain, JointTarget);

                        if (isStepsSaveMode)    // 実験用
                        {
                            if (stepTag == 0) stepsContainerTmp[stepTag] = k + 1;
                            else stepsContainerTmp[stepTag] = k - kOld;
                            kOld = k;
                            stepTag++;
                        }
                    }
                    else if (!K_J.IsZeta_JConverged(JointTarget))
                    {
                        K_J.Zeta_JUpdate(Chain, JointTarget);

                        if (isStepsSaveMode)    // 実験用
                        {
                            if (stepTag == 0) stepsContainerTmp[stepTag] = k + 1;
                            else stepsContainerTmp[stepTag] = k - kOld;
                            kOld = k;
                            stepTag++;
                        }
                    }
                    else K.ZetaUpdate(Chain, Target);
                }

                J.Update(Chain);
                //tauOld.Set = tau.Get;     // 削除
                tau = J.Trans * K * e + K_J * e_J;      // 一部追加
                J.ReflectJointMobility(Chain, tau);
                JTKJ = J.Trans * K * J;
                //delta = (RedundantDOF + 1) * K_J.lengthMax * K_J.lengthMax / 1000f;     // 2018の関口さんの論文より抜粋. 大体うまくいかない
                delta = JTKJ.Trace / (1000f * nSum);    // 2023の関口さんの論文にて示されている値. うまくいく
                E.SetDiagonal(0.5f * Vk + delta);   // 変更
                //T.SetDiagonal(tauTilde + tau.Abs);    // 削除
                //CalcTauTilde();   // 削除
                D = JTKJ + K_J + E;     // 変更, 提案手法B(西村さんの修論より抜粋)
                dq = D.Inv * tau;
                q = dq + q;

                k++;
            }

            e2Tmp = e.SquareSum;
            e_J2Tmp = e_J.SquareSum;    // 追加
        }

        public void AltInverseKinematics()  // 仮想関節バネを用いて逆運動学を計算する関数. 優先度を「手先＞関節＞台車」とするバージョン
        {
            int k = 0;
            bool IsVkStop = false;

            ForwardKinematics();
            e.Update(Chain, Target);
            e_J.Update(Chain, JointTarget);     // 追加

            if (Math.Abs(e2Tmp - e.SquareSum) < epsilon && (Math.Abs(e_J2Tmp - e_J.SquareSum) < epsilon)) return;     // 一部追加
            else
            {
                K.ZetaReset(Chain, Target);
                K_J.Zeta_JReset(Chain);     // 追加
                //tauTilde.SetZeroVector();     // 削除
                tau.SetZeroVector();
                Vk = e.Trans * K * e + e_J.Trans * K_J * e_J;   // 一部追加
            }

            while (k < 100)
            {
                ForwardKinematics();
                e.Update(Chain, Target);
                e_J.Update(Chain, JointTarget);     // 追加

                VkOld = Vk;
                Vk = 0.5f * e.Trans * K * e + 0.5f * e_J.Trans * K_J * e_J;     // 一部追加

                if (VkOld <= Vk || Math.Abs(VkOld - Vk) < epsilon)
                {
                    // 以下大きく変更
                    if (K.IsZetaConverged(Target) && K_J.IsZeta_JConverged(JointTarget))
                    {
                        if (IsVkStop) break;
                        else IsVkStop = true;
                    }
                    else if (!K.IsZetaConverged(Target)) K.ZetaUpdate(Chain, Target);
                    else if (!K_J.IsZeta_JConverged(JointTarget)) K_J.Zeta_JUpdate(Chain, JointTarget);
                }

                J.Update(Chain);
                //tauOld.Set = tau.Get;     // 削除
                tau = J.Trans * K * e + K_J * e_J;      // 一部追加
                J.ReflectJointMobility(Chain, tau);
                JTKJ = J.Trans * K * J;
                //delta = (RedundantDOF + 1) * K_J.lengthMax * K_J.lengthMax / 1000f;     // 2018の関口さんの論文より抜粋. 大体うまくいかない
                delta = JTKJ.Trace / (1000f * nSum);    // 2023の関口さんの論文にて示されている値. うまくいく
                E.SetDiagonal(0.5f * Vk + delta);   // 変更
                //T.SetDiagonal(tauTilde + tau.Abs);    // 削除
                //CalcTauTilde();   // 削除
                D = JTKJ + K_J + E;     // 変更, 提案手法B(西村さんの修論より抜粋)
                dq = D.Inv * tau;
                q = dq + q;

                k++;
            }

            e2Tmp = e.SquareSum;
            e_J2Tmp = e_J.SquareSum;    // 追加
        }


        private int k = 0;
        private bool IsVkStop = false;
        private bool IsCalcStop = false;    // Paralllelクラスの関数実行を支配する変数

        private void OldIKWhile()   // OldInverseKinematics()内のwhile文の中身. ObserveOldIK()用にコンソール出力などを設定
        {
            ForwardKinematics();
            e.Update(Chain, Target);

            VkOld = Vk;
            Vk = 0.5f * e.Trans * K * e;

            if (VkOld <= Vk || Math.Abs(VkOld - Vk) < epsilon)
            {
                if (K.IsZetaConverged(Target))
                {
                    if (IsVkStop)
                    {
                        Console.WriteLine("\nOne of the answers of inverse kinematics has been calculated.");
                        Console.WriteLine("The total number of steps is " + k + ".");
                        Console.WriteLine("Finally Vk = " + VkOld + ".");
                        IsCalcStop = true;
                        return;
                    }
                    else IsVkStop = true;
                }
                else
                {
                    K.ZetaUpdate(Chain, Target);
                    Console.WriteLine("Zeta has been updated at step " + k + ".");
                }
            }

            J.Update(Chain);
            tau = J.Trans * K * e;
            J.ReflectJointMobility(Chain, tau);
            JTKJ = J.Trans * K * J;
            delta = JTKJ.Trace / (1000f * nSum);
            E.SetDiagonal(0.5f * Vk + delta);
            D = JTKJ + E;
            dq = D.Inv * tau;
            q = dq + q;

            ForwardKinematics();
            Console.WriteLine("Vk = " + Vk + " at step " + k + ".");

            k++;
        }

        private void CompleteOldIK()    // ObserveOldIK()完了時に実行する関数
        {
            IsCalcStop = false;
            e2Tmp = e.SquareSum;

            Console.WriteLine("\nCompleted!\n");
        }

        public void ObserveOldIK(uint timeSpanMs)  // OldInverseKinematics()で逆運動学解を求める様子を観察するための関数. 様々な情報をコンソールに出力する
        {
            k = 0;
            IsVkStop = false;

            ForwardKinematics();
            e.Update(Chain, Target);

            if (Math.Abs(e2Tmp - e.SquareSum) < epsilon) return;
            else
            {
                K.ZetaReset(Chain, Target);
                tau.SetZeroVector();
                Vk = e.Trans * K * e;
            }

            Parallel.Run(OldIKWhile, () => IsCalcStop, timeSpanMs);
            Parallel.RunWait(CompleteOldIK, IsCalcStop, (int)timeSpanMs);
        }

        private void NewIKWhile()   // NewInverseKinematics()内のwhile文の中身. ObserveNewIK()用にコンソール出力などを設定
        {
            ForwardKinematics();
            e.Update(Chain, Target);
            e_J.Update(Chain, JointTarget);

            VkOld = Vk;
            Vk = 0.5f * e.Trans * K * e + 0.5f * e_J.Trans * K_J * e_J;

            if (VkOld <= Vk || Math.Abs(VkOld - Vk) < epsilon)
            {
                if (K.IsZetaConverged(Target) && K_J.IsZeta_JConverged(JointTarget))
                {
                    if (IsVkStop)
                    {
                        Console.WriteLine("\nOne of the answers of inverse kinematics has been calculated.");
                        Console.WriteLine("The total number of steps is " + k + ".");
                        Console.WriteLine("Finally Vk = " + VkOld + ".");
                        IsCalcStop = true;
                        return;
                    }
                    else IsVkStop = true;
                }
                else if (!K.IsZetaConverged(Target) && !K_J.IsZeta_JConverged(JointTarget))
                {
                    K.ZetaUpdate(Chain, Target);
                    K_J.Zeta_JUpdate(Chain, JointTarget);
                    Console.WriteLine("Zeta and Zeta_J have been updated at step " + k + ".");
                }
                else if (!K_J.IsZeta_JConverged(JointTarget))
                {
                    K_J.Zeta_JUpdate(Chain, JointTarget);
                    Console.WriteLine("Zeta_J has been updated at step " + k + ".");
                }
                else
                {
                    K.ZetaUpdate(Chain, Target);
                    Console.WriteLine("Zeta has been updated at step " + k + ".");
                }
            }

            J.Update(Chain);
            tau = J.Trans * K * e + K_J * e_J;
            J.ReflectJointMobility(Chain, tau);
            JTKJ = J.Trans * K * J;
            delta = JTKJ.Trace / (1000f * nSum);
            E.SetDiagonal(0.5f * Vk + delta);
            D = JTKJ + K_J + E;
            dq = D.Inv * tau;
            q = dq + q;

            ForwardKinematics();
            Console.WriteLine("Vk = " + Vk + " at step " + k + ".");

            k++;
        }

        private void CompleteNewIK()    // ObserveNewIK()完了時に実行する関数
        {
            IsCalcStop = false;
            e2Tmp = e.SquareSum;
            e_J2Tmp = e_J.SquareSum;

            Console.WriteLine("\nCompleted!\n");
        }

        public void ObserveNewIK(uint timeSpanMs)  // NewInverseKinematics()で逆運動学解を求める様子を観察するための関数. 様々な情報をコンソールに出力する
        {
            k = 0;
            IsVkStop = false;

            ForwardKinematics();
            e.Update(Chain, Target);
            e_J.Update(Chain, JointTarget);

            if (Math.Abs(e2Tmp - e.SquareSum) < epsilon && Math.Abs(e_J2Tmp - e_J.SquareSum) < epsilon) return;
            else
            {
                K.ZetaReset(Chain, Target);
                K_J.Zeta_JReset(Chain);
                tau.SetZeroVector();
                Vk = e.Trans * K * e + e_J.Trans * K_J * e_J;
            }

            Parallel.Run(NewIKWhile, () => IsCalcStop, timeSpanMs);
            Parallel.RunWait(CompleteNewIK, IsCalcStop, (int)timeSpanMs);
        }

        private void AltIKWhile()   // AltInverseKinematics()内のwhile文の中身. ObserveNewIK()用にコンソール出力などを設定
        {
            ForwardKinematics();
            e.Update(Chain, Target);
            e_J.Update(Chain, JointTarget);

            VkOld = Vk;
            Vk = 0.5f * e.Trans * K * e + 0.5f * e_J.Trans * K_J * e_J;

            if (VkOld <= Vk || Math.Abs(VkOld - Vk) < epsilon)
            {
                if (K.IsZetaConverged(Target) && K_J.IsZeta_JConverged(JointTarget))
                {
                    if (IsVkStop)
                    {
                        Console.WriteLine("\nOne of the answers of inverse kinematics has been calculated.");
                        Console.WriteLine("The total number of steps is " + k + ".");
                        Console.WriteLine("Finally Vk = " + VkOld + ".");
                        IsCalcStop = true;
                        return;
                    }
                    else IsVkStop = true;
                }
                else if (!K.IsZetaConverged(Target))
                {
                    K.ZetaUpdate(Chain, Target);
                    Console.WriteLine("Zeta has been updated at step " + k + ".");
                }
                else if (!K_J.IsZeta_JConverged(JointTarget))
                {
                    K_J.Zeta_JUpdate(Chain, JointTarget);
                    Console.WriteLine("Zeta_J has been updated at step " + k + ".");
                }
            }

            J.Update(Chain);
            tau = J.Trans * K * e + K_J * e_J;
            J.ReflectJointMobility(Chain, tau);
            JTKJ = J.Trans * K * J;
            delta = JTKJ.Trace / (1000f * nSum);
            E.SetDiagonal(0.5f * Vk + delta);
            D = JTKJ + K_J + E;
            dq = D.Inv * tau;
            q = dq + q;

            ForwardKinematics();
            Console.WriteLine("Vk = " + Vk + " at step " + k + ".");

            k++;
        }

        private void CompleteAltIK()    // ObserveAltIK()完了時に実行する関数
        {
            IsCalcStop = false;
            e2Tmp = e.SquareSum;
            e_J2Tmp = e_J.SquareSum;

            Console.WriteLine("\nCompleted!\n");
        }

        public void ObserveAltIK(uint timeSpanMs)  // AltInverseKinematics()で逆運動学解を求める様子を観察するための関数. 様々な情報をコンソールに出力する
        {
            k = 0;
            IsVkStop = false;

            ForwardKinematics();
            e.Update(Chain, Target);
            e_J.Update(Chain, JointTarget);

            if (Math.Abs(e2Tmp - e.SquareSum) < epsilon && Math.Abs(e_J2Tmp - e_J.SquareSum) < epsilon) return;
            else
            {
                K.ZetaReset(Chain, Target);
                K_J.Zeta_JReset(Chain);
                tau.SetZeroVector();
                Vk = e.Trans * K * e + e_J.Trans * K_J * e_J;
            }

            Parallel.Run(AltIKWhile, () => IsCalcStop, timeSpanMs);
            Parallel.RunWait(CompleteAltIK, IsCalcStop, (int)timeSpanMs);
        }



        // hayato：ここまで


        private BlockVector q
        {
            get
            {
                BlockVector result = new BlockVector(n);

                for (int i = 0; i < n.Length; i++)
                {
                    for (int j = 0; j < n[i]; j++)
                    {
                        result.Ref[i].Ref[j] = Chain[i].Pair[j].q;
                    }
                }

                return result;
            }
            set
            {
                for (int i = 0; i < n.Length; i++)
                {
                    for (int j = 0; j < n[i]; j++)
                    {
                        Chain[i].Pair[j].q = value[i][j];
                    }
                }
            }
        }

        private void CalcTauTilde()
        {
            for (int i = 0; i < tau.Length; i++)
            {
                for (int j = 0; j < tau[i].Length; j++)
                {
                    if (0 <= tauOld[i][j] * tau[i][j])
                    {
                        tauTilde[i][j] -= Math.Abs(tau[i][j]);
                        if (tauTilde[i][j] < 0) tauTilde[i][j] = 0;
                    }
                    else
                    {
                        tauTilde[i][j] += Math.Abs(tau[i][j]);
                    }
                }
            }
        }

        public bool IsCollisionBody(params ICollision[] obstacle)
        {
            for (int c = 0; c < obstacle.Length; c++)
            {
                for (int i = 0; i < Chain.Length; i++)
                {
                    for (int j = 0; j < Chain[i].DOF; j++)
                    {
                        if (Chain[i].Pair[j].IsCollision(obstacle[c])) return true;
                    }
                }
            }

            return false;
        }

        public bool IsCollisionEffc(params ICollision[] obstacle)
        {
            for (int c = 0; c < obstacle.Length; c++)
            {
                for (int i = 0; i < Chain.Length; i++)
                {
                    if (Chain[i].PairEnd.IsCollision(obstacle[c])) return true;
                }
            }

            return false;
        }

        public bool IsCollisionSelf()
        {
            for (int I = 0; I < Chain.Length; I++)
            {
                for (int i = 0; i < Chain.Length; i++)
                {
                    for (int j = 0; j < Chain[i].DOF + 1; j++)
                    {
                        if (I == i && Chain[i].DOF - 1 <= j) break;
                        else if (I == 0 && 0 < i && j == 0) break;

                        if (Chain[i].Pair[j].IsCollision(Chain[I].PairEnd.p.Get, 0.5f * Chain[I].PairEnd.JointObj.Radius)) return true;
                        if (Chain[i].Pair[j].IsCollision(Chain[I].PairEnd.pc.Get, 0.5f * Chain[I].PairEnd.JointObj.Radius)) return true;
                    }
                }
            }

            return false;
        }

        private void CalcRegressorMatrix()
        {
            for (int m = 0; m < Chain[0].DOF; m++)
            {
                for (int j = 0; j <= m; j++)
                {
                    if (Chain[0].Pair[j].IsRevolute || Chain[0].Pair[j].IsParallelA)
                    {
                        Y.Ref[0, 0].Ref[j, m] = -(Chain[0].Pair[j].Axis * (Chain[0].Pair[m + 1].p - Chain[0].Pair[j].p))[2];
                    }
                    else if (Chain[0].Pair[j].IsPrismatic)
                    {
                        Y.Ref[0, 0].Ref[j, m] = -Chain[0].Pair[j].Axis[2];
                    }
                    else
                    {
                        Y.Ref[0, 0].Ref[j, m] = 0;
                    }
                }
            }

            for (int i = 1; i < Chain.Length; i++)
            {
                for (int m = 0; m < Chain[i].DOF; m++)
                {
                    for (int j = 0; j < Chain[0].DOF; j++)
                    {
                        if (Chain[0].Pair[j].IsRevolute || Chain[0].Pair[j].IsParallelA)
                        {
                            Y.Ref[0, i].Ref[j, m] = -(Chain[0].Pair[j].Axis * (Chain[i].Pair[m + 1].p - Chain[0].Pair[j].p.Get))[2];
                        }
                        else if (Chain[0].Pair[j].IsPrismatic)
                        {
                            Y.Ref[0, i].Ref[j, m] = -Chain[0].Pair[j].Axis[2];
                        }
                        else
                        {
                            Y.Ref[0, i].Ref[j, m] = 0;
                        }
                    }

                    for (int j = 0; j <= m; j++)
                    {
                        if (Chain[i].Pair[j].IsRevolute || Chain[i].Pair[j].IsParallelA)
                        {
                            Y.Ref[i, i].Ref[j, m] = -(Chain[i].Pair[j].Axis * (Chain[i].Pair[m + 1].p - Chain[i].Pair[j].p.Get))[2];
                        }
                        else if (Chain[i].Pair[j].IsPrismatic)
                        {
                            Y.Ref[i, i].Ref[j, m] = -Chain[i].Pair[j].Axis[2];
                        }
                        else
                        {
                            Y.Ref[i, i].Ref[j, m] = 0;
                        }
                    }
                }
            }

        }

        public BlockVector CalcTauSelf()
        {
            CalcRegressorMatrix();

            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF; j++)
                {
                    mg.Ref[i].Ref[j] = Chain[i].Pair[j].Mass * g;
                }
            }

            tauSelf = Y * mg;

            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF; j++)
                {
                    Chain[i].Pair[j].TauSelf = tauSelf.Ref[i].Ref[j];
                }
            }

            return tauSelf;
        }

        public void RecordCurrentJointAngles()
        {
            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF; j++)
                {
                    Chain[i].Pair[j].qRec = Chain[i].Pair[j].q;
                }
            }
        }

        public void ReadRecordJointAngles()
        {
            for (int i = 0; i < Chain.Length; i++)
            {
                for (int j = 0; j < Chain[i].DOF; j++)
                {
                    Chain[i].Pair[j].q = Chain[i].Pair[j].qRec;
                }
            }

            ForwardKinematics();
        }

    }

    public class ErrorVector : BlockVector
    {
        public ErrorVector(int[] rows) : base(rows) { }

        private RotationMatrix dRRT = new RotationMatrix();
        private Vector dn = new Vector(3);
        private Vector dne = new Vector(3);
        private Vector no = new Vector(3);
        public void Update(Chain[] chain, Target[] target)
        {
            for (int i = 0; i < chain.Length; i++)
            {
                if (target[i].DOF == 6)
                {
                    dRRT = target[i].Rotate * chain[i].Re.Trans;
                    vector[i] = (target[i].Position - chain[i].pe) & dRRT.AngleAxisVector;
                }
                else if (target[i].DOF == 5)
                {
                    dn.Set = target[i].Rotate.GetColumn(2);
                    dne.Set = chain[i].Re.GetColumn(2);
                    no = (dn * dne).Normalize;
                    vector[i] = (target[i].Position - chain[i].pe) & (-dn.FormedAngle(dne) * no);
                }
                else if (target[i].DOF == 3)
                {
                    vector[i] = (target[i].Position - chain[i].pe) & new Vector(3);
                }
                else
                {
                    vector[i].SetZeroVector();
                }
            }
        }

    }

    public class VirtualSpringMatrix : BlockMatrix
    {
        private float[] zeta;

        public VirtualSpringMatrix(int[] rows, int[] columns) : base(rows, columns)
        {
            zeta = new float[rows.Length];
            for (int i = 0; i < zeta.Length; i++) zeta[i] = 1;
        }

        public void Update(Chain[] chain, Target[] target)
        {
            float lengthMax = chain[0].LinkLength;

            for (int i = 1; i < chain.Length; i++)
            {
                if (lengthMax < chain[i].LinkLength) lengthMax = chain[i].LinkLength;
            }

            float Kf, Km;
            for (int i = 0; i < chain.Length; i++)
            {
                if (0 < target[i].DOF) Kf = zeta[i] * 1;
                else Kf = 0;

                if (3 < target[i].DOF) Km = zeta[i] * lengthMax * lengthMax / (float)(2 * Math.PI);
                else Km = 0;

                Ref[i, i].SetDiagonal(new float[6] { Kf, Kf, Kf, Km, Km, Km });
            }
        }

        public void ZetaUpdate(Chain[] chain, Target[] target)
        {
            for (int i = 0; i < target.Length; i++)
            {
                if (target[i].Priority == false && 0.249f < zeta[i])
                {
                    zeta[i] -= 0.25f;
                }
            }

            Update(chain, target);
        }

        public bool IsZetaConverged(Target[] target)
        {
            for (int i = 0; i < target.Length; i++)
            {
                if (0 < target[i].DOF && !target[i].Priority && 0.01f < zeta[i]) return false;
            }

            return true;
        }

        public void ZetaReset(Chain[] chain, Target[] target)
        {
            for (int i = 0; i < zeta.Length; i++) { zeta[i] = 1; }
            Update(chain, target);
        }

    }



    // hayato：ここから

    public class JointTarget : Vector   // 関節の目標変位のクラス
    {
        public bool Priority = false;    // 優先度はfalseをデフォルトに

        public JointTarget(int rows) : base(rows) { }

        float[] currentJointTmp;    // 現在の関節の変位を一時的に格納するための配列
        public void SetCurrentJoint(Robot robot, int chainTag)     // 関節の目標変位を, 任意の直鎖における現在の関節変位で指定する関数
        {
            currentJointTmp = new float[robot.Kinematics.Chain[chainTag].DOF];

            for (int i = 0; i < robot.Kinematics.Chain[chainTag].DOF; i++)
            {
                currentJointTmp[i] = robot[chainTag, i].q;
            }

            Ref = currentJointTmp;
        }
    }

    public class JointErrorVector : BlockVector     // 関節変位の偏差のクラス, ErrorVectorクラスを参考に作成
    {
        public JointErrorVector(int[] rows) : base(rows) { }

        public void Update(Chain[] chain, JointTarget[] jointTarget)
        {
            for (int i = 0; i < chain.Length; i++)
            {
                for (int j = 0; j < chain[i].DOF; j++)
                {
                    vector[i][j] = jointTarget[i][j] - chain[i].Pair[j].q;
                }
            }
        }
    }

    public class VirtualJointSpringMatrix : BlockMatrix     // 仮想関節バネ行列のクラス, VirtualSpringMatrixクラスを参考に作成
    {
        private float[] zeta_J;
        public float lengthMax;

        public VirtualJointSpringMatrix(int[] rows, int[] columns) : base(rows, columns)
        {
            zeta_J = new float[rows.Length];
            for (int i = 0; i < zeta_J.Length; i++) zeta_J[i] = 1;
        }

        public void Update(Chain[] chain)
        {
            lengthMax = chain[0].LinkLength;
            float[] diagTmp;

            for (int i = 1; i < chain.Length; i++)
            {
                if (lengthMax < chain[i].LinkLength) lengthMax = chain[i].LinkLength;
            }

            for (int i = 0; i < chain.Length; i++)
            {
                diagTmp = (zeta_J[i] * chain[i].VirtualJointSpringDiag).Get;

                Ref[i, i].SetDiagonal(diagTmp);
            }
        }

        public void Zeta_JUpdate(Chain[] chain, JointTarget[] jointTarget)
        {
            for (int i = 0; i < jointTarget.Length; i++)
            {
                if (jointTarget[i].Priority == false && 0.249f < zeta_J[i])
                {
                    zeta_J[i] -= 0.25f;
                }
            }

            Update(chain);
        }

        public bool IsZeta_JConverged(JointTarget[] jointTarget)
        {
            for (int i = 0; i < jointTarget.Length; i++)
            {
                if (!jointTarget[i].Priority && 0.01f < zeta_J[i]) return false;
            }

            return true;
        }

        public void Zeta_JReset(Chain[] chain)
        {
            for (int i = 0; i < zeta_J.Length; i++) { zeta_J[i] = 1; }
            Update(chain);
        }
    }

    // hayato：ここまで



    public class JacobianMatrix : BlockMatrix
    {
        public JacobianMatrix(int[] rows, int[] columns) : base(rows, columns) { }

        private Vector sj = new Vector(6);
        private Vector bj = new Vector(6);

        public void Update(Chain[] chain)
        {
            for (int i = 0; i < chain.Length; i++)
            {
                for (int j = 0; j < chain[0].DOF; j++)
                {
                    if (chain[0].Pair[j].IsLocked)
                    {
                        sj.SetZeroVector();
                    }
                    else
                    {
                        if (chain[0].Pair[j].IsRevolute)
                        {
                            sj = (chain[0].Pair[j].Axis * (chain[i].pe - chain[0].Pair[j].p)) & chain[0].Pair[j].Axis;
                        }
                        else if (chain[0].Pair[j].IsPrismatic)
                        {
                            sj = chain[0].Pair[j].Axis & new float[3];
                        }
                        else if (chain[0].Pair[j].IsParallelA)
                        {
                            sj = (chain[0].Pair[j].Axis * chain[0].Pair[j + 1].Link) & new float[3];
                        }
                        else
                        {
                            sj.SetZeroVector();
                        }
                    }

                    Ref[i, 0].SetColumn(j, sj.Get);
                }
            }

            for (int i = 1; i < chain.Length; i++)
            {
                for (int j = 0; j < chain[i].DOF; j++)
                {
                    if (chain[i].Pair[j].IsLocked)
                    {
                        bj.SetZeroVector();
                    }
                    else
                    {
                        if (chain[i].Pair[j].IsRevolute)
                        {
                            bj = chain[i].Pair[j].Axis * (chain[i].pe - chain[i].Pair[j].p) & chain[i].Pair[j].Axis;
                        }
                        else if (chain[i].Pair[j].IsPrismatic)
                        {
                            bj = chain[i].Pair[j].Axis & new float[3];
                        }
                        else if (chain[i].Pair[j].IsParallelA)
                        {
                            bj = (chain[i].Pair[j].Axis * chain[i].Pair[j + 1].Link) & new float[3];
                        }
                        else
                        {
                            bj.SetZeroVector();
                        }
                    }

                    Ref[i, i].SetColumn(j, bj.Get);
                }
            }
        }

        public void ReflectJointMobility(Chain[] chain, BlockVector tau)
        {
            for (int i = 0; i < chain.Length; i++)
            {
                for (int j = 0; j < chain[0].DOF; j++)
                {
                    if (chain[0].Pair[j].IsDrivable(tau[0][j]) == false)
                    {
                        Ref[i, 0].SetColumn(j, new float[6]);
                    }
                }
            }

            for (int i = 1; i < chain.Length; i++)
            {
                for (int j = 0; j < chain[i].DOF; j++)
                {
                    if (chain[i].Pair[j].IsDrivable(tau[i][j]) == false)
                    {
                        Ref[i, i].SetColumn(j, new float[6]);
                    }
                }
            }
        }

    }

    public class Target : PrickleBall
    {
        public bool Priority = true;

        public int DOF { get; private set; } = 6;
        public void SetDOF6() { DOF = 6; }
        public void SetDOF5() { DOF = 5; }
        public void SetDOF3() { DOF = 3; }
        public void SetDOF0() { DOF = 0; }

        public Target(float diameter)
        {
            Diameter = diameter;
            Color.SetDarkGray();
            Rotate.SetRy(0.5f * (float)Math.PI);
        }

        new public void Draw()
        {
            if (0 < DOF)
            {
                if (DOF == 6) base.ConeNum = 3;
                else if (DOF == 5) base.ConeNum = 1;
                else if (DOF == 3) base.ConeNum = 0;
                base.Draw();
            }
        }
    }

    public class Chain
    {
        public Pair[] Pair;
        public PrickleBall EndPoint = new PrickleBall();

        public Vector pe = new Vector(3);
        public RotationMatrix Re = new RotationMatrix();


        // hayato：ここから

        public Vector peOffset = new Vector(3);    // peのオフセット
        public RotationMatrix ReOffset = new RotationMatrix();     // Reのオフセット

        // hayato：ここまで


        public Chain(int DOF)
        {
            Pair = new Pair[DOF + 1];

            for (int i = 0; i < Pair.Length; i++)
            {
                Pair[i] = new Pair();
            }

            EndPoint.Position = pe;
            EndPoint.Rotate = Re;
        }

        public int DOF { get { return Pair.Length - 1; } }

        public Pair PairEnd { get { return Pair[DOF]; } }

        public float LinkLength
        {
            get
            {
                float result = 0;
                for (int i = 0; i < Pair.Length; i++) { result += Pair[i].LinkInit.Norm; }
                return result;
            }
        }

        public void Draw()
        {
            Pair[0].DrawBaseLink();

            for (int i = 0; i < DOF; i++)
            {
                Pair[i].DrawLink();
                Pair[i].DrawJoint();
            }

            Pair[DOF].DrawEndLink();

            //EndPoint.Diameter = Pair[DOF].JointObj.Radius;    // hayato
            EndPoint.Draw();
        }


        // hayato：ここから

        private Vector virtualJointSpringDiag;   // 仮想関節バネ定数行列の対角成分
        public Vector VirtualJointSpringDiag { get { return virtualJointSpringDiag; } }     // 読み取り用
        public void SetVirtualJointSpringDiag(float[] diag)    // 仮想関節バネ定数行列の対角成分を指定する関数
        {
            virtualJointSpringDiag = diag;

            if (virtualJointSpringDiag.Length != DOF)
            {
                Console.WriteLine("Error : Size of array do not match. It has to match with the DOF of chain.");
            }
        }

        public void SetPositionOffsetToEffector(float x = 0, float y = 0, float z = 0)      // エンドエフェクタに位置のオフセットを設ける関数. そのまま実行するとオフセットのリセットができる
        {
            peOffset.SetValue(x, y, z);
        }

        public void SetRotateOffsetToEffector(float roll = 0, float pitch = 0, float yaw = 0)       // エンドエフェクタに姿勢のオフセットを設ける関数. そのまま実行するとオフセットのリセットができる
        {
            ReOffset.SetRollPitchYaw(roll, pitch, yaw);
        }

        // hayato：ここまで

    }

    public class Pair
    {
        public Vector p = new Vector(3);
        public RotationMatrix R = new RotationMatrix();
        public Vector Link = new Vector(3);
        public Vector LinkInit = new Vector(3);
        public Vector Axis = new Vector(3);
        public Vector AxisInit = new Vector(3);

        public Vector pt = new Vector(3);
        public Vector pc = new Vector(3);
        public Vector pb = new Vector(3);

        public RobotObject.Joint JointObj;
        public RobotObject.Link LinkObj;

        public float Mass = 0.2f;
        public float TauSelf = 0;

        public Pair(float jointRadius = 0.04f)
        {
            JointObj = new RobotObject.Joint(ref AxisInit);
            LinkObj = new RobotObject.Link(ref LinkInit);

            LinkInit[2] = 0.2f;
            AxisInit.SetUnitVectorY();

            JointObj.Radius = jointRadius;
            JointObj.Position = p;
            JointObj.Rotate = R;

            LinkObj.Radius = jointRadius;
            LinkObj.Position = pc;
            LinkObj.Rotate = R;
        }

        public void DrawLink() { LinkObj.Draw(JointObj.Radius); }

        public void DrawBaseLink() { LinkObj.DrawBase(JointObj.Radius); }

        public void DrawEndLink() { LinkObj.DrawEnd(JointObj.Radius); }

        public void DrawJoint()
        {
            if (IsLimited) JointObj.EnableLimitedColor();
            else JointObj.DisableLimitedColor();

            if (IsRevolute || IsParallelA || IsParallelB) JointObj.DrawRevolute();
            else if (IsPrismatic) JointObj.DrawPrismatic(q);
        }

        private float[][] CheckPoint
        {
            get
            {
                if (0.001f < LinkObj.Radius)
                {
                    int num = (int)(LinkInit.Norm / (2 * LinkObj.Radius)) + 1;

                    float[][] result = new float[num][];
                    float t = 1f / (num + 1);

                    for (int i = 0; i < num; i++)
                    {
                        result[i] = p.Plus(-t * Link);
                        t += 1f / (num + 1);
                    }

                    return result;
                }
                else
                {
                    return new float[0][];
                }
            }
        }

        public bool IsCollision(float[] position, float threshold = 0)
        {
            if (0.001f < LinkInit.AbsSum)
            {
                float d = p.Distance(position);

                if (threshold + JointObj.Radius + LinkInit.AbsSum < d) return false;
                else if (d < threshold + JointObj.Radius) return true;
                else
                {
                    float[][] checkPoint = CheckPoint;

                    for (int i = 0; i < checkPoint.GetLength(0); i++)
                    {
                        if (Float3.Distance(checkPoint[i], position) < threshold + LinkObj.Radius) return true;
                    }
                }
            }

            return false;
        }

        public bool IsCollision(ICollision obstacle, float threshold = 0)
        {
            if (0.001f < LinkInit.AbsSum)
            {
                if (!obstacle.IsCollision(p.Get, threshold + JointObj.Radius + LinkInit.AbsSum)) return false;
                else if (obstacle.IsCollision(p.Get, threshold + JointObj.Radius)) return true;
                else
                {
                    float[][] checkPoint = CheckPoint;

                    for (int i = 0; i < checkPoint.GetLength(0); i++)
                    {
                        if (obstacle.IsCollision(checkPoint[i], threshold + LinkObj.Radius)) return true;
                    }
                }
            }

            return false;
        }

        public float[] JointRange = new float[2] { -0.75f * (float)Math.PI, 0.75f * (float)Math.PI };

        public float JointRangeMin
        {
            get { return JointRange[0]; }
            set { JointRange[0] = value; }
        }

        public float JointRangeMax
        {
            get { return JointRange[1]; }
            set { JointRange[1] = value; }
        }

        public void SetJointRangeInfinite()
        {
            JointRange[0] = float.MinValue;
            JointRange[1] = float.MaxValue;
        }

        public float qHome, qRec;
        private float _q;
        public float q
        {
            get { return _q; }
            set
            {
                if (float.IsNaN(value))
                {
                    Console.WriteLine("Error : NaN is assigned to q");
                }
                else
                {
                    if (value <= JointRange[0])
                    {
                        _q = JointRange[0];
                        IsLimited = true;
                    }
                    else if (JointRange[1] <= value)
                    {
                        _q = JointRange[1];
                        IsLimited = true;
                    }
                    else
                    {
                        _q = value;
                        IsLimited = false;
                    }
                }
            }
        }

        public bool IsLocked = false;

        public bool IsLimited { get; private set; } = false;

        public bool IsDrivable(float direction)
        {
            if (IsLimited)
            {
                if (q <= 0.5f * (JointRange[0] + JointRange[1]) && direction <= 0) return false;
                else if (0.5f * (JointRange[0] + JointRange[1]) <= q && 0 <= direction) return false;
            }

            return true;
        }

        private byte jointTypeID = 0;

        public void SetRevolute() { jointTypeID = 0; }

        public void SetPrismatic() { jointTypeID = 1; }

        public void SetParallelA() { jointTypeID = 2; }

        public void SetParallelB() { jointTypeID = 3; }

        public bool IsRevolute { get { return jointTypeID == 0; } }

        public bool IsPrismatic { get { return jointTypeID == 1; } }

        public bool IsParallelA { get { return jointTypeID == 2; } }

        public bool IsParallelB { get { return jointTypeID == 3; } }

        public float[] qTars = new float[100];
        private float qTarsInit;
        internal void ProceedToTargets(int qTarNum, float val0to1, bool isFirst)
        {
            if (isFirst)
            {
                if (qTarNum == 0) qTarsInit = q;
                else qTarsInit = qTars[qTarNum - 1];
            }

            q = qTarsInit + val0to1 * (qTars[qTarNum] - qTarsInit);
        }

        public float qTar { get { return qTars[0]; } set { qTars[0] = value; } }

    }

}
