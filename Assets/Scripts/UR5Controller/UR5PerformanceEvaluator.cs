using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System;
using System.Linq;

public class UR5PerformanceEvaluator : MonoBehaviour
{
    public UR5RobotController robot;
    public int numTests = 10000;

    private double[][] H;
    private double[][] P;
    private double[,] R_6T_ref;

    void Start()
    {
        if (robot == null) robot = GetComponent<UR5RobotController>();
        SetupReferenceKinematics();
        UnityEngine.Debug.Log("<color=orange><b>[Evaluation]</b> Starting Synchronized Performance Test...</color>");
        StartCoroutine(RunEvaluationRoutine());
    }

    void SetupReferenceKinematics()
    {
        // Must match UR5RobotController exactly
        double[] ex = { 1, 0, 0 };
        double[] ey = { 0, 0, 1 };
        double[] ez = { 0, 1, 0 };

        H = new double[][] { ez, ey, ey, ey, Mul(ez, -1), ey };
        P = new double[][] {
            Mul(ez, 0.1625),                         // p01
            new double[] { 0, 0, 0 },               // p12
            Mul(ex, -0.425),                        // p23
            Mul(ex, -0.3922),                       // p34
            Add(Mul(ey, -0.1333), Mul(ez, -0.0997)),// p45
            new double[] { 0, 0, 0 },               // p56
            Mul(ey, -0.0996)                        // p6T
        };

        // This matches the 90deg rotation around X used in the Controller
        R_6T_ref = RotDouble(ex, Math.PI / 2.0);
    }

    IEnumerator RunEvaluationRoutine()
    {
        yield return new WaitForSeconds(0.5f);

        double totalJointError = 0;
        double totalPosError = 0;
        int successCount = 0;
        long totalTicks = 0;
        Stopwatch sw = new Stopwatch();

        for (int i = 0; i < numTests; i++)
        {
            // 1. Generate Ground Truth (Tool Pose T, not just Joint 6)
            double[] q_true = new double[6];
            float[] q_current_seed = new float[6];
            for (int j = 0; j < 6; j++) {
                q_true[j] = UnityEngine.Random.Range(-(float)Math.PI, (float)Math.PI);
                q_current_seed[j] = (float)q_true[j]; 
            }

            var (R_0T, p_0T) = ForwardKinToolDouble(q_true);
            
            // 2. Prepare Unity inputs (Matrix4x4 and Vector3)
            Matrix4x4 targetR = DoubleToUnityMatrix(R_0T);
            Vector3 targetP = new Vector3((float)p_0T[0], (float)p_0T[1], (float)p_0T[2]);

            // 3. Timed Solve (Find 8 solutions -> Pick closest to seed)
            sw.Restart();
            double[] resultQ = robot.SolveIK(targetR, targetP, q_current_seed);
            sw.Stop();
            
            if (resultQ != null)
            {
                totalTicks += sw.ElapsedTicks;
                successCount++;

                totalJointError += JointDistance(q_true, resultQ);

                var (_, p_verify) = ForwardKinToolDouble(resultQ);
                totalPosError += Dist(p_0T, p_verify);
            }
        }

        double avgMicroseconds = (double)totalTicks / successCount / (Stopwatch.Frequency / 1000000.0);

        UnityEngine.Debug.Log("<color=cyan><b>--- UR5 PERFORMANCE REPORT (SYNCHRONIZED) ---</b></color>");
        UnityEngine.Debug.Log($"<b>Avg Time:</b> <color=yellow>{avgMicroseconds:F2} μs</color>");
        UnityEngine.Debug.Log($"<b>Joint Accuracy:</b> {(totalJointError / successCount):E2} rad");
        UnityEngine.Debug.Log($"<b>Pos Accuracy:</b> {(totalPosError / successCount):E2} m");
    }

    // FK that includes the Tool Frame R6T and P6T
    (double[,] R, double[] p) ForwardKinToolDouble(double[] q)
    {
        double[] p_res = P[0];
        double[,] R_res = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        for (int i = 0; i < 6; i++) {
            R_res = MatMul(R_res, RotDouble(H[i], q[i]));
            p_res = Add(p_res, Multiply(R_res, P[i + 1]));
        }
        // Apply final Tool Rotation R_0T = R_06 * R_6T
        R_res = MatMul(R_res, R_6T_ref);
        return (R_res, p_res);
    }

    #region Math Helpers
    double JointDistance(double[] a, double[] b) {
        double s = 0;
        for (int i = 0; i < 6; i++) {
            double d = Math.Abs(Math.Atan2(Math.Sin(a[i]), Math.Cos(a[i])) - Math.Atan2(Math.Sin(b[i]), Math.Cos(b[i])));
            if (d > Math.PI) d = 2 * Math.PI - d;
            s += d * d;
        }
        return Math.Sqrt(s);
    }
    double Dist(double[] a, double[] b) => Math.Sqrt(Math.Pow(a[0] - b[0], 2) + Math.Pow(a[1] - b[1], 2) + Math.Pow(a[2] - b[2], 2));
    double[] Multiply(double[,] R, double[] v) => new double[] { R[0,0]*v[0]+R[0,1]*v[1]+R[0,2]*v[2], R[1,0]*v[0]+R[1,1]*v[1]+R[1,2]*v[2], R[2,0]*v[0]+R[2,1]*v[1]+R[2,2]*v[2] };
    double[] Add(double[] a, double[] b) => new double[] { a[0]+b[0], a[1]+b[1], a[2]+b[2] };
    double[] Mul(double[] a, double s) => new double[] { a[0]*s, a[1]*s, a[2]*s };
    double[,] MatMul(double[,] A, double[,] B) {
        double[,] C = new double[3,3];
        for (int i=0; i<3; i++) for (int j=0; j<3; j++) for (int k=0; k<3; k++) C[i,j] += A[i,k]*B[k,j];
        return C;
    }
    double[,] RotDouble(double[] k, double theta) {
        double c = Math.Cos(theta), s = Math.Sin(theta), v = 1-c;
        return new double[,] {
            { c+k[0]*k[0]*v, k[0]*k[1]*v-k[2]*s, k[0]*k[2]*v+k[1]*s },
            { k[1]*k[0]*v+k[2]*s, c+k[1]*k[1]*v, k[1]*k[2]*v-k[0]*s },
            { k[2]*k[0]*v-k[1]*s, k[2]*k[1]*v+k[0]*s, c+k[2]*k[2]*v }
        };
    }
    Matrix4x4 DoubleToUnityMatrix(double[,] m) {
        Matrix4x4 res = Matrix4x4.identity;
        res.m00 = (float)m[0,0]; res.m01 = (float)m[0,1]; res.m02 = (float)m[0,2];
        res.m10 = (float)m[1,0]; res.m11 = (float)m[1,1]; res.m12 = (float)m[1,2];
        res.m20 = (float)m[2,0]; res.m21 = (float)m[2,1]; res.m22 = (float)m[2,2];
        return res;
    }
    #endregion
}