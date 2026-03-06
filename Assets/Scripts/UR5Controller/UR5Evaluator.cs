using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics; 
using System; 
using System.Linq;

public class UR5Evaluator : MonoBehaviour
{
    private double[][] H;
    private double[][] P;

    void Start()
    {
        SetupKinematics();
        RunFullEvaluation();
    }

    void SetupKinematics()
    {
        double[] ex = { 1, 0, 0 };
        double[] ey = { 0, 1, 0 };
        double[] ez = { 0, 0, 1 };

        H = new double[][] { ez, ey, ey, ey, Mul(ez, -1), ey };
        P = new double[][] {
            Mul(ez, 0.1625),
            new double[] { 0, 0, 0 },
            Mul(ex, -0.425),
            Mul(ex, -0.3922),
            Add(Mul(ey, -0.1333), Mul(ez, -0.0997)),
            new double[] { 0, 0, 0 },
            Mul(ey, -0.0996)
        };
    }

    public void RunFullEvaluation()
    {
        int numTests = 10000;
        double totalJointError = 0;
        double totalPosError = 0;
        int successCount = 0;

        UnityEngine.Debug.Log("<color=cyan><b>--- UR5 DOUBLE-PRECISION EVALUATION ---</b></color>");

        Stopwatch sw = Stopwatch.StartNew();
        for (int i = 0; i < numTests; i++)
        {
            double[] q_true = new double[6];
            for (int j = 0; j < 6; j++) q_true[j] = UnityEngine.Random.Range(-(float)Math.PI, (float)Math.PI);

            var (R_06, p_0T) = ForwardKinDouble(q_true);
            List<double[]> solutions = SolveIKDouble(R_06, p_0T);

            if (solutions.Count > 0)
            {
                successCount++;
                double minJ = double.MaxValue;
                double minP = double.MaxValue;

                foreach (var sol in solutions)
                {
                    minJ = Math.Min(minJ, JointDistance(q_true, sol));
                    var (_, p_verify) = ForwardKinDouble(sol);
                    minP = Math.Min(minP, Dist(p_0T, p_verify));
                }
                totalJointError += minJ;
                totalPosError += minP;
            }
        }
        sw.Stop();

        // Fix locale formatting for microseconds
        double timePerPose = (sw.Elapsed.TotalMilliseconds * 1000.0) / numTests;

        UnityEngine.Debug.Log($"<b>Success Rate:</b> {(successCount / (float)numTests) * 100:F2}%");
        UnityEngine.Debug.Log($"<b>Avg Speed:</b> {timePerPose:F2} μs / pose");
        UnityEngine.Debug.Log($"<b>Joint Accuracy:</b> {(totalJointError/successCount):E2} rad");
        UnityEngine.Debug.Log($"<b>Pos Accuracy:</b> {(totalPosError/successCount):E2} m");

        RunSingularityTest();
    }

    void RunSingularityTest()
    {
        UnityEngine.Debug.Log("<b>Checking Singularity Robustness (q=0)...</b>");
        double[] q0 = { 0, 0, 0, 0, 0, 0 };
        var (R06, p0T) = ForwardKinDouble(q0);
        List<double[]> sols = SolveIKDouble(R06, p0T);

        // In a singularity, we check if the SOLUTION reached the same physical POSITION, 
        // even if the angles (q) are different (due to redundancy).
        double minPosErr = double.MaxValue;
        foreach(var s in sols)
        {
            var (_, p_verify) = ForwardKinDouble(s);
            minPosErr = Math.Min(minPosErr, Dist(p0T, p_verify));
        }

        if (minPosErr < 1e-12)
            UnityEngine.Debug.Log("<color=green><b>Singularity Test Passed:</b> IK successfully reached the q=0 position.</color>");
        else
            UnityEngine.Debug.Log($"<color=red><b>Singularity Test Failed:</b> Position error {minPosErr:E2}</color>");
    }

    public List<double[]> SolveIKDouble(double[,] R_06, double[] p_0T)
    {
        List<double[]> Q = new List<double[]>();
        double[] p_06 = Sub(Sub(p_0T, P[0]), Multiply(R_06, P[6]));
        double[] sumP25 = Add(Add(Add(P[1], P[2]), P[3]), P[4]);

        var (theta1, _) = SP4D(H[1], p_06, Mul(H[0], -1), Dot(H[1], sumP25));
        foreach (double q1 in theta1)
        {
            double[,] R_01 = RotDouble(H[0], q1);
            double[] targetV5 = Multiply(Transpose(R_01), Multiply(R_06, H[5]));
            var (theta5, _) = SP4D(H[1], H[5], H[4], Dot(H[1], targetV5));
            foreach (double q5 in theta5)
            {
                double[,] R_45 = RotDouble(H[4], q5);
                double q_14 = SP1D(Multiply(R_45, H[5]), Multiply(Transpose(R_01), Multiply(R_06, H[5])), H[1]);
                double q6 = SP1D(Multiply(Transpose(R_45), H[1]), Multiply(Transpose(R_06), Multiply(R_01, H[1])), Mul(H[5], -1));
                double[] d_inner = Sub(Sub(Multiply(Transpose(R_01), p_06), P[1]), Multiply(RotDouble(H[1], q_14), P[4]));
                var (theta3, _) = SP3D(Mul(P[3], -1), P[2], H[1], Norm(d_inner));
                foreach (double q3 in theta3)
                {
                    double q2 = SP1D(Add(P[2], Multiply(RotDouble(H[1], q3), P[3])), d_inner, H[1]);
                    // WrapToPi ensures angles are in [-PI, PI]
                    Q.Add(new double[] { WrapToPi(q1), WrapToPi(q2), WrapToPi(q3), WrapToPi(q_14 - q2 - q3), WrapToPi(q5), WrapToPi(q6) });
                }
            }
        }
        return Q;
    }

    #region Math Primitives
    double WrapToPi(double a) => Math.Atan2(Math.Sin(a), Math.Cos(a));

    (double[], bool) SP4D(double[] h, double[] p, double[] k, double d) {
        double[] KxP = Cross(k, p);
        double Ay = Dot(h, KxP), Ax = Dot(h, Mul(Cross(k, KxP), -1)), b = d - Dot(h, k) * Dot(k, p);
        double normA2 = Ax * Ax + Ay * Ay;
        if (normA2 > b * b) {
            double xi = Math.Sqrt(Math.Max(0, normA2 - b * b));
            return (new double[] { Math.Atan2(Ay * b + Ax * xi, Ax * b - Ay * xi), Math.Atan2(Ay * b - Ax * xi, Ax * b + Ay * xi) }, false);
        }
        return (new double[] { Math.Atan2(Ay * b, Ax * b) }, true);
    }
    double SP1D(double[] p1, double[] p2, double[] k) => Math.Atan2(Dot(Cross(k, p1), p2), Dot(Mul(Cross(k, Cross(k, p1)), -1), p2));
    (double[], bool) SP3D(double[] p1, double[] p2, double[] k, double d) => SP4D(p2, p1, k, 0.5 * (Dot(p1, p1) + Dot(p2, p2) - d * d));
    
    (double[,], double[]) ForwardKinDouble(double[] q) {
        double[] p = P[0]; double[,] R = { {1,0,0}, {0,1,0}, {0,0,1} };
        for (int i = 0; i < 6; i++) {
            R = MatMul(R, RotDouble(H[i], q[i]));
            p = Add(p, Multiply(R, P[i+1]));
        }
        return (R, p);
    }

    double Dot(double[] a, double[] b) => a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    double[] Cross(double[] a, double[] b) => new double[] { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] };
    double[] Add(double[] a, double[] b) => new double[] { a[0]+b[0], a[1]+b[1], a[2]+b[2] };
    double[] Sub(double[] a, double[] b) => new double[] { a[0]-b[0], a[1]-b[1], a[2]-b[2] };
    double[] Mul(double[] a, double s) => new double[] { a[0]*s, a[1]*s, a[2]*s };
    double Norm(double[] a) => Math.Sqrt(Dot(a, a));
    double Dist(double[] a, double[] b) => Norm(Sub(a, b));
    double JointDistance(double[] a, double[] b) {
        double s = 0;
        for(int i=0; i<6; i++) {
            double d = Math.Abs(WrapToPi(a[i]) - WrapToPi(b[i]));
            if (d > Math.PI) d = 2 * Math.PI - d;
            s += d * d;
        }
        return Math.Sqrt(s);
    }
    double[] Multiply(double[,] R, double[] v) => new double[] { R[0,0]*v[0]+R[0,1]*v[1]+R[0,2]*v[2], R[1,0]*v[0]+R[1,1]*v[1]+R[1,2]*v[2], R[2,0]*v[0]+R[2,1]*v[1]+R[2,2]*v[2] };
    double[,] Transpose(double[,] m) => new double[,] { {m[0,0], m[1,0], m[2,0]}, {m[0,1], m[1,1], m[2,1]}, {m[0,2], m[1,2], m[2,2]} };
    double[,] MatMul(double[,] A, double[,] B) {
        double[,] C = new double[3,3];
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) for(int k=0; k<3; k++) C[i,j] += A[i,k]*B[k,j];
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
    #endregion
}