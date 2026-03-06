using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class UR5RobotController : MonoBehaviour
{
    [Header("Robot Visuals")]
    public Material linkMaterial;
    public Material jointMaterial;
    public float jointSize = 0.08f;
    public float linkRadius = 0.04f;

    [HideInInspector] public Transform[] jointTransforms = new Transform[6];
    
    // Internal kinematic parameters in double precision for accuracy
    private double[][] H_double; 
    private double[][] P_double;
    private Vector3[] H_float; 

    void Awake() {
        DefineKinematics();
        BuildRobotModel();
    }

    void DefineKinematics() {
        // Double precision primitives
        double[] ex = { 1, 0, 0 };
        double[] ey = { 0, 0, 1 };
        double[] ez = { 0, 1, 0 };

        H_double = new double[][] { ez, ey, ey, ey, Mul(ez, -1), ey };
        P_double = new double[][] {
            Mul(ez, 0.1625),
            new double[] { 0, 0, 0 },
            Mul(ex, -0.425),
            Mul(ex, -0.3922),
            Add(Mul(ey, -0.1333), Mul(ez, -0.0997)),
            new double[] { 0, 0, 0 },
            Mul(ey, -0.0996)
        };

        // Cache float versions for Unity Transform operations
        H_float = new Vector3[] { Vector3.up, Vector3.forward, Vector3.forward, Vector3.forward, Vector3.down, Vector3.forward };
    }

    void BuildRobotModel() {
        Transform lastParent = this.transform;
        for (int i = 0; i < 6; i++) {
            GameObject jointObj = new GameObject("Joint_" + (i + 1));
            jointObj.transform.parent = lastParent;
            
            // Convert double P to float Vector3 for Unity
            Vector3 pos = new Vector3((float)P_double[i][0], (float)P_double[i][1], (float)P_double[i][2]);
            jointObj.transform.localPosition = pos;
            jointObj.transform.localRotation = Quaternion.identity;
            jointTransforms[i] = jointObj.transform;

/*#if UNITY_EDITOR
            Texture2D icon = UnityEditor.EditorGUIUtility.IconContent("sv_label_0").image as Texture2D;
            UnityEditor.EditorGUIUtility.SetIconForObject(jointObj, icon);
#endif*/

            // Visual Sphere
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.parent = jointObj.transform;
            sphere.transform.localPosition = Vector3.zero;
            sphere.transform.localScale = Vector3.one * jointSize;
            sphere.GetComponent<MeshRenderer>().material = jointMaterial;
            Destroy(sphere.GetComponent<SphereCollider>());

            // Visual Cylinder
            Vector3 nextOffset = (i < 5) ? 
                new Vector3((float)P_double[i+1][0], (float)P_double[i+1][1], (float)P_double[i+1][2]) : 
                new Vector3((float)P_double[6][0], (float)P_double[6][1], (float)P_double[6][2]);

            if (nextOffset.magnitude > 0.01f) {
                GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                cylinder.transform.parent = jointObj.transform;
                cylinder.transform.localPosition = nextOffset / 2.0f;
                cylinder.transform.localRotation = Quaternion.FromToRotation(Vector3.up, nextOffset.normalized);
                cylinder.transform.localScale = new Vector3(linkRadius, nextOffset.magnitude / 2.0f, linkRadius);
                cylinder.GetComponent<MeshRenderer>().material = linkMaterial;
                Destroy(cylinder.GetComponent<CapsuleCollider>());
            }
            lastParent = jointObj.transform;
        }
    }

    public void ApplyJointAngles(float[] q_radians) {
        for (int i = 0; i < 6; i++) {
            // Apply rotation q[i] around axis H[i]
            jointTransforms[i].localRotation = Quaternion.AngleAxis(q_radians[i] * Mathf.Rad2Deg, H_float[i]);
        }
    }

    /// <summary>
    /// Solves IK and returns the solution closest to the current joint angles
    /// </summary>
    public double[] SolveIK(Matrix4x4 R_0T_unity, Vector3 p_0T_unity, float[] current_q) {
        // Convert Unity inputs to double precision
        double[,] R_0T = UnityMatrixToDouble(R_0T_unity);
        double[] p_0T = { p_0T_unity.x, p_0T_unity.y, p_0T_unity.z };

        // 1. Tool Offset correction
        double[,] R_6T = RotDouble(new double[] { 1, 0, 0 }, Math.PI / 2.0);
        double[,] R_06 = MatMul(R_0T, Transpose(R_6T));
        
        // 2. Wrist Position
        double[] p_06 = Sub(Sub(p_0T, P_double[0]), Multiply(R_06, P_double[6]));

        // 3. Find all potential solutions
        List<double[]> allSolutions = new List<double[]>();
        double[] sumP25 = Add(Add(Add(P_double[1], P_double[2]), P_double[3]), P_double[4]);
        
        var (theta1, _) = SP4D(H_double[1], p_06, Mul(H_double[0], -1), Dot(H_double[1], sumP25));

        foreach (double q1 in theta1) {
            double[,] R_01 = RotDouble(H_double[0], q1);
            double[] targetV5 = Multiply(Transpose(R_01), Multiply(R_06, H_double[5]));
            var (theta5, _) = SP4D(H_double[1], H_double[5], H_double[4], Dot(H_double[1], targetV5));

            foreach (double q5 in theta5) {
                double[,] R_45 = RotDouble(H_double[4], q5);
                double q_14 = SP1D(Multiply(R_45, H_double[5]), Multiply(Transpose(R_01), Multiply(R_06, H_double[5])), H_double[1]);
                double q6 = SP1D(Multiply(Transpose(R_45), H_double[1]), Multiply(Transpose(R_06), Multiply(R_01, H_double[1])), Mul(H_double[5], -1));
                
                double[] d_inner = Sub(Sub(Multiply(Transpose(R_01), p_06), P_double[1]), Multiply(RotDouble(H_double[1], q_14), P_double[4]));
                var (theta3, _) = SP3D(Mul(P_double[3], -1), P_double[2], H_double[1], Norm(d_inner));

                foreach (double q3 in theta3) {
                    double q2 = SP1D(Add(P_double[2], Multiply(RotDouble(H_double[1], q3), P_double[3])), d_inner, H_double[1]);
                    double q4 = q_14 - q2 - q3;
                    allSolutions.Add(new double[] { WrapToPi(q1), WrapToPi(q2), WrapToPi(q3), WrapToPi(q4), WrapToPi(q5), WrapToPi(q6) });
                }
            }
        }

        if (allSolutions.Count == 0) return null;

        // 4. Select the solution with minimum joint distance to current_q (smoothest motion)
        double minDistance = double.MaxValue;
        double[] bestSol = allSolutions[0];

        foreach (var sol in allSolutions) {
            double dist = 0;
            for (int i = 0; i < 6; i++) {
                double diff = Math.Abs(sol[i] - current_q[i]);
                if (diff > Math.PI) diff = 2 * Math.PI - diff;
                dist += diff * diff;
            }
            if (dist < minDistance) {
                minDistance = dist;
                bestSol = sol;
            }
        }

        return bestSol;
        /*// Convert back to float for Unity
        float[] finalQ = new float[6];
        for (int i = 0; i < 6; i++) finalQ[i] = (float)bestSol[i];
        return finalQ;*/
    }

    #region Math Subproblems (Double Precision)
    double WrapToPi(double a) => Math.Atan2(Math.Sin(a), Math.Cos(a));
    
    double SP1D(double[] p1, double[] p2, double[] k) => Math.Atan2(Dot(Cross(k, p1), p2), Dot(Mul(Cross(k, Cross(k, p1)), -1), p2));
    
    (double[], bool) SP3D(double[] p1, double[] p2, double[] k, double d) => SP4D(p2, p1, k, 0.5 * (Dot(p1, p1) + Dot(p2, p2) - d * d));
    
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
    #endregion

    #region Double Precision Matrix/Vector Helpers
    double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    double[] Cross(double[] a, double[] b) => new double[] { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
    double[] Add(double[] a, double[] b) => new double[] { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
    double[] Sub(double[] a, double[] b) => new double[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
    double[] Mul(double[] a, double s) => new double[] { a[0] * s, a[1] * s, a[2] * s };
    double Norm(double[] a) => Math.Sqrt(Dot(a, a));
    double[] Multiply(double[,] R, double[] v) => new double[] { R[0, 0] * v[0] + R[0, 1] * v[1] + R[0, 2] * v[2], R[1, 0] * v[0] + R[1, 1] * v[1] + R[1, 2] * v[2], R[2, 0] * v[0] + R[2, 1] * v[1] + R[2, 2] * v[2] };
    double[,] Transpose(double[,] m) => new double[,] { { m[0, 0], m[1, 0], m[2, 0] }, { m[0, 1], m[1, 1], m[2, 1] }, { m[0, 2], m[1, 2], m[2, 2] } };
    double[,] MatMul(double[,] A, double[,] B) {
        double[,] C = new double[3, 3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++) C[i, j] += A[i, k] * B[k, j];
        return C;
    }
    double[,] RotDouble(double[] k, double theta) {
        double c = Math.Cos(theta), s = Math.Sin(theta), v = 1 - c;
        return new double[,] {
            { c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s },
            { k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s },
            { k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v }
        };
    }
    double[,] UnityMatrixToDouble(Matrix4x4 m) {
        return new double[,] { { m.m00, m.m01, m.m02 }, { m.m10, m.m11, m.m12 }, { m.m20, m.m21, m.m22 } };
    }
    #endregion
}