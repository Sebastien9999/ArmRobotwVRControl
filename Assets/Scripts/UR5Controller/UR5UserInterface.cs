using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UR5UserInterface : MonoBehaviour
{
    public enum ControlMode { InverseKinematics, ManualJoints, MATLABPresets }
    public enum MATLABTest { q_test_0, q_test_1, q_test_2 }
    private float[] activeAngles = new float[6]; // Current joint angles in radians for robot control in Unity
    private double[] activeAnglesDouble = new double[6]; // Precision for ROS communication
    public double[] ActiveAnglesDouble { get { return activeAnglesDouble; } }
    public float[] ActiveAngles { get { return activeAngles; } }

    [Header("Controller Setup")]
    public UR5RobotController robot;
    public Transform ikTarget;

    [Header("Control Settings")]
    public ControlMode mode = ControlMode.InverseKinematics;
    public MATLABTest activePreset = MATLABTest.q_test_0;

    [Header("Manual Angles (Degrees)")]
    [Range(-180, 180)] public float q1, q2, q3, q4, q5, q6;

    void Update()
    {
        if (robot == null) return;

        switch (mode)
        {
            case ControlMode.InverseKinematics:
                if (ikTarget == null) break;
                // Inside Update() of UR5UserInterface:
                double[] ikResult = robot.SolveIK(
                Matrix4x4.Rotate(ikTarget.rotation), 
                ikTarget.position, 
                activeAngles // Pass the current angles to find the nearest next pose
                );

                //TODO : send ikresult to real robot
                if(ikResult != null) Array.Copy(ikResult, activeAnglesDouble, 6);
                if (ikResult != null) activeAngles = Array.ConvertAll(ikResult, x => (float)x);
                break;

            case ControlMode.ManualJoints:
                activeAngles = new float[] { q1 * Mathf.Deg2Rad, q2 * Mathf.Deg2Rad, q3 * Mathf.Deg2Rad, q4 * Mathf.Deg2Rad, q5 * Mathf.Deg2Rad, q6 * Mathf.Deg2Rad };
                break;

            case ControlMode.MATLABPresets:
                activeAngles = GetPresetAngles(activePreset);
                break;
        }

        robot.ApplyJointAngles(activeAngles);
        DisplayRobotStatus(); // Call the updated display function
    }

    float[] GetPresetAngles(MATLABTest test)
    {
        switch (test)
        {
            case MATLABTest.q_test_0: return new float[] { 0, 0, 0, 0, 0, 0 };
            case MATLABTest.q_test_1: return new float[] { 1.5708f, 1.5708f, 1.5708f, 1.5708f, 1.5708f, 1.5708f }; // 90 deg
            case MATLABTest.q_test_2: return new float[] { 10 * Mathf.Deg2Rad, 20 * Mathf.Deg2Rad, 30 * Mathf.Deg2Rad, 40 * Mathf.Deg2Rad, 50 * Mathf.Deg2Rad, 60 * Mathf.Deg2Rad };
            default: return new float[6];
        }
    }

    void DisplayRobotStatus()
{
    // 1. Get Tip Position
    Transform tip = robot.jointTransforms[5].Find("Tool_Frame") ?? robot.jointTransforms[5];

    // 2. Format the Joint Angles in Radians
    string jointReport = "";
    for (int i = 0; i < 6; i++)
    {
        float rad = activeAngles[i];

        // Wrap radians to [-PI, PI] range for cleaner reading
        while (rad > Mathf.PI) rad -= 2 * Mathf.PI;
        while (rad < -Mathf.PI) rad += 2 * Mathf.PI;

        jointReport += $"<b>q{i + 1}:</b> {rad:F4} ";
    }

    // 3. Print to Console every 30 frames
    if (Time.frameCount % 30 == 0)
    {
        // I increased decimal precision to F4 (4 decimal places) 
        // because radian changes are much smaller than degree changes.
        Debug.Log($"<b>[Tip Pos]</b> X:{tip.position.x:F4} Y:{tip.position.y:F4} Z:{tip.position.z:F4} | <b>[Joints (rad)]</b> {jointReport}");
    }
}
}