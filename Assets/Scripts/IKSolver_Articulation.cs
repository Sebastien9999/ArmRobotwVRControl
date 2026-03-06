using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKSolver_Articulation : MonoBehaviour
{
    [Header("IK Components")]
    [Tooltip("The final part of the robot arm (the gripper or tool).")]
    public Transform endEffector;
    [Tooltip("The IK will try to reach this Transform's position.")]
    public Transform target;

    [Header("Solver Parameters")]
    [Tooltip("How many times the solver iterates per FixedUpdate. 10-20 is usually good.")]
    [Range(1, 100)]
    public int iterations = 15;

    [Tooltip("The angular speed (degrees/sec) at which the joints will move towards their calculated target. THIS IS KEY for stability.")]
    public float solverSpeed = 100.0f;

    [Tooltip("The distance at which the solver stops trying to reach the target.")]
    [Range(0.0f, 0.1f)]
    public float tolerance = 0.01f;

    [SerializeField] private RobotController robotController;
    private ArticulationBody[] articulationChain;
    private float maxArmLength = 0f;

    void Start()
    {
        InitializeArticulationChain();
    }

    void FixedUpdate()
    {
        if (target == null || articulationChain == null)
            return;

        // --- SAFETY CHECK ---
        // If the target is further than the arm can possibly reach, don't try to solve.
        float targetDistance = Vector3.Distance(articulationChain[0].transform.position, target.position);
        if (targetDistance > maxArmLength)
        {
            // You could also have it just point towards the target here if you want
            return;
        }

        SolveIK();
    }

    private void InitializeArticulationChain()
    {
        int jointCount = robotController.joints.Length;
        articulationChain = new ArticulationBody[jointCount];
        Transform lastJoint = this.transform;

        for (int i = 0; i < jointCount; i++)
        {
            articulationChain[i] = robotController.joints[i].robotPart.GetComponent<ArticulationBody>();
            if (articulationChain[i] == null)
            {
                Debug.LogError($"Joint '{robotController.joints[i].robotPart.name}' is missing an ArticulationBody component.", this);
            }

            // Calculate the total arm length for our safety check
            if(i > 0)
            {
                 maxArmLength += Vector3.Distance(articulationChain[i-1].transform.position, articulationChain[i].transform.position);
            }
        }
        // Add the last segment to the end effector
        maxArmLength += Vector3.Distance(articulationChain[jointCount - 1].transform.position, endEffector.position);
    }

    private void SolveIK()
    {
        if (Vector3.Distance(endEffector.position, target.position) < tolerance)
        {
            return;
        }

        // We run the iterations to find a good *direction* to move the joints,
        // but we will only apply a small, controlled step towards that direction.
        for (int i = 0; i < iterations; i++)
        {
            for (int j = articulationChain.Length - 1; j >= 0; j--)
            {
                ArticulationBody currentJoint = articulationChain[j];
                var drive = currentJoint.xDrive;

                Vector3 rotationAxis = currentJoint.transform.right;
                Vector3 toEndEffector = endEffector.position - currentJoint.transform.position;
                Vector3 toTarget = target.position - currentJoint.transform.position;

                float angleChange = Vector3.SignedAngle(toEndEffector, toTarget, rotationAxis);

                float currentAngle = currentJoint.jointPosition[0] * Mathf.Rad2Deg;
                
                // --- THIS IS THE KEY CHANGE ---
                // Instead of jumping to the final angle, we calculate a goal for this frame.
                // We move the current target angle towards the desired angle at a controlled speed.
                float newTargetAngle = Mathf.MoveTowardsAngle(drive.target, currentAngle + angleChange, solverSpeed * Time.fixedDeltaTime);
                
                // Clamp to the joint's physical limits
                drive.target = Mathf.Clamp(newTargetAngle, drive.lowerLimit, drive.upperLimit);
                currentJoint.xDrive = drive;
            }
        }
    }
}