using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[System.Serializable]
public struct JointConstraint
{
    public Transform joint;
    [Tooltip("The local axis of rotation for this hinge joint (e.g., Vector3.right for an elbow).")]
    public Vector3 rotationAxis;
    public float minAngle;
    public float maxAngle;
}


public class IKSolver : MonoBehaviour
{
    [Header("IK Chain")]
    public Transform[] joints;
    public Transform endEffector;

    [Header("IK Target")]
    public Transform target;

    [Header("Solver Parameters")]
    [Range(0, 100)]
    public int iterations = 10;
    [Range(0.0f, 0.1f)]
    public float tolerance = 0.01f;

    // Array for defining constraints in the Inspector
    [Header("Joint Constraints")]
    public JointConstraint[] constraints;

    // dictionary for fast lookups
    private Dictionary<Transform, JointConstraint> constraintDict;

    // Awake method to initialize the dictionary
    private void Awake()
    {
        // Convert the array to a dictionary for efficient access
        constraintDict = constraints.ToDictionary(c => c.joint, c => c);
    }

    private void LateUpdate()
    {
        if (target == null)
            return;

        SolveIK();
    }

    private void SolveIK()
    {
        if (joints == null || joints.Length == 0)
            return;

        for (int i = 0; i < iterations; i++)
        {
            if (Vector3.Distance(endEffector.position, target.position) < tolerance)
                return;

            for (int j = joints.Length - 1; j >= 0; j--)
            {
                // Store the current joint
                Transform currentJoint = joints[j];

                Vector3 toEndEffector = (endEffector.position - currentJoint.position).normalized;
                Vector3 toTarget = (target.position - currentJoint.position).normalized;

                Quaternion rotation = Quaternion.FromToRotation(toEndEffector, toTarget);

                // Apply rotation and then enforce constraints 
                currentJoint.rotation = rotation * currentJoint.rotation;

                // Check if a constraint exists for this joint and apply it
                if (constraintDict.ContainsKey(currentJoint))
                {
                    EnforceConstraint(currentJoint);
                }
            }
        }
    }

    // Method to apply the joint constraints 
    private void EnforceConstraint(Transform joint)
    {
        JointConstraint constraint = constraintDict[joint];

        // Get the rotation relative to the parent
        Quaternion currentLocalRotation = joint.localRotation;
        
        // Convert the quaternion to an angle-axis representation
        currentLocalRotation.ToAngleAxis(out float angle, out Vector3 axis);

        // Normalize the angle to be within -180 and 180 degrees
        if (angle > 180f)
        {
            angle -= 360f;
        }

        // Project the current rotation axis onto the allowed constraint axis
        // This effectively isolates the rotation we want to limit
        float angleOnConstraintAxis = Vector3.Dot(axis, constraint.rotationAxis.normalized) * angle;

        // Clamp the angle to the defined min/max limits
        float clampedAngle = Mathf.Clamp(angleOnConstraintAxis, constraint.minAngle, constraint.maxAngle);

        // Create a new, clamped rotation around the allowed axis
        Quaternion constrainedLocalRotation = Quaternion.AngleAxis(clampedAngle, constraint.rotationAxis);

        // Apply the constrained rotation
        joint.localRotation = constrainedLocalRotation;
    }
}