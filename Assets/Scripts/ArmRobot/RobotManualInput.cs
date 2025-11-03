using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class RobotManualInput : MonoBehaviour
{
    [Header("Robot Reference")]
    public GameObject robot;

    [Header("Input Actions (One per Joint)")]
    public InputAction[] jointInputs; // Assign in Inspector or via script

    private RobotController robotController;

    private void Awake()
    {
        robotController = robot.GetComponent<RobotController>();

        // Make sure all input actions are enabled
        foreach (var action in jointInputs)
        {
            if (action != null)
                action.Enable();
        }
    }

    private void OnDestroy()
    {
        // Always disable input actions to prevent leaks
        foreach (var action in jointInputs)
        {
            if (action != null)
                action.Disable();
        }
    }

    private void Update()
    {
        if (robotController == null || jointInputs.Length == 0)
            return;

        bool anyInput = false;

        for (int i = 0; i < robotController.joints.Length && i < jointInputs.Length; i++)
        {
            float inputVal = jointInputs[i].ReadValue<float>();

            if (Mathf.Abs(inputVal) > 0.01f)
            {
                RotationDirection direction = GetRotationDirection(inputVal);
                robotController.RotateJoint(i, direction);
                anyInput = true;
                break; // same behavior as your original: only one joint per frame
            }
        }

        if (!anyInput)
        {
            robotController.StopAllJointRotations();
        }
    }

    // HELPERS
    static RotationDirection GetRotationDirection(float inputVal)
    {
        if (inputVal > 0)
            return RotationDirection.Positive;
        else if (inputVal < 0)
            return RotationDirection.Negative;
        else
            return RotationDirection.None;
    }
}
