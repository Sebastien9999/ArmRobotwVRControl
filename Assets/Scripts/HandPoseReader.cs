using UnityEngine;
using UnityEngine.InputSystem;

public class HandPoseReader : MonoBehaviour
{
    public InputActionProperty rightHandPosition;
    public InputActionProperty rightHandRotation;

    public InputActionProperty testActionValue;

    [Header("IK Target")]
    public Transform ikTarget; // Assign the VRControllerTarget GameObject here in the Inspector

    // --- NEW SECTION FOR COORDINATE SYSTEM CONVERSION ---
    [Header("Control Space Conversion")]
    [Tooltip("The reference transform for the controller. The controller's position/rotation is relative to this object. This acts as your 'conversion matrix'.")]
    public Transform controlFrame;

    [Tooltip("Scales the controller's movement. A value of 2 means a 10cm hand movement becomes a 20cm target movement.")]
    public float positionScale = 1.0f;
    // --- END OF NEW SECTION ---


    void Update()
    {
        // Check if our necessary transforms have been assigned in the Inspector
        if (ikTarget == null || controlFrame == null)
        {
            Debug.LogWarning("IK Target or Control Frame has not been assigned in the Inspector.", this);
            return;
        }

        Vector3 handPos = rightHandPosition.action.ReadValue<Vector3>();
        Quaternion handRot = rightHandRotation.action.ReadValue<Quaternion>();


        // 1. Scale the hand position. This allows you to make larger or smaller movements.
        Vector3 scaledHandPos = handPos * positionScale;

        // 2. Convert the local, scaled hand position to a world space position relative to the controlFrame.
        // This is the core of the "conversion matrix" logic.
        ikTarget.position = controlFrame.TransformPoint(scaledHandPos);

        // 3. Combine the rotation of the controlFrame with the hand's rotation.
        // This ensures that if you rotate the control frame, the target's orientation follows correctly.
        ikTarget.rotation = controlFrame.rotation * handRot;


        // Debugging logs remain helpful
        Debug.Log("Right Hand Position (World): " + ikTarget.position.ToString("F3"));
        Debug.Log("Right Hand Rotation (World): " + ikTarget.rotation.eulerAngles.ToString("F3"));

        float value = testActionValue.action.ReadValue<float>();
        Debug.Log("Test Action Value: " + value);
    }
}