using UnityEngine;
using UnityEngine.InputSystem;

public class HandPoseReader : MonoBehaviour
{
    public InputActionProperty rightHandPosition;
    public InputActionProperty rightHandRotation;
    
    public InputActionProperty testActionValue;



    void Update()
    {

        Vector3 handPos = rightHandPosition.action.ReadValue<Vector3>();
        Quaternion handRot = rightHandRotation.action.ReadValue<Quaternion>();

        // For Meta Quest using OVRInput fast access (commented out)
        //Vector3 rightPos = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
        //Quaternion rightRot = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);

        Debug.Log("Right Hand Position: " + handPos.ToString("F3"));
        Debug.Log("Right Hand Rotation: " + handRot.eulerAngles.ToString("F3"));

        float value = testActionValue.action.ReadValue<float>();
        Debug.Log("Test Action Value: " + value);
    }
}
