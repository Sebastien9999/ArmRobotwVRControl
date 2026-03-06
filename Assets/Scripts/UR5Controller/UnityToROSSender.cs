using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Requires StdMsgs
using RosMessageTypes.UnityRoboticsDemo;
using RosMessageTypes.ArmRobot; // Requires ArmRobot msgs   

public class UnityToROSSender : MonoBehaviour
{
    [Header("References")]
    public UR5RobotController robotController;
    public UR5UserInterface RobotAngleUI;

    [Header("ROS Configuration")]
    public string posTopicName = "pos_rot";
    public string jointsTopicName = "joints_angles";

    [Header("Publish Options")]
    [Tooltip("Publish the world-space position/rotation of the target object.")]
    public bool publishTargetPose = true;
    [Tooltip("Publish the current joint angles calculated by the UI/controller.")]
    public bool publishJointAngles = true;
    
    private ROSConnection ros;
    public GameObject target;
    private double[] lastSentAngles = new double[6];
    private float publishInterval = 0.2f; 
    private float timeElapsed = 0f;

    void Start()
    {
        // Initialize ROS Connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(posTopicName);
        ros.RegisterPublisher<Float64MultiArrayMsg>(jointsTopicName);
    }

    void Update()
    {

        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishInterval)
        {
            if (publishTargetPose && target != null)
            {
                PosRotMsg targetPos = new PosRotMsg
                (
                    target.transform.position.x,
                    target.transform.position.y,
                    target.transform.position.z,
                    target.transform.rotation.x,
                    target.transform.rotation.y,
                    target.transform.rotation.z,
                    target.transform.rotation.w
                );

                PublishPosToROS(targetPos);
                
            }

            if (publishJointAngles && RobotAngleUI != null)
            {
                // convert UI floats to double if necessary
                double[] inv = {-1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // Identity conversion for now
                double[] rosOrder = new double[6];
                rosOrder[0] = (double)RobotAngleUI.ActiveAngles[0] * inv[0]; // Elbow
                rosOrder[1] = (double)RobotAngleUI.ActiveAngles[1] * inv[1]; // Shoulder Lift
                rosOrder[2] = (double)RobotAngleUI.ActiveAngles[2] * inv[2]; // Shoulder Pan
                rosOrder[3] = (double)RobotAngleUI.ActiveAngles[3] * inv[3]; // Wrist 1
                rosOrder[4] = (double)RobotAngleUI.ActiveAngles[4] * inv[4]; // Wrist 2
                rosOrder[5] = (double)RobotAngleUI.ActiveAngles[5] * inv[5]; // Wrist 3
                Float64MultiArrayMsg msg = new Float64MultiArrayMsg();
                msg.data = rosOrder;

                PublishAnglesToROS(msg);
                
                
            }

            timeElapsed = 0f;
        }
    }

    void PublishPosToROS(PosRotMsg targetPos)
    {
        Debug.Log($"Publishing to ROS: Position({target.transform.position.x}, {target.transform.position.y}, {target.transform.position.z}), Rotation({target.transform.rotation.x}, {target.transform.rotation.y}, {target.transform.rotation.z}, {target.transform.rotation.w})");
        ros.Publish(posTopicName, targetPos);
    }
    void PublishAnglesToROS(Float64MultiArrayMsg jointAngles)
    {
        Debug.Log($"Publishing Joint Angles to ROS: {string.Join(", ", jointAngles.data)}");
        ros.Publish(jointsTopicName, jointAngles);
    }

    bool HasMoved(double[] newAngles)
    {
        for (int i = 0; i < 6; i++)
        {
            if (Mathf.Abs((float)(newAngles[i] - lastSentAngles[i])) > 0.001f) return true;
        }
        return false;
    }
}