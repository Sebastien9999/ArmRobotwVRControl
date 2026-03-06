using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ArmRobot
{
    [Serializable]
    public class JointsMsg : Message
    {
        public const string k_RosMessageName = "unity_robotics_demo_msgs/Joints";
        public override string RosMessageName => k_RosMessageName;

        public double q1;
        public double q2;
        public double q3;
        public double q4;
        public double q5;
        public double q6;

        public JointsMsg()
        {
            this.q1 = 0.0f;
            this.q2 = 0.0f;
            this.q3 = 0.0f;
            this.q4 = 0.0f;
            this.q5 = 0.0f;
            this.q6 = 0.0f;
        }

        public JointsMsg(double[] angles)
        {
            if (angles.Length != 6)
                throw new ArgumentException("Expected an array of 6 joint angles.");

            this.q1 = angles[0];
            this.q2 = angles[1];
            this.q3 = angles[2];
            this.q4 = angles[3];
            this.q5 = angles[4];
            this.q6 = angles[5];
        }
        public JointsMsg(double q1, double q2, double q3, double q4, double q5, double q6)
        {
            this.q1 = q1;
            this.q2 = q2;
            this.q3 = q3;
            this.q4 = q4;
            this.q5 = q5;
            this.q6 = q6;
        }

        public static JointsMsg Deserialize(MessageDeserializer deserializer) => new JointsMsg(deserializer);

        private JointsMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.q1);
            deserializer.Read(out this.q2);
            deserializer.Read(out this.q3);
            deserializer.Read(out this.q4);
            deserializer.Read(out this.q5);
            deserializer.Read(out this.q6);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.q1);
            serializer.Write(this.q2);
            serializer.Write(this.q3);
            serializer.Write(this.q4);
            serializer.Write(this.q5);
            serializer.Write(this.q6);
        }

        public override string ToString()
        {
            return "JointsMsg: " +
            "\nq1: " + q1.ToString() +
            "\nq2: " + q2.ToString() +
            "\nq3: " + q3.ToString() +
            "\nq4: " + q4.ToString() +
            "\nq5: " + q5.ToString() +
            "\nq6: " + q6.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
