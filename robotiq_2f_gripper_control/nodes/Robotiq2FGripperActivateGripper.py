#!/usr/bin/env python
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

if __name__ == "__main__":
    rospy.init_node("robotiq_activate_gripper", anonymous=True)

    pub = rospy.Publisher(
        "Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output,
        queue_size=1,
        latch=True
    )

    rospy.sleep(0.5)

    # Activate gripper
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 1
    pub.publish(command)

    rospy.sleep(1.0)
