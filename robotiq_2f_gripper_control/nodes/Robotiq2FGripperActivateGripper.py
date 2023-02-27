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

    rospy.sleep(1.0)

    print("Resetting and activating gripper")

    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 0
    pub.publish(command)
    rospy.sleep(0.1)

    # Activate gripper
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = 0
    command.rSP = 255
    command.rFR = 150
    pub.publish(command)

    rospy.sleep(1.0)
