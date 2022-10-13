#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2022, University of Oxford
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Oxford nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2022, University of Oxford
# Revision $Id$

"""@package docstring
Command-line interface for receiving the status of a 2F gripper and publishing a corresponding joint state message.
"""

import rospy
from time import sleep
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg


class Robotiq2FGripperStatusListener(object):
    def __init__(self, prefix=""):
        self._prefix = prefix
        self._sub = rospy.Subscriber(
            "Robotiq2FGripperRobotInput",
            inputMsg.Robotiq2FGripper_robot_input,
            self.callback,
            queue_size=10,
        )
        self._pub = rospy.Publisher("joint_states", JointState, queue_size=10)

    def callback(self, msg):
        js_msg = JointState()

        # The original message has no time, so timestamp it here on translation
        js_msg.header.stamp = rospy.Time.now()

        js_msg.name = [self._prefix + "finger_joint"]
        js_msg.velocity = [0.0]  # We do not have this information
        js_msg.effort = [msg.gCU * 10]  # mA
        js_msg.position = [(msg.gPO / 255) * 0.7]

        self._pub.publish(js_msg)


if __name__ == "__main__":
    rospy.init_node("Robotiq2FGripperJointStatePublisher")
    sleep(0.5)

    obj = Robotiq2FGripperStatusListener()
    rospy.spin()
