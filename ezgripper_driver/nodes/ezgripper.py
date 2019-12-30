#!/usr/bin/python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, SAKE Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##

#
#  If you want to modify this program, you can find the "Main program" and
#  "Main loop" by searching for these terms.  They exist near the end of this file.
#

import rospy
from std_srvs.srv import Empty, EmptyResponse
from libezgripper import create_connection, Gripper
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState

DIAG_UPDATE_INTERVAL_S = 1.0
STATUS_UPDATE_INTERVAL_S = 0.2


class GripperActionServer(object):
    def __init__(self, action_name, gripper):
        self.gripper = gripper
        self.action_server = actionlib.SimpleActionServer(
            action_name,
            GripperCommandAction,
            self.gripper_action_execute,
            False,
        )
        self.action_server.start()

    def gripper_action_execute(self, goal):
        rospy.loginfo(
            "Execute goal: position=%.1f, max_effort=%.1f"
            % (goal.command.position, goal.command.max_effort)
        )

        if goal.command.max_effort == 0.0:
            rospy.loginfo("Release torque: start")
            self.gripper.release()
            rospy.loginfo("Release torque: done")
        else:
            rospy.loginfo("Go to position: start")
            self.gripper.goto_position(
                goal.command.position, goal.command.max_effort
            )
            rospy.loginfo("Go to position: done")

        result = GripperCommandResult()
        # not necessarily the current position of the gripper if the gripper did not reach its goal position.
        result.position = goal.command.position
        result.effort = goal.command.max_effort
        result.stalled = False
        result.reached_goal = True
        self.action_server.set_succeeded(result)


class GripperDiagnostics(object):
    def __init__(self, grippers):
        self._grippers = grippers

        self._pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=1
        )

    def send_diags(self):
        # See diagnostics with: rosrun rqt_runtime_monitor rqt_runtime_monitor
        msg = DiagnosticArray()
        msg.status = []
        msg.header.stamp = rospy.Time.now()

        for gripper in (g.gripper for g in self._grippers):
            for servo in gripper.servos:
                status = DiagnosticStatus()
                status.name = "Gripper '%s' servo %d" % (
                    gripper.name,
                    servo.servo_id,
                )
                status.hardware_id = '%s' % servo.servo_id
                temperature = servo.read_temperature()
                status.values.append(KeyValue('Temperature', str(temperature)))
                status.values.append(
                    KeyValue('Voltage', str(servo.read_voltage()))
                )

                if temperature >= 70:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'OVERHEATING'
                elif temperature >= 65:
                    status.level = DiagnosticStatus.WARN
                    status.message = 'HOT'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = 'OK'

                msg.status.append(status)

        self._pub.publish(msg)


class GripperStatus(object):
    PALM_L1_CLOSED_POS = 1.84
    L1_L2_CLOSED_POS = 0.0

    def __init__(self, grippers):
        self._grippers = grippers

        self._pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    def publish_status(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for gripper in (g.gripper for g in self._grippers):
            # gripper.name
            pos = gripper.get_position()
            joint_pos = (100.0 - pos) / 100.0 * self.PALM_L1_CLOSED_POS
            msg.name.append(
                '{}_ezgripper_knuckle_palm_L1_1'.format(gripper.name)
            )
            msg.position.append(joint_pos)
            joint_pos = (100.0 - pos) / 100.0 * self.L1_L2_CLOSED_POS
            msg.name.append(
                'main_ezgripper_knuckle_L1_L2_1'.format(gripper.name)
            )
            msg.position.append(joint_pos)

        self._pub.publish(msg)


class EZGripper(object):
    def __init__(self, connection, gripper_name, servo_ids):
        self.gripper = Gripper(connection, gripper_name, servo_ids)

        self._action_srv = GripperActionServer('~' + gripper_name, self.gripper)
        self._calibrate_srv = rospy.Service(
            '~' + gripper_name + '/calibrate', Empty, self._calibrate_srv
        )

        self.gripper.calibrate()
        self.gripper.open()

    def _calibrate_srv(self, _msg):
        rospy.loginfo("Calibrate service: request received")
        self.gripper.calibrate()
        self.gripper.open()
        rospy.loginfo("Calibrate service: request completed")
        return EmptyResponse()


def main():
    # Main Program

    rospy.init_node('ezgripper')
    rospy.loginfo("Started")

    port_name = rospy.get_param('~port', '/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud', '57600'))
    gripper_params = rospy.get_param('~grippers')

    connection = create_connection(port_name, baud)

    all_servos = []
    grippers = []

    for gripper_name, servo_ids in gripper_params.items():
        gripper = EZGripper(connection, gripper_name, servo_ids)
        all_servos += gripper.gripper.servos
        grippers.append(gripper)

    diagnostics = GripperDiagnostics(grippers)
    status = GripperStatus(grippers)

    # Main Loop
    r = rospy.Rate(20)  # hz
    diags_last_sent = 0
    status_last_sent = 0
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if now - diags_last_sent > DIAG_UPDATE_INTERVAL_S:
            try:
                diagnostics.send_diags()
                diags_last_sent = now
            except Exception as e:
                rospy.logerr("Exception while reading diagnostics: %s" % e)

        if now - status_last_sent > STATUS_UPDATE_INTERVAL_S:
            try:
                status.publish_status()
                status_last_sent = now
            except Exception as e:
                rospy.logerr("Exception while publishing status %s" % e)

        for servo in all_servos:
            try:
                servo.check_overload_and_recover()
            except Exception as e:
                rospy.logerr("Exception while checking overload: %s" % e)
                servo.flushAll()

        r.sleep()

    rospy.loginfo("Exiting")


if __name__ == '__main__':
    main()
