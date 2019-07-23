#!/usr/bin/env python

# IK and FK Services taken from example provided in
# http://sdk.rethinkrobotics.com/intera/IK_Service_Example and
# https://github.com/RethinkRobotics/intera_sdk/blob/master/intera_examples/scripts/fk_service_client.py
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import intera_interface
from intera_core_msgs.srv import (
    SolvePositionFK,
    SolvePositionFKRequest,
    SolvePositionIK,
    SolvePositionIKRequest,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class fk_service_client():

    def __init__(self):
        try:
            rospy.init_node("fk_service")
        except:
            pass
        self._limb = intera_interface.Limb("right")
        ns = "ExternalTools/right/PositionKinematicsNode/FKService"
        self._fksvc = rospy.ServiceProxy(ns, SolvePositionFK)
        rospy.wait_for_service(ns, 5.0)

    def request(self, joint_angles):
        joints = JointState()
        fkreq = SolvePositionFKRequest()

        if len(joint_angles) is 7:
            joints.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                           'right_j4', 'right_j5', 'right_j6']

            joints.position=[]
            for n in joints.name:
                # joints.position.append(joint_angles[n])
                joints.position = joint_angles

            # Add desired pose for forward kinematics
            fkreq.configuration.append(joints)
            # Request forward kinematics from base to "right_hand" link
            fkreq.tip_names.append('right_hand')

            try:
                resp = self._fksvc(fkreq)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
                return False

            # Check if result valid
            if (resp.isValid[0]):
                return resp
            else:
                rospy.logerr("INVALID JOINTS - No Cartesian Solution Found.")
                return False


def ik_service_client(input_pose, limb = "right", use_advanced_options = False, seed_position=None, nullspace=False):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=input_pose[0],
                    y=input_pose[1],
                    z=input_pose[2],
                ),
                orientation=Quaternion(
                    x=input_pose[3],
                    y=input_pose[4],
                    z=input_pose[5],
                    w=input_pose[6],
                ),
            ),
        ),
    }
    #print "POSES: ", poses
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        # rospy.loginfo("Running Advanced IK Service.")
        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']

        if seed_position is None:
            seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        else:
            seed.position = seed_position
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(nullspace)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [-1.1816591796875, -0.0020947265625, 2.177681640625]#[0.1, -0.3, 0.5]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.4)
    else:
        pass
        # rospy.loginfo("Running Simple IK Service.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints

    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False

    # return True

if __name__ == '__main__':
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('right_arm')
    print group.get_current_joint_values()
    limb = intera_interface.Limb('right')
    print limb.joint_angles()
