import sys

import actionlib
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from moveit_commander.conversions import pose_to_list
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm_left_torso"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        gripper_group = moveit_commander.MoveGroupCommander("gripper_left")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        # box_name = self.box_name
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "map"

        box_pose.pose.position.x = 4.0
        box_pose.pose.position.y = -2.7
        box_pose.pose.position.z = 0.8393

        box_pose.pose.orientation.x = 0
        box_pose.pose.orientation.y = 0
        box_pose.pose.orientation.z = 0
        box_pose.pose.orientation.w = 1.0

        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))

        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    # 4.2 -3 0 0 0 1.57
    def add_table(self, timeout=4):
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "map"

        box_pose.pose.position.x = 4.2
        box_pose.pose.position.y = -3
        box_pose.pose.position.z = 0.4

        q = quaternion_from_euler(0, 0, 1.57)
        box_pose.pose.orientation.x = q[0]
        box_pose.pose.orientation.y = q[1]
        box_pose.pose.orientation.z = q[2]
        box_pose.pose.orientation.w = q[3]

        box_name = "table"
        scene.add_box(box_name, box_pose, size=(1 + 0.1, 0.8 + 0.05, 0.83))

        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def go_to_prepick_pos(self):
        box_x = 4
        box_y = -2.7 + 0.035
        box_z = 0.84

        move_group = self.move_group
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "map"
        pose_goal.pose.position.x = box_x - 0.2
        pose_goal.pose.position.y = box_y
        pose_goal.pose.position.z = box_z + 0.2

        q = quaternion_from_euler(1.57, 0, 0)
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]

        move_group.set_pose_target(pose_goal)

        move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        # current_pose = self.move_group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)

    def go_to_pick_pos(self):
        box_x = 4
        box_y = -2.7 + 0.035
        box_z = 0.84

        move_group = self.move_group
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = "map"
        pose_goal.pose.position.x = box_x - 0.2
        pose_goal.pose.position.y = box_y
        pose_goal.pose.position.z = box_z + 0.05

        q = quaternion_from_euler(1.57, 0, 0)
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]

        move_group.set_pose_target(pose_goal)

        move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

    def close_gripper(self):
        assert self is not None
        rospy.wait_for_service('/parallel_gripper_left_controller/grasp')
        try:
            grasp = rospy.ServiceProxy('/parallel_gripper_left_controller/grasp', Empty)
            resp1 = grasp()
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def open_gripper(self):
        assert self is not None
        rospy.wait_for_service('/parallel_gripper_left_controller/release')
        try:
            release = rospy.ServiceProxy('/parallel_gripper_left_controller/release', Empty)
            resp1 = release()
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def go_to_base_position(self):
        assert self is not None

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 3.2
        goal.target_pose.pose.position.y = -3.0153
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1

        client.send_goal(goal)
        client.wait_for_result()


def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()
        tutorial.add_box()
        tutorial.add_table()

        tutorial.go_to_base_position()
        tutorial.go_to_prepick_pos()
        tutorial.go_to_pick_pos()
        tutorial.close_gripper()

        tutorial.go_to_prepick_pos()
        tutorial.open_gripper()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
