
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from std_msgs.msg import String

# Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import *
import math
from geometry_msgs.msg import Quaternion

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      #joint_positions[6] = 0.0
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose
  # Original function to reach cartesian pose
  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)
  

    # function definition of MoveIt python tutorial
  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    arm_group = self.arm_group

    # Set the tolerance
    arm_group.set_goal_position_tolerance(0.05)

    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    was = quaternion_from_euler(1.5707, 1.5707, 00)
    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.x = -0.596112750726
    # pose_goal.orientation.y = 0.382257023395
    # pose_goal.orientation.z = 0.589502190021
    # pose_goal.orientation.w = 0.388608188859
    pose_goal.orientation.x = was[0]
    pose_goal.orientation.y = was[1]
    pose_goal.orientation.z = was[2]
    pose_goal.orientation.w = was[3]

    # pose_goal.orientation.w = 1
    pose_goal.position.x = 0
    pose_goal.position.y =0.
    pose_goal.position.z =0.80

 
    arm_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = arm_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    arm_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    arm_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.arm_group.get_current_pose().pose
    #return all_close(pose_goal, current_pose, 0.01)
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

#Define the callback function which will be used later on
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener1', anonymous=True)

    rospy.Subscriber("chatter1", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def main():
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  if success:
    actual_pose = example.get_cartesian_pose()
    #Try to use the MoveIt function to reach a cartesian pose
    rospy.loginfo("Reaching cartesian pose...")
    success &= example.go_to_pose_goal()

    # rospy.loginfo("Reaching Named Target Vertical...")
    
    # success &= example.reach_named_position("vertical")

    # rospy.loginfo("Reaching Joint Angles...")
    
    # success &= example.reach_joint_angles(tolerance=0.01) #rad

    # rospy.loginfo("Reaching Named Target Home...")
    
    # success &= example.reach_named_position("home")

    # rospy.loginfo("Reaching Cartesian Pose...")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z += 0.2
    

    # success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    
    # if example.degrees_of_freedom == 7:
    #   rospy.loginfo("Reach Cartesian Pose with constraints...")
    #   # Get actual pose
    #   actual_pose = example.get_cartesian_pose()
    #   actual_pose.position.y -= 0.3
      
    #   # Orientation constraint (we want the end effector to stay the same orientation)
    #   constraints = moveit_msgs.msg.Constraints()
    #   orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    #   orientation_constraint.orientation = actual_pose.orientation
    #   constraints.orientation_constraints.append(orientation_constraint)

    #   # Send the goal
    #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

    # if example.is_gripper_present:
    #   rospy.loginfo("Opening the gripper...")
    #   success &= example.reach_gripper_position(0)

    #   rospy.loginfo("Closing the gripper 50%...")
    #   success &= example.reach_gripper_position(0.5)

    # # For testing purposes
    # rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    # if not success:
    #     rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
  # listener()
