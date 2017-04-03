#!/usr/bin/env python

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from object_msgs.msg import Object
from object_msgs.srv import ObjectInfo
from grasp_msgs.msg import GraspedObject
from pprint import pprint
from math import pi

### CONST ###
# Distance obove the object for pregrasp posture
PRE_GRASP_Z_DIST = 0.2 # [m]
# Distance to drive in z-axis in cartesian space for grasp
GRASP_Z_DELTA = -0.075 # [m]
# Distance to drive in z-axis after grasp
DESTINATION_Z = 0.10 # [m]
# between 0 and 1, a too fast gripper results in numerical problems
GRIPPER_VELOCITY_SCALING_FACTOR = 0.04 # [-]

def get_object_info(name):
    rospy.wait_for_service('/gazebo_objects/get_info')
    try:
        object_info = rospy.ServiceProxy('/gazebo_objects/get_info', ObjectInfo)
        resp = object_info(name, True) # [name, return_geometry]
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        sys.exit(1)

def move_cartesian(group, waypoints):
    fraction = 0.0
    maxtries = 100
    attempts = 0

    # Set the internal state to the current state
    group.set_start_state_to_current_state()

    # Plan the Cartesian path connecting the waypoints
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = group.compute_cartesian_path (
                                 waypoints,   # waypoint poses
                                 0.01,        # eef_step
                                 0.0,         # jump_threshold
                                 True)        # avoid_collisions

        # Increment the number of attempts
        attempts += 1

         # Print out a progress message
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

     # If we have a complete plan, execute the trajectory
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")

        group.execute(plan)

        rospy.loginfo("Path execution complete.")
    else:
        rospy.loginfo("Path planning failed with only " +
    str(fraction) + " success after " + str(maxtries) + " attempts.")

def pose2rpy(pose):
    quaternions = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    
    euler = tf.transformations.euler_from_quaternion(quaternions)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll, pitch, yaw

def calc_pregrasp_pose(box_pose, box_dimensions):
    target_pose = geometry_msgs.msg.PoseStamped()
    target_pose.header.frame_id = "/world"
    target_pose.header.stamp = rospy.Time.now()
    
    target_pose.pose = box_pose
    
    roll, pitch, yaw = pose2rpy(box_pose)
    
    if box_dimensions[0] > box_dimensions[1]:
        yaw += pi/2.0
    
    target_pose.pose.position.z = target_pose.pose.position.z + box_dimensions[2]/2.0 + PRE_GRASP_Z_DIST
    print target_pose.pose.position.z
    
    orient = tf.transformations.quaternion_from_euler(0.0, pi/2.0, yaw)
    target_pose.pose.orientation.x = orient[0]
    target_pose.pose.orientation.y = orient[1]
    target_pose.pose.orientation.z = orient[2]
    target_pose.pose.orientation.w = orient[3]
    
    return target_pose

def open_gripper():
    global gripper
    print "==================== opening gripper"
    gripper.set_start_state_to_current_state()
    gripper.set_named_target("open")
    gripper.go(wait=True)
    rospy.sleep(1.0)


def close_gripper():
    global gripper
    print "==================== closing gripper"
    gripper.set_start_state_to_current_state()
    gripper.set_named_target("closed")
    gripper.set_max_velocity_scaling_factor(GRIPPER_VELOCITY_SCALING_FACTOR)
    gripper.go(wait=False)
        
def home_pos():
    global ur5_arm
    print "==================== driving to home position"
    ur5_arm.set_named_target("up")
    ur5_arm.go()

def callback(data):
    global gripper
    print "STOPPING GRIPPER"
    gripper.stop()

def init_robot():
    global gripper, ur5_arm, robot, scene

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()
    
    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    gripper = moveit_commander.MoveGroupCommander("endeffector")
    gripper.set_planner_id("RRTConnectkConfigDefault")
    
    # Initialize the move group for the arm
    ur5_arm = moveit_commander.MoveGroupCommander("manipulator")  
    ur5_arm.set_planner_id("RRTConnectkConfigDefault")
    
    # Set the reference frame for pose targets
    reference_frame = "/world"
    # Set the arm reference frame accordingly
    ur5_arm.set_pose_reference_frame(reference_frame)
    # Allow replanning to increase the odds of a solution
    ur5_arm.allow_replanning(True)
    # Allow some leeway in position (meters) and orientation (radians)
    ur5_arm.set_goal_position_tolerance(0.02)
    ur5_arm.set_goal_orientation_tolerance(0.1)
    
    ur5_arm.set_planning_time(5)
    

def ar_grasp_test(name):
    global gripper, ur5_arm, robot, scene
    
    
    init_robot()

    rospy.Subscriber("/gripper_feedback", GraspedObject, callback)

    open_gripper()
    home_pos()
       
    object_info = get_object_info(name)
    
    # Get the name of the end-effector link
    end_effector_link = ur5_arm.get_end_effector_link()


    #Move the end effecor to the x , y, z positon
    #Set the target pose in the base_link frame    
    box_pose = object_info.object.primitive_poses[0]
    box_dimensions = object_info.object.primitives[0].dimensions
    target_pose = calc_pregrasp_pose(box_pose, box_dimensions)
        
    # Set the start state to the current state
    ur5_arm.set_start_state_to_current_state()
    # Set the goal pose of the end effector to the stored pose
    ur5_arm.set_pose_target(target_pose, end_effector_link)
    # Plan the trajectory to the goal
    traj = ur5_arm.plan()
    # Execute the planned trajectory
    ur5_arm.execute(traj)
    # Pause for a second
    rospy.sleep(1)

    target_pose.pose.position.z = target_pose.pose.position.z + GRASP_Z_DELTA
    
    move_cartesian(ur5_arm, [target_pose.pose])
    
    close_gripper()
    try:
        rospy.wait_for_message("/gripper_feedback", GraspedObject, timeout=10)
        gripper.stop()
    except:
        pass
    
    rospy.sleep(2)
    
    # drive 10 cm up
    target_pose.pose.position.z = target_pose.pose.position.z + DESTINATION_Z
    ur5_arm.set_pose_target(target_pose, end_effector_link)

    ur5_arm.set_max_velocity_scaling_factor(0.1)
    traj = ur5_arm.plan()

    ur5_arm.execute(traj)
    
    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)        

    print "============ STOPPING"

def usage():
    return "USAGE: %s [name]"%sys.argv[0]

if __name__=='__main__':
    
    if len(sys.argv) == 2:
        name = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    
    try:
        ## First initialize moveit_commander and rospy.
        print "============ Starting tutorial setup"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ar_grasp_test',
                      anonymous=True)
        ar_grasp_test(name)
    except rospy.ROSInterruptException:
        pass

