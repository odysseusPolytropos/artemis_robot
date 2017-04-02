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
        yaw += 3.14159/2.0
    
    target_pose.pose.position.z = target_pose.pose.position.z+ box_dimensions[2]/2.0 + 0.2
    
    orient = tf.transformations.quaternion_from_euler(0.0, 3.14159/2.0, yaw)
    target_pose.pose.orientation.x = orient[0]
    target_pose.pose.orientation.y = orient[1]
    target_pose.pose.orientation.z = orient[2]
    target_pose.pose.orientation.w = orient[3]
    
    return target_pose

def open_gripper(gripper):
    gripper.set_start_state_to_current_state()
    gripper.set_named_target("open")
    gripper.go(wait=True)
    rospy.sleep(1.0)


def close_gripper(gripper):
    print "==================== closing gripper"
    gripper.set_start_state_to_current_state()
    gripper.set_named_target("closed")
    gripper.set_max_velocity_scaling_factor(0.1)
    gripper.go(wait=False)
        
def home_pos(ur5_arm):
    ur5_arm.set_named_target("up")
    ur5_arm.go()

def callback(data):
    global gripper
    print "STOPPING GRIPPER"
    gripper.stop()

def ar_grasp_test(name):
    global gripper
    
    rospy.Subscriber("/gripper_feedback", GraspedObject, callback)

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
    
    open_gripper(gripper)
    home_pos(ur5_arm)
    
    
    object_info = get_object_info(name)
    pprint(object_info)

    
    # Get the name of the end-effector link
    end_effector_link = ur5_arm.get_end_effector_link()

    # Set the reference frame for pose targets
    reference_frame = "/world"
    # Set the arm reference frame accordingly
    ur5_arm.set_pose_reference_frame(reference_frame)
    # Allow replanning to increase the odds of a solution
    ur5_arm.allow_replanning(True)
    # Allow some leeway in position (meters) and orientation (radians)
    ur5_arm.set_goal_position_tolerance(0.01)
    ur5_arm.set_goal_orientation_tolerance(0.05)
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
    target_pose.pose.position.z = target_pose.pose.position.z-0.05
    
    move_cartesian(ur5_arm, [target_pose.pose])
    
    close_gripper(gripper)
    rospy.wait_for_message("/gripper_feedback", GraspedObject, timeout=10)
    print "STOPPING GRIPPER"
    gripper.stop()
    
    
    rospy.sleep(2)
    
    target_pose.pose.position.z = target_pose.pose.position.z+0.10
    ur5_arm.set_pose_target(target_pose, end_effector_link)

    ur5_arm.set_max_velocity_scaling_factor(0.1)
    traj = ur5_arm.plan()

    ur5_arm.execute(traj)
    
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

    return
    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    gripper.set_start_state_to_current_state()
    
    
    gripper_end_effector_link = gripper.get_end_effector_link()
    gripper.set_pose_target(target_pose, gripper_end_effector_link)
    traj = gripper.plan()
    
    gripper.execute(traj)
    gripper.go()
    rospy.sleep(1)
    
    
    print "============ STOPPING"

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    rospy.sleep(2)
    scene.remove_world_object("floor")
    

    
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % gripper.get_planning_frame()

    ## We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % gripper.get_end_effector_link()

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    gripper.set_planner_id("RRTConnectkConfigDefault")


    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector
    print "============ Generating plan 1"
    x = 0
    while x < 100 and not rospy.is_shutdown():
        gripper.set_random_target()

        ## Now, we call the planner to compute the plan
        ## and visualize it if successful
        ## Note that we are just planning, not asking move_group 
        ## to actually move the robot
        plan1 = gripper.plan()
        gripper.go(wait=True)
        x+=1


    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)


    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again).
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);

    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(5)


    ## Moving to a pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^
    ##
    ## Moving to a pose goal is similar to the step above
    ## except we now use the go() function. Note that
    ## the pose goal we had set earlier is still active 
    ## and so the robot will try to move to that goal. We will
    ## not use that function in this tutorial since it is 
    ## a blocking function and requires a controller to be active
    ## and report success on execution of a trajectory.

    # Uncomment below line when working with a real robot
    gripper.go(wait=True)



    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL

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

