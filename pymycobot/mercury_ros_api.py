#!/usr/bin/env python
# coding=UTF-8
import time
import threading
import time
import sys
import subprocess
import os
import glob

try:
    with open('/proc/device-tree/model', 'r') as model_file:
        model = model_file.read().strip().lower()  # nvidia jetson xavier nx developer kit
    if "xavier" in model:
        import rospy
        import actionlib
        import tf2_ros
        import tf_conversions
        from std_msgs.msg import Float32
        from actionlib_msgs.msg import *
        from actionlib_msgs.msg import GoalID
        from actionlib_msgs.msg import GoalStatusArray
        from tf.transformations import quaternion_from_euler
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from geometry_msgs.msg import Point
        from geometry_msgs.msg import Point
        from geometry_msgs.msg import Twist
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from geometry_msgs.msg import PoseStamped, PoseArray
except Exception as e:
    pass

# MapNavigation


class MapNavigation:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('map_navigation', anonymous=False)

        # ros publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_setpose = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_cancel = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=10)
        self.pub_tempPose = rospy.Publisher(
            'move_base_simple/goal_temp', PoseStamped, queue_size=1)
        self.goal_pub = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=1)

        # ros subscriber
        # self.voltage_subscriber = rospy.Subscriber("/PowerVoltage", Float32, self.voltage_callback) #Create a battery-voltage topic subscriber
        self.initialpose_subscriber = rospy.Subscriber(
            '/initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)
        self.goal_subscriber = rospy.Subscriber(
            'move_base/status', GoalStatusArray, self.goalCntCallback)

        self.RobotVersion = 1.0
        self.SystemVersion = 1.0
        self.is_running = False
        self.thread = None
        self.LastError = None
        self.directory = "~/mercury_x1_ros/src/turn_on_mercury_robot/map"

        self.pose = PoseStamped()
        self.pose_array = PoseArray()
        self.pose_stamped = PoseStamped()

        # getPosition
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.point_count = 0
        self.flag_HomePosition = True
        self.flag_Parking = True
        self.flag_Charge = True
        self.flag_Load = True
        self.flag_Unload = True

    def getRobotVersion(self):
        return self.RobotVersion

    def getSystemVersion(self):
        return self.SystemVersion

    # voltage Callback
    def voltage_callback(self, msg):
        return msg.data

    def initial_pose_callback(self, msg):
        # rospy.loginfo("Received Initial Pose:\n%s", msg)
        return msg

    # 回调函数订阅话题move_base_simple/goal_temp，填入PoseArray
    def goalCntCallback(self, goal_msg):

        self.pose_stamped.header = goal_msg.header
        self.pose_stamped.pose = goal_msg.pose

        # self.pose_array.header.append(pose_stamped.pose)
        self.pose_array.poses.append(self.pose_stamped.pose)

    def setpoint(self, identifier, xGoal, yGoal, orientation_z, orientation_w):
        self.pose = PoseStamped()
        self.pose.header.seq = self.point_count
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = 'map'
        self.pose.pose.position.x = xGoal
        self.pose.pose.position.y = yGoal
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = orientation_z
        self.pose.pose.orientation.w = orientation_w
        # 发布新的点位
        rospy.sleep(1)
        self.pub_tempPose.publish(self.pose)
        self.point_count += 1
        rospy.loginfo(' {} 对应 {} 号点位 pose: {}'.format(
            identifier, self.point_count, self.pose))
        return self.point_count

    # 重置点位
    def clearPosition(self):
        self.pose_array.poses = []
        self.point_count = 0
        self.flag_HomePosition = True
        self.flag_Parking = True
        self.flag_Charge = True
        self.flag_Load = True
        self.flag_Unload = True
        rospy.loginfo('clearPosition')

    def getPose(self, identifier, point_index):
        if point_index > 0:
            if point_index <= len(self.pose_array.poses):
                curGoalIdx_ = point_index % len(self.pose_array.poses)
                pose = self.pose_array.poses[curGoalIdx_-1]
                rospy.loginfo('{} Retrieving {} point pose: {}' .format(
                    identifier, point_index, pose))

    def goTopoint(self, identifier, point_index):
        if point_index > 0:
            if point_index <= len(self.pose_array.poses):
                curGoalIdx_ = point_index % len(self.pose_array.poses)
                goal = PoseStamped()
                goal.header = self.pose_array.header
                goal.pose = self.pose_array.poses[curGoalIdx_-1]
                self.goal_pub.publish(goal)
                rospy.loginfo('Going to {} point:{}'.format(
                    identifier, goal.pose))

    def setHomePosition(self, xGoal, yGoal, orientation_z, orientation_w):
        global point_HomePositionCount
        if self.flag_HomePosition == True:
            self.flag_HomePosition = False
            # Returns the navigation point count
            point_HomePositionCount = self.setpoint(
                "HomePosition", xGoal, yGoal, orientation_z, orientation_w)
        else:
            print(
                "The navigation point has been set. clearPosition() is required to change the navigation.")

    def setParking(self, xGoal, yGoal, orientation_z, orientation_w):
        global point_ParkingCount
        if self.flag_Parking == True:
            self.flag_Parking = False
            # Returns the navigation point count
            point_ParkingCount = self.setpoint(
                "Parking", xGoal, yGoal, orientation_z, orientation_w)
        else:
            print(
                "The navigation point has been set. clearPosition() is required to change the navigation.")

    def setCharge(self, xGoal, yGoal, orientation_z, orientation_w):
        global point_ChargeCount
        if self.flag_Charge == True:
            self.flag_Charge = False
            # Returns the navigation point count
            point_ChargeCount = self.setpoint(
                "Charge", xGoal, yGoal, orientation_z, orientation_w)
        else:
            print(
                "The navigation point has been set. clearPosition() is required to change the navigation.")

    def setLoad(self, xGoal, yGoal, orientation_z, orientation_w):
        global point_LoadCount
        if self.flag_Load == True:
            self.flag_Load = False
            # Returns the navigation point count
            point_LoadCount = self.setpoint(
                "Load", xGoal, yGoal, orientation_z, orientation_w)
        else:
            print(
                "The navigation point has been set. clearPosition() is required to change the navigation.")

    def setUnload(self, xGoal, yGoal, orientation_z, orientation_w):
        global point_UnloadCount
        if self.flag_Unload == True:
            self.flag_Unload = False
            # Returns the navigation point count
            point_UnloadCount = self.setpoint(
                "Unload", xGoal, yGoal, orientation_z, orientation_w)
        else:
            print(
                "The navigation point has been set. clearPosition() is required to change the navigation.")

    # HomePosition
    def getHomePosition(self):
        self.getPose("HomePosition", point_HomePositionCount)

    def goToHomePosition(self):
        self.goTopoint("HomePosition", point_HomePositionCount)

    # Parking
    def getParking(self):
        self.getPose("Parking", point_ParkingCount)

    def goToParking(self):
        self.goTopoint("Parking", point_ParkingCount)

    # Charge
    def getCharge(self):
        self.getPose("Charge", point_ChargeCount)

    def goToCharge(self):
        self.goTopoint("Charge", point_ChargeCount)

    # Load
    def getLoad(self):
        self.getPose("Load", point_LoadCount)

    def goToLoad(self):
        self.goTopoint("Load", point_LoadCount)

    # Unload
    def getUnload(self):
        self.getPose("Unload", point_UnloadCount)

    def goToUnload(self):
        self.goTopoint("Unload", point_UnloadCount)

    # init robot  pose AMCL
    def set_pose(self, xGoal, yGoal, orientation_z, orientation_w, covariance):
        pose = PoseWithCovarianceStamped()
        pose.header.seq = 0
        pose.header.stamp.secs = 0
        pose.header.stamp.nsecs = 0
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = xGoal
        pose.pose.pose.position.y = yGoal
        pose.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, 1.57)
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = orientation_z
        pose.pose.pose.orientation.w = orientation_w
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, covariance]
        rospy.sleep(1)
        self.pub_setpose.publish(pose)
        rospy.loginfo('Published robot pose: %s' % pose)

    # move_base
    def moveToGoal(self, xGoal, yGoal, orientation_z, orientation_w):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        while (not ac.wait_for_server(rospy.Duration.from_sec(5.0))):

            sys.exit(0)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = orientation_z
        goal.target_pose.pose.orientation.w = orientation_w

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if (ac.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def shutdown(self):
        rospy.loginfo("Quit program")
        rospy.sleep()

    # speed command
    def pub_vel(self, x, y, theta):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = theta
        self.pub.publish(twist)

    def vel_control(self, direction=[0, 0, 0], speed=0.0, control_time=0.0):
        """
        Function to control velocity.

        Parameters:
            - direction (list): An array containing three elements representing the direction of motion.
            - speed (float): Speed value.
            - time (float): Time value.

        Raises:
            - ValueError: If the direction is not a list containing three elements.

        Usage:
            >>> vel_control([1, 0, 0], 0.2, 5)
        """
        duration, vel = control_time, speed
        start_time = time.time()
        # Motion control
        if isinstance(direction, list) and len(direction) == 3:
            while (time.time() - start_time) < duration:
                self.pub_vel(direction[0] * abs(vel), direction[1]
                             * abs(vel), direction[2] * abs(vel))
            self.pub_vel(0, 0, 0)
        else:
            print("Direction should be a list containing three elements")

    def turnRight(self, speed, control_time):
        self.vel_control([0, 0, -1], speed, control_time)

    def turnLeft(self, speed, control_time):
        self.vel_control([0, 0, 1], speed, control_time)

    def goStraight(self, speed, control_time):
        self.vel_control([1, 0,  0], speed, control_time)

    def goBack(self, speed, control_time):
        self.vel_control([-1, 0, 0], speed, control_time)

    def stop(self):
        self.pub_vel(0, 0, 0)

    def startMapping(self):
        try:
            launch_command = "roslaunch turn_on_tringai_robot mapping.launch"
            subprocess.run(
                ['gnome-terminal', '-e', f"bash -c '{launch_command}; exec $SHELL'"])
        except subprocess.CalledProcessError as e:
            self.LastError = sys.exc_info()
            print(e)

    def stopMapping(self):
        try:
            # Kill the corresponding process
            close_command = "ps -ef | grep -E " + "mapping.launch" + \
                " | grep -v 'grep' | awk '{print $2}' | xargs kill -2"
            subprocess.run(close_command, shell=True)
        except subprocess.CalledProcessError as e:
            self.LastError = sys.exc_info()
            print(e)

    def deleteMap(self):
        files_to_delete = glob.glob(os.path.join(self.directory, "map.pgm")) + \
            glob.glob(os.path.join(self.directory, "map.yaml"))
        try:
            for file_to_delete in files_to_delete:
                os.remove(file_to_delete)
                print(f"Deleted file: {file_to_delete}")
        except Exception as e:
            self.LastError = sys.exc_info()
            print(e)

    def agvOn(self):
        try:
            # Start lidar and odometer communication
            launch_command = "roslaunch turn_on_mercury_robot turn_on_mercury_robot.launch"
            subprocess.run(
                ['gnome-terminal', '-e', f"bash -c '{launch_command}; exec $SHELL'"])
        except Exception as e:
            self.LastError = sys.exc_info()
            print(e)

    def agvOff(self):
        try:
            # Kill the corresponding process
            close_command = "ps -ef | grep -E " + "turn_on_mercury_robot.launch" + \
                " | grep -v 'grep' | awk '{print $2}' | xargs kill -2"
            subprocess.run(close_command, shell=True)
        except Exception as e:
            self.LastError = sys.exc_info()
            print(e)

    def isAgvOn(self):
        try:
            # Check whether there is a corresponding process
            process_check_command = "ps -ef | grep -E 'turn_on_tringai_robot.launch' | grep -v 'grep'"
            result = subprocess.run(
                process_check_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if len(result.stdout) > 0:
                return True
            else:
                return False
        except Exception as e:
            self.LastError = sys.exc_info()
            print(e)

    def batteryState(self):
        voltage_data = self.voltage_callback(None)
        return voltage_data

    def initPosition(self, x_goal, y_goal, orientation_z, orientation_w, covariance):
        try:
            self.set_pose(x_goal, y_goal, orientation_z,
                          orientation_w, covariance)
        except Exception as e:
            self.LastError = sys.exc_info()

    def getPosition(self):
        # Position = self.initial_pose_callback(None)
        # return Position
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_up", rospy.Time(0), rospy.Duration(1.0))
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            x = translation.x
            y = translation.y

            euler = tf_conversions.transformations.euler_from_quaternion(
                [rotation.x, rotation.y, rotation.z, rotation.w])

            tw = euler[2]  # Yaw

            rospy.loginfo("X position{}".format(x))
            rospy.loginfo("Y position{}".format(y))
            rospy.loginfo("TW orientation{}".format(tw))

            return x, y, tw

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF transformation query failed:{}".format(e))

    def goToPosition(self, goal_x, goal_y, orientation_z, orientation_w):
        flag_feed_goalReached = self.moveToGoal(
            goal_x, goal_y, orientation_z, orientation_w)
        return flag_feed_goalReached

    def pause(self):
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(
                target=self.stop_navigation, daemon=True)
            self.thread.start()

    def stop_navigation(self):
        while self.is_running:
            goal_id = GoalID()
            self.pub_cancel.publish(goal_id)
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            rospy.sleep(0.1)

    def unpause(self):
        self.is_running = False
        if self.thread is not None:
            self.thread.join()

    def getLastError(self):
        if self.LastError is not None:
            exc_type, exc_value, exc_traceback = self.LastError
            print(f"Exception Type: {exc_type}")
            print(f"Exception Value: {exc_value}")
            print("Exception Traceback:")
            traceback_str = "\n".join(self.LastError.format_tb(exc_traceback))
            print(traceback_str)
        else:
            print("No recent errors.")
        return self.LastError

    def startNavigation(self):
        try:
            # Start lidar and odometer communication
            launch_command = "roslaunch myagv_navigation navigation_active.launch"
            subprocess.run(
                ['gnome-terminal', '-e', f"bash -c '{launch_command}; exec $SHELL'"])
        except Exception as e:
            self.LastError = sys.exc_info()
            print(e)


if __name__ == '__main__':
    # init navigation
    map_navigation = MapNavigation()

    print(map_navigation.getRobotVersion())
    print(map_navigation.getSystemVersion())

    print(map_navigation.isAgvOn())

    # map_navigation.getLastError()

    # map_navigation.initPosition(1.1, -0.22,  0.7, 0.99,0.55)

    # print(map_navigation.getPosition())

    # map_navigation.getLastError()

    # map_navigation.turnRight(0.2,5)

    # map_navigation.goStraight(0.2,5)
