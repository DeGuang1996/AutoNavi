#!/usr/bin/env python

# ROS imports
import roslib
import rospy
import numpy as np
import math
import copy

#ROS messages
from geometry_msgs.msg import Pose2D, Point
from autonomous_nav.msg import PotentialGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool

from autonomous_nav.srv import PotentialPlanner, PotentialPlannerResponse, PotentialPlannerRequest
from autonomous_nav.srv import WaypointProposition, WaypointPropositionResponse, WaypointPropositionRequest

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from threading import Thread, Lock

from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

def angle_wrap(ang):
    """
    Return the angle normalized between [-pi, pi].
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang

class WaypointHistory:
    def __init__(self):
        self.wp_history = None
        self.hist_size = 70
        self.thres = 0.35
        
    def isWaypointInHistory(self , msg_pose):
        if self.wp_history is None:
            return False

        waypoints_hist = self.wp_history - np.array([msg_pose.x, msg_pose.y])
        eucl_dist = np.sqrt(np.sum(np.power(waypoints_hist, 2.0), axis=1)) #axis = 1 -> sum the rows

        # Computing Eucleadean Distance by checking Threshold
        return np.min(eucl_dist) < self.thres

        
    def addWaypointToHistory(self, msg_pose ):
        if self.wp_history is None:
            self.wp_history = np.array([[msg_pose.x, msg_pose.y]])
        else:
            self.wp_history = np.vstack((self.wp_history, 
                                         np.array([[msg_pose.x, msg_pose.y]])))
            
            if self.wp_history.shape[0] > self.hist_size:
                self.wp_history = np.delete( self.wp_history , (0), axis=0 )

    def getRvizMarkers(self):
        marker = Marker()

        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontier_history"
        marker.id = 0

        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        
        marker.scale.x = marker.scale.y = self.thres
        marker.scale.z = 0.06

        marker.color.a = 0.5
        marker.color.r = 0.9
        marker.color.g = marker.color.b = 0.2

        for i in range(self.wp_history.shape[0]):
            p = Point()
            p.x = self.wp_history[i, 0]
            p.y = self.wp_history[i, 1]
            p.z = 0.03

            marker.points.append(p)    

        return marker


class MissionHandler:
    """Class to provide the turtle specified waypoint."""

    def __init__( self ):
        """Class constructor."""
        rospy.init_node('mission_node')

        self.mutex = Lock()

        # Initialize flags
        self.exploring = True
        self.needs_new_frontier = True
        self.next_frontier_random = False
        self.current_wp = None
        self.fresh_frontiers = False

        # Initalize variables
        self.robot_x = self.robot_y = self.robot_theta = None

        self.frontiers = None
        self.waypoint_filter = WaypointHistory()

        # UPDATE
        self.auto_goal = Pose2D()
        self.found_waypoint = False
        self.hist_count = 1
        self.force = True
        self.start_x = 0.0
        self.start_y = 0.0
        self.started = False
        self.returned = False

        self.gotFirst = False

        # Mission Handler Publisher
        self.pub_rviz = rospy.Publisher("/mission_visualize", Marker, queue_size=10)

        # Mission Handler Subscribers
        rospy.Subscriber("/odom", Odometry, self.odometryCallback)
        rospy.Subscriber("/potential_map", PotentialGrid, self.frontierCallback, queue_size=1)
        # rospy.Subscriber("/mission_replan", Bool, self.replanCallback)
        # UPDATE
        rospy.Subscriber("/move_base_simple/auto_goal_find", Int8, self.autoGoalFindCallback, queue_size=1)
        rospy.Subscriber("/move_base_simple/invalid_path", Int8, self.invalidPathCallback, queue_size=1)
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumperCallback, queue_size=1)
        self.auto_goal_pub = rospy.Publisher("/move_base_simple/auto_goal", PoseStamped, queue_size=1)
        self.control_input_pub_ = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size = 10)
        
        # self.tel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        

        # Path planning service proxy (object connecting to service)
        # rospy.wait_for_service('/autonomous_nav/WaypointProposition')
        # try:
        #     self.propose_waypoint = rospy.ServiceProxy('/autonomous_nav/WaypointProposition', WaypointProposition, persistent=True)
        # except rospy.ServiceException, e:
        #     print("Service call failed: " + str(e))

    def rotateOnce(self):
        print ('current orientation' + str(self.robot_theta))
        control_input = Twist()
        control_input.angular.z = 0.3
        #rate = rospy.Rate(10) # 10hz
        while (np.abs(self.robot_theta) < 0.5).any():
            #control_input.angular.z  = control_input.angular.z * 1.1 + 0.05
            # self.vmsg.linear.x = min(0.2, self.vmsg.linear.x)
            # rospy.loginfo("driver speed : %f %f", self.vmsg.angular.z, self.vmsg.linear.x)
            self.control_input_pub_.publish(control_input)

        #rospy.sleep(1.0)
        rotate = 0
        while True:
            #control_input.angular.z  = control_input.angular.z * 1.1 + 0.05
            # self.vmsg.linear.x = min(0.2, self.vmsg.linear.x)
            # rospy.loginfo("driver speed : %f %f", self.vmsg.angular.z, self.vmsg.linear.x)
            self.control_input_pub_.publish(control_input)
            #print ('current orientation' + str(self.current_orientation_))
            #rospy.sleep(0.1)
            if (np.abs(self.robot_theta) < 0.1).any():
                rotate += 1
                print ('rotate' + str(rotate))
            if rotate == 1:
                break

    def invalidPathCallback(self, msg):
        self.mutex.acquire()
        self.hist_count += 1
        print("invalid, use past")
        print(self.waypoint_filter.wp_history)

        target_goal = PoseStamped()
        target_goal.header.seq = 0
        target_goal.header.stamp = rospy.get_rostime()
        target_goal.header.frame_id = "map"
        target_goal.pose.position.z = 0.0
        target_goal.pose.orientation.x = 0.0
        target_goal.pose.orientation.y = 0.0
        target_goal.pose.orientation.z = 0.0
        target_goal.pose.orientation.w = 1.0

        if (self.hist_count <= 3) and self.waypoint_filter.wp_history is not None and self.waypoint_filter.wp_history.shape[0] > self.hist_count:
            
            target_goal.pose.position.x = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 0]
            target_goal.pose.position.y = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 1]

            print(target_goal.pose.position.x, target_goal.pose.position.y)

            self.auto_goal_pub.publish(target_goal)

        else:
            print("invalid, use force")
            self.force = True
            command = Twist()
            rate = rospy.Rate(10)
            command.linear.x = 0.1
            command.angular.z = 0
            i = 0
            while self.force and i < 60:
                self.control_input_pub_.publish(command)
                i += 1
                rate.sleep()
            print("back safety dis")
            safety_dis = 0.01

            target_goal.pose.position.x = self.robot_x - math.cos(self.robot_theta) * safety_dis
            target_goal.pose.position.y = self.robot_y - math.sin(self.robot_theta) * safety_dis

            print(target_goal.pose.position.x, target_goal.pose.position.y)

            self.auto_goal_pub.publish(target_goal)

        self.mutex.release()

    def bumperCallback(self, msg):
        # self.mutex.acquire()
        self.force = False
        # command = Twist()
        # rate = rospy.Rate(10)
        # command.linear.x = -0.1
        # if msg.bumper == BumperEvent.LEFT:
        #     command.angular.z = -0.4
        #     for i in range(0, 10):
        #         self.control_input_pub_.publish(command)
        #         rate.sleep()
        # elif msg.bumper == BumperEvent.CENTER:
        #     command.angular.z = 0
        #     for i in range(0, 10):
        #         self.control_input_pub_.publish(command)
        #         rate.sleep()
        # else:
        #     command.angular.z = 0.4
        #     for i in range(0, 10):
        #         self.control_input_pub_.publish(command)
        #         rate.sleep()
        # self.force = True
        # self.rotateOnce()
        # self.mutex.release()
        # self.invalidPathCallback("1")


    def autoGoalFindCallback(self, msg):

        # self.rotateOnce()
        self.needs_new_frontier = True
        # self.next_frontier_random = True
        self.force = False

        print("start find")
        print(self.frontiers)
        self.proposeWaypoints()

        target_goal = PoseStamped()
        target_goal.header.seq = 0
        target_goal.header.stamp = rospy.get_rostime()
        target_goal.header.frame_id = "map"
        target_goal.pose.position.z = 0.0
        target_goal.pose.orientation.x = 0.0
        target_goal.pose.orientation.y = 0.0
        target_goal.pose.orientation.z = 0.0
        target_goal.pose.orientation.w = 1.0

        if self.found_waypoint:
            print("found goal")
            
            target_goal.pose.position.x = self.auto_goal.x
            target_goal.pose.position.y = self.auto_goal.y

            self.auto_goal_pub.publish(target_goal)

            self.updateCurrentWaypoint()

            self.hist_count = 1
            self.returned = False
        else:
            print("use past goal")
            self.hist_count += 1
            print(self.waypoint_filter.wp_history)
            if (self.hist_count <= 3) and self.waypoint_filter.wp_history is not None and self.waypoint_filter.wp_history.shape[0] > self.hist_count:
                target_goal.pose.position.x = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 0]
                target_goal.pose.position.y = self.waypoint_filter.wp_history[self.waypoint_filter.wp_history.shape[0] - self.hist_count, 1]

                self.auto_goal_pub.publish(target_goal)
            else:
                if self.returned is False:
                    target_goal.pose.position.x = self.start_x
                    target_goal.pose.position.y = self.start_y

                    self.auto_goal_pub.publish(target_goal)
                    self.returned = True
                    print("return phase")
                else:
                    print("returned, auto navi ended")
                

    def replanCallback(self, msg):
        self.mutex.acquire()

        # If msg.data -> random replan
        if msg.data:
            self.needs_new_frontier = True
            self.next_frontier_random = True
        else:
            self.needs_new_frontier = True

        self.mutex.release()

    def odometryCallback(self, msg):
        self.mutex.acquire()

        self.robot_x = msg.pose.pose.position.x 
        self.robot_y = msg.pose.pose.position.y 

        quaternion = (msg.pose.pose.orientation.x, 
                      msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, 
                      msg.pose.pose.orientation.w)

        self.robot_theta = euler_from_quaternion(quaternion)
        self.robot_theta = angle_wrap(self.robot_theta[2])

        if self.started is False:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            print("start: " + str(self.start_x) + "," + str(self.start_y))
            self.started = True

        # print(self.robot_theta)
        # print(self.robot_x, self.robot_y)

        self.mutex.release()

    # Calling callback
    def frontierCallback(self, msg):

        if self.gotFirst is False:
            rospy.logwarn("potential map first got")
            self.gotFirst = True

        self.mutex.acquire()

        if not self.exploring:
            self.mutex.release()
            return
        
        w = int(msg.info.width)
        
        self.frontiers = None
        for i in range(2, msg.info.height - 1):
            for j in range(2, msg.info.width - 1):
                # If not free space, continue
                if msg.data[w*i + j] < 3: 
                    continue

                if  (msg.data[w*i + j - 1] == 2 and msg.data[w*(i - 1) + j] == 1) or \
                        (msg.data[w*i + j - 1] == 1 and msg.data[w*(i - 1) + j] == 2) or \
                        (msg.data[w*i + j + 1] == 2 and msg.data[w*(i - 1) + j] == 1) or \
                        (msg.data[w*i + j + 1] == 1 and msg.data[w*(i - 1) + j] == 2) or \
                        (msg.data[w*(i + 1) + j] == 2 and msg.data[w*i + j + 1] == 1) or \
                        (msg.data[w*(i + 1) + j] == 1 and msg.data[w*i + j + 1] == 2) or \
                        (msg.data[w*(i + 1) + j] == 2 and msg.data[w*i + j - 1] == 1) or \
                        (msg.data[w*(i + 1) + j] == 1 and msg.data[w*i + j - 1] == 2) or \
                        (msg.data[w*(i + 1) + j] == 2 and msg.data[w*(i - 1) + j] == 1) or \
                        (msg.data[w*(i + 1) + j] == 1 and msg.data[w*(i - 1) + j] == 2) or \
                        (msg.data[w*i + j - 1] == 2 and msg.data[w*i + j + 1] == 1) or \
                        (msg.data[w*i + j - 1] == 1 and msg.data[w*i + j + 1] == 2) :

                    frontier_x = float(j * msg.info.resolution + msg.info.origin.position.x)
                    frontier_y = float(i * msg.info.resolution + msg.info.origin.position.y)

                    angle_to_robot = np.abs(angle_wrap(self.robot_theta - np.arctan2(frontier_y - self.robot_y, frontier_x - self.robot_x)))

                    frontier_weight = msg.data[w*i + j] * (1 + 0.8*np.exp(-2.0/angle_to_robot))

                    if self.frontiers is None:
                        self.frontiers = np.array([[frontier_x, frontier_y, frontier_weight]])
                    else:                    
                        self.frontiers = np.vstack((self.frontiers, 
                                                    np.array([[frontier_x, frontier_y, frontier_weight]])))

        self.fresh_frontiers = True
        # self.showFrontiersRviz()

        # rospy.sleep(1)

        self.mutex.release()

    def updateCurrentWaypoint(self):
        self.mutex.acquire()

        # if self.frontiers is None or self.current_wp is None:
        #     self.mutex.release()
        #     return

        # # Check if last waypoint still exists
        # for i in range(self.frontiers.shape[0]):
        #     if np.abs(self.current_wp.x - self.frontiers[i, 0]) < 0.04 and \
        #             np.abs(self.current_wp.y - self.frontiers[i, 1]) < 0.04:
        #         # Locked Waypoint found, checking if reached

        #         if np.sqrt(np.power(self.current_wp.x - self.robot_x, 2) + \
        #                 np.power(self.current_wp.y - self.robot_y, 2)) < 0.2:
        #             self.waypoint_filter.addWaypointToHistory(self.current_wp)
        #             self.pub_rviz.publish(self.waypoint_filter.getRvizMarkers())
        #             rospy.loginfo('Current waypoint reached, adding to history...')
        #             break
        #         else:
        #             self.mutex.release()
        #             return
        if self.current_wp is not None:
            self.waypoint_filter.addWaypointToHistory(self.current_wp)

        # Waypoint reached or dissapeared -> Put flags for recomputing
        self.fresh_frontiers = False
        self.needs_new_frontier = True

        self.mutex.release()
    
    def showFrontiersRviz(self):
        '''
        Shows frontiers in self.frontiers
        '''

        if self.frontiers is None:
            return

        marker = Marker()

        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontiers"
        marker.id = 0

        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        marker.scale.x = marker.scale.y = 0.1
        marker.scale.z = 0.15

        marker.color.a = 0.75
        marker.color.r = marker.color.g = marker.color.b = 0.9

        for i in range(self.frontiers.shape[0]):
            p = Point()
            p.x = self.frontiers[i, 0]
            p.y = self.frontiers[i, 1]
            p.z = 0.075

            marker.points.append(p)

        self.pub_rviz.publish(marker)

    def showWaypointRviz(self, msg_pose):
        marker = Marker()

        marker.header.frame_id = "/odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoint"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.scale.y = marker.scale.z = 0.1
        marker.scale.x = 0.3

        marker.color.a = 1.0
        marker.color.b = 1.0
        marker.color.r = marker.color.g = 0.4

        marker.pose.position.x = msg_pose.x
        marker.pose.position.y = msg_pose.y
        marker.pose.position.z = 0.3

        quaternion = quaternion_from_euler(0, np.pi/2.0, 0)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]

        self.pub_rviz.publish(marker)

    def showTextRobot(self, text):
        marker = Marker()

        marker.header.frame_id = "/base_link"
        marker.frame_locked = True
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_text"
        marker.id = 0

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = marker.color.g = 0.9
        marker.color.b = 0.6

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5

        marker.text = text

        self.pub_rviz.publish(marker)


    # Calling frontiers function
    def proposeWaypoints(self):

        if self.gotFirst is False:
            rospy.logwarn("potential map not got, try again later")
            rospy.sleep(2)

        self.mutex.acquire()

        if self.frontiers is None:
            self.frontiers = np.array([[0.0, 0.0, 1.0]])

        # If last waypoint explored, look for next frontier
        self.found_waypoint = False
        waypoints_tried = 0

        while not self.found_waypoint and waypoints_tried < 2*self.frontiers.shape[0]:
            if not self.next_frontier_random:
                row_min = np.argmin(self.frontiers[:, 2])
            else:
                # Propose random waypoint
                row_min = np.random.random_integers(0, self.frontiers.shape[0]-1)

            print("last:" + str(self.auto_goal.x) + "," + str(self.auto_goal.y))
            print("selected: " + str(self.frontiers[row_min, 0]) + "," + str(self.frontiers[row_min, 1]))

            if (self.auto_goal.x != self.frontiers[row_min, 0] or self.auto_goal.y != self.frontiers[row_min, 1]):
                self.auto_goal.x = self.frontiers[row_min, 0] 
                self.auto_goal.y = self.frontiers[row_min, 1]

                # msg_pose = Pose2D()
                # msg_pose.x = self.frontiers[row_min, 0] 
                # msg_pose.y = self.frontiers[row_min, 1]

                self.showWaypointRviz(self.auto_goal)

            # Only consider history if we are proposing frontiers for the first time
                if waypoints_tried < self.frontiers.shape[0] and self.waypoint_filter.isWaypointInHistory(self.auto_goal):
                    self.frontiers[row_min, 2] *= 10
                    waypoints_tried += 1
                    continue
            
                self.current_wp = self.auto_goal
                self.found_waypoint = True
            
            else:
                print("not found, break")
                break

            # # Make waypoint proposition to Controller
            # proposition = WaypointPropositionRequest()
            # proposition.waypoint.x = msg_pose.x
            # proposition.waypoint.y = msg_pose.y

            # rospy.loginfo("Mission: Proposing waypoint")
            # response = WaypointPropositionResponse()
            # response.accepted = False
            # try:
            #     response = self.propose_waypoint(proposition)
            # except rospy.ServiceException, e:
            #     rospy.logwarn("Mission: " + str(e))

            # # Process proposition response
            # if response.accepted:
            #     self.current_wp = msg_pose
            #     self.needs_new_frontier = False
            #     found_waypoint = True
            # else:
            #     self.frontiers[row_min, 2] *= 10
            #     waypoints_tried += 1
        
        self.fresh_frontiers = False
        self.next_frontier_random = False

        self.mutex.release()

    # def finishExploration(self, event):
    #     self.exploring = False

    #     rospy.loginfo("\n\n\nExploration time finished. Returning to (0,0).")

    #     self.showTextRobot('Returning...')

    #     proposition = WaypointPropositionRequest()
    #     proposition.waypoint.x = 0.0
    #     proposition.waypoint.y = 0.0
        
    #     response = WaypointPropositionResponse()
    #     response.accepted = False
        
    #     found_path = False
    #     while not found_path:
    #         rospy.sleep(0.3)

    #         try:
    #             response = self.propose_waypoint(proposition)

    #             found_path = response.accepted

    #             if not found_path:
    #                 rospy.logwarn("No path home found :(, trying again...")
    #         except rospy.ServiceException, e:
    #             rospy.logwarn("PROBLEM WHEN RETURNING: " + str(e))




if __name__ == '__main__':

    mission_handler = MissionHandler()
    rospy.spin()


    # try:
    #     mission_handler = MissionHandler()

    #     exploration_time = rospy.get_param('exploration_time', 60)
        
    #     rospy.sleep(1)
    #     rospy.loginfo("\n\n\nAutonomous_nav starting... exploration time " + str(exploration_time) + " s. \n\n")
    #     rospy.sleep(2)

    #     t = rospy.Timer(rospy.Duration(exploration_time), mission_handler.finishExploration, oneshot=True)

    #     r = rospy.Rate(5)
    #     while not rospy.is_shutdown():
    #         if not mission_handler.exploring:
    #             r.sleep()
    #             continue

    #         if mission_handler.needs_new_frontier and mission_handler.fresh_frontiers:
    #             mission_handler.proposeWaypoints()

    #         mission_handler.updateCurrentWaypoint()

    #         r.sleep()
            
            
    # except rospy.ROSInterruptException:
    #     pass
