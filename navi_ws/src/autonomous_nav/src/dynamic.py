#!/usr/bin/env python

# ROS imports
import roslib
import rospy
import numpy as np
import math
import copy

import dynamic_reconfigure.client

from std_msgs.msg import Int8

class Dynamic_Config:
    def __init__( self ):
        """Class constructor."""
        rospy.init_node('dynamic_config')

        self.update = False

        rospy.Subscriber("/move_base_simple/failed_path", Int8, self.invalidPathCallback, queue_size=1)
        rospy.Subscriber("/move_base_simple/invalid_path", Int8, self.invalidPathCallback, queue_size=1)
        rospy.Subscriber("/move_base_simple/auto_goal_find", Int8, self.autoGoalFindCallback, queue_size=1)


    def invalidPathCallback(self, msg):
        if self.update is False:
            client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
            params_local = { 'inflation_radius' : '0.4' }
            config_local = client_local.update_configuration(params_local)
            print("local:")
            print(config_local)

            client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
            params_global = { 'inflation_radius' : '0.3' }
            config_global = client_global.update_configuration(params_global)
            print("global:")
            print(config_global)

            client_auto = dynamic_reconfigure.client.Client("/potential_map/autoConfig")
            params_auto = { 'inflation_radius' : '6' }
            config_auto = client_auto.update_configuration(params_auto)
            print("auto:")
            print(config_auto)

        self.update = True


    def autoGoalFindCallback(self, msg):
        if self.update is True:
            client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
            params_local = { 'inflation_radius' : '0.4' }
            config_local = client_local.update_configuration(params_local)
            print("local:")
            print(config_local)

            client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
            params_global = { 'inflation_radius' : '0.6' }
            config_global = client_global.update_configuration(params_global)
            print("global:")
            print(config_global)

            client_auto = dynamic_reconfigure.client.Client("/potential_map/autoConfig")
            params_auto = { 'inflation_radius' : '8' }
            config_auto = client_auto.update_configuration(params_auto)
            print("auto:")
            print(config_auto)

        self.update = False


if __name__ == '__main__':
    dynamic_Config = Dynamic_Config()
    rospy.spin()
