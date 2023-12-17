#!/usr/bin/env python
"""
.. module:: robot_states
    :platform: Unix
    :synopsis: the robot_states python script in ros-moveit-opencv-ontology package

.. moduleauthor:: Saeed Kalateh <s4982001@studenti.unige.it>

Subscribes to:
    /odom

Uses Service:
    /state/set_battery_level

    /state/get_battery_level
    
    /state/get_pose
  
    /state/set_base_movement_state
  
    /state/get_base_movement_state

This node defines battery level, robot pose and robot base movement state in order to be used by other
nodes in the software architecture. 
"""
import rospy
from assignment2 import architecture_name_mapper as anm
from assignment2.msg import Point
from assignment2.srv import GetPose, GetPoseResponse, GetBatteryLevel, SetBatteryLevel, GetBatteryLevelResponse, SetBatteryLevelResponse
from assignment2.srv import GetBaseMovementState, GetBaseMovementStateResponse, SetBaseMovementState, SetBaseMovementStateResponse
from nav_msgs.msg import Odometry

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE

class RobotState:
    """
        Inits ``robot-states`` node to provide some usefull information about robot current state, such as
        pose, battery level and base movement state
    """
    def __init__(self):
        # Initialise this node
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        # Initialise robot position
        self._pose = Point()
        # Initialise robot battery level
        self._battery_level = 1000
        # Initialise robot movement state
        self._base_movement_state = False
        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_GET_BATTERY_LEVEL, GetBatteryLevel, self.get_battery_level)
        rospy.Service(anm.SERVER_SET_BATTERY_LEVEL, SetBatteryLevel, self.set_battery_level)
        rospy.Service(anm.SERVER_SET_BASE_MOVEMENT_STATE, SetBaseMovementState, self.set_base_movement_state)
        rospy.Service(anm.SERVER_GET_BASE_MOVEMENT_STATE, GetBaseMovementState, self.get_base_movement_state)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        # Log information.
        log_msg = (f'Initialise node `{anm.NODE_ROBOT_STATE}` with services `{anm.SERVER_GET_POSE}` and '
                   f' `{anm.SERVER_GET_BATTERY_LEVEL}` and `{anm.SERVER_SET_BATTERY_LEVEL}` and '
                   f' `{anm.SERVER_GET_BASE_MOVEMENT_STATE}` and `{anm.SERVER_SET_BASE_MOVEMENT_STATE}`.')
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        while not rospy.is_shutdown(): 
            if self._base_movement_state == True:
                self._battery_level -= 1
            rospy.sleep(1)

    def odom_callback(self, data):
        """
            Callback function for ``/odom`` topic subscriber, update robot current pose in ``robot-states``
            node.

            Args:
                data(nav_msgs.msg.Odometry)
        """
        self._pose.x = data.pose.pose.position.x
        self._pose.y = data.pose.pose.position.y

    def get_pose(self, request):
        """
            The `state/get_pose` service implementation.
            The `request` input parameter is given by the client as empty. Thus, it is not used.
            The `response` returned to the client contains the current robot pose.

            Args:
                request(GetPoseRequest)
            
            Returns:
                response(GetPoseResponse)
        """
        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            log_msg = f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        response = GetPoseResponse()
        response.pose = self._pose
        return response

    def set_battery_level(self, request):
        """
            The `state/set_battery_level` service implementation.
            The `request` input parameter is the current robot battery level to be set,
            as given by the client. This server returns an empty `response`.

            Arg:
                request(SetBatteryLevelRequest)
        """
        if request.battery_level is not None:
            self._battery_level = request.battery_level
            log_msg = (f'Set current robot battery level through `{anm.SERVER_SET_BATTERY_LEVEL}` '
                             f'as ({self._battery_level}).')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot battery level', LOG_TAG))
        return SetBatteryLevelResponse()

    def get_battery_level(self, request):
        """
            The `state/get_battery_level` service implementation.
            The `request` input parameter is given by the client as empty. Thus, it is not used.
            The `response` returned to the client contains the current robot battery level.

            Args:
                request(GetBatteryLevelRequest)

            Returns:
                response(GetBatteryLevelResponse)
        """
        if self._battery_level is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot battery level', LOG_TAG))
        else:
            log_msg = f'Get current robot battery level through `{anm.SERVER_GET_BATTERY_LEVEL}` as ({self._battery_level})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        response = GetBatteryLevelResponse()
        response.battery_level = self._battery_level
        return response

    def set_base_movement_state(self, request):
        """
            The `state/set_base_movement_state` service implementation.
            The `request` input parameter is the current robot base movement state to be set,
            as given by the client. This server returns an empty `response`.

            Arg:
                request(SetBaseMovementStateRequest)
        """
        if request.base_movement_state is not None:
            self._base_movement_state = request.base_movement_state
            log_msg = (f'Set current robot movement state through `{anm.SERVER_SET_BASE_MOVEMENT_STATE}` '
                       f'as ({self._base_movement_state}).')
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot movement state', LOG_TAG))
        return SetBaseMovementStateResponse()

    def get_base_movement_state(self, request):
        """
            The `state/get_base_movement_state` service implementation.
            The `request` input parameter is given by the client as empty. Thus, it is not used.
            The `response` returned to the client contains the current robot base movement state.

            Args:
                request(GetBaseMovementStateRequest)

            Returns:
                response(GetBaseMovementStateResponse)
        """
        if self._base_movement_state is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot movement state', LOG_TAG))
        else:
            log_msg = f'Get current robot movement state through `{anm.SERVER_GET_BASE_MOVEMENT_STATE}` as ({self._base_movement_state})'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        response = GetBaseMovementStateResponse()
        response.base_movement_state = self._base_movement_state
        return response


if __name__ == "__main__":
    RobotState()
    rospy.spin()

