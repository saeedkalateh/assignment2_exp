#!/usr/bin/env python
"""
.. module:: finite_state_machine
    :platform: Unix
    :synopsis: the fsm python script in assignment2 package

.. moduleauthor:: Saeed Kalateh <s4982001@studenti.unige.it>

Subscribes to:
    /image_id

Uses Service:
    /state/set_battery_level

    /state/get_pose

    /state/set_base_movement_state

    /room_info

    /move_arm

Uses Action:
    /move_base

Uses helper script:
    /utilities/assignment2/helper.py

Defines the states and transitions for the finite state machine of this rospackage. In the initial state
robot builds the semantic map of environment using image id of the markers detected by robot camera. This node
uses ``helper.py`` script to update the ontology while the process is running, and retreives
the target room based on last visit times and robot battey state. In the next state it moves to the target room
and if battery level gets lower than threshold, it goes to charger, and charges the battery.
"""
import rospy
import smach
import smach_ros
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from assignment2.msg import Point
from assignment2.srv import SetBatteryLevel, GetPose, SetBaseMovementState, RoomInformation
from assignment2 import architecture_name_mapper as anm
from assignment2.helper import TopologicalMap
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

# Import mutex to manage synchronization among ROS-based threads (i.e., node loop and subscribers)
from threading import Lock

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE
LOOP_TIME = 5

pub = None
tm = None
mutex = None
rooms_id = []
rooms_name = []
rooms_center = []
rooms_connections = []
rooms_doors = []

def get_room_info(room_id):
    """
        Server client for ``marker_server``, gets information for each room using ``room_info`` service

        Args: 
            room_id(int)

        Returns:
            resp(RoomInformationResponse)
    """
    rospy.wait_for_service('room_info')
    try:
        srv = rospy.ServiceProxy('room_info', RoomInformation)
        resp = srv(room_id)
        return resp 
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def marker_id_callback(data):
    """
        Callback function for ``/image_id`` topic subscriber. Eveytime an image id is detected, checks 
        if image id is valuable and not already available, then saves the corresponding information of
        each room in the global variables by calling ``get_room_info(room_id)`` function, and modifies
        the ontology using ``add_room(room)``, ``add_door(door)``, ``assign_doors_to_room(room, doors)``
        ``disjoint_individuals()`` and ``add_last_visit_time(room, visit_time)`` functions from 
        ``topological_map.py`` helper script.

        Args:
            data(int32)
    """
    global rooms_id
    global rooms_name
    global rooms_center
    global rooms_connections
    global rooms_doors
    if data.data not in rooms_id and data.data > 10 and data.data < 18:
        rooms_id.append(data.data)
        log_msg = 'Image id detected: %d ' % (data.data)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        log_msg = 'Number of detected IDs: %d ' % (len(rooms_id))
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        
        room_info = get_room_info(data.data)
        rooms_name.append(room_info.room)
        log_msg = 'Semantic map updated, room '+ room_info.room + ' detected'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        tm.add_room(room_info.room)

        rooms_center.append([room_info.x, room_info.y])
        log_msg = 'Center position is: [%f, %f]' % (room_info.x, room_info.y)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        for i in range(len(room_info.connections)):
            rooms_connections.append(room_info.connections[i].connected_to)
            rooms_doors.append(room_info.connections[i].through_door)
            log_msg = 'Room ' + room_info.room + ' is connected to ' + room_info.connections[i].connected_to + ' through door ' + room_info.connections[i].through_door
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
            tm.add_door(room_info.connections[i].through_door)
            tm.assign_doors_to_room(room_info.room, room_info.connections[i].through_door)

        tm.disjoint_individuals()
        tm.add_last_visit_time(room_info.room, str(room_info.visit_time))

def move_to_pose(pose):
    """
        Action client function for ``move_base`` node, gets a pose as an argument and sends it as 
        ``MoveBaseGoal.msg`` to the action server

        Args:
            pose(Point)
        
        Returns:
            result(MoveBaseResult.msg)
    """
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose.x
    goal.target_pose.pose.position.y = pose.y
    goal.target_pose.pose.orientation.w = 1.0
    client.wait_for_server()
    client.send_goal(goal)

def check_target_reached(target_pose):
    """
    Checks if the robot has reached to a specific point by computing the eucledian distance between
    robot current pose and target pose 

    Args:
        target_pose(Point)

    Returns:
        target_reached(Bool)
    """
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        if math.sqrt((target_pose.x - pose.x)**2 + (target_pose.y - pose.y)**2) < 1:
            target_reached = True
        else:
            target_reached = False
        log_msg = 'target reached state: ' + str(target_reached)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return target_reached
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def get_room_pose(room):
    """
    Detects the center postion by using the room information for corresponding room

    Args:
        room(string)

    Returns:
        room_pose(Point)
    """
    global rooms_name
    global rooms_center
    room_pose = Point()
    room_index = rooms_name.index(room)
    room_pose.x = rooms_center[room_index][0]
    room_pose.y = rooms_center[room_index][1]
    return room_pose

def set_battery_level(battery_level):
    """
    Service client function for ``/state/set_battery_level`` Update the current robot battery level
    stored in the ``robot-state`` node

    Args:
        battery_level(int)
    """
    rospy.wait_for_service(anm.SERVER_SET_BATTERY_LEVEL)
    try:
        log_msg = f'Set current robot battery level to the `{anm.SERVER_SET_BATTERY_LEVEL}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy(anm.SERVER_SET_BATTERY_LEVEL, SetBatteryLevel)
        service(battery_level)
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot battery level: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def set_arm_movement_state(arm_movement_state):
    """
        Service client function for ``/move_arm``. Updates the current robot arm movement state stored 
        in ``my_moveit`` node

        Args:
            arm_movement_state(bool)
    """
    rospy.wait_for_service('/move_arm')
    try:
        log_msg = 'Setting robot arm movement state to ' + str(arm_movement_state)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy('move_arm', SetBool)
        service(arm_movement_state)
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current arm movement state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def set_base_movement_state(base_movement_state):
    """
        Service client function for ``/base_movement_state``. Updates the current robot base movement state stored 
        in ``robot-states`` node

        Args:   
            base_movement_state(bool)
    """
    rospy.wait_for_service(anm.SERVER_SET_BASE_MOVEMENT_STATE)
    try:
        log_msg = 'Setting robot base movement state to ' + str(base_movement_state)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        service = rospy.ServiceProxy(anm.SERVER_SET_BASE_MOVEMENT_STATE, SetBaseMovementState)
        service(base_movement_state)  
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current base movement state: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

class CreateTopologicalMap(smach.State):
    """
    Defines the initial state when robot creates the topological map. It enables the robot arm to start moving on the desired 
    trajectories as it is defined in ``my_moveit.cpp`` file using ``set_arm_movement_state(arm_movement_state)``
    function, and then checks if the number of detected room ids are enough, then exits from state by returning ``map_created``
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['map_created'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global rooms_id
        set_arm_movement_state(True)
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                if len(rooms_id) > 6:
                    return 'map_created'
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class MoveToRoom(smach.State):
    """
    Defines the state when robot moves to target room found by the ontology, first enables battery consumption
    using ``set_base_movement_state(base_movement_state)`` function and then moves to target room using 
    ``move_to_pose(pose)`` function. Additionally, it updates the ontology while it moves to target room
    using ``update_ontology(now)`` function, until it reaches the target room, then exits the state by
    returning ``room_reached``. If the battery level gets lower than threshold, it returns ``battery_low``
    and target room will be canceled.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['room_reached', 'battery_low'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        now = rospy.get_rostime()
        [target_room, battery_low] = tm.update_ontology(now)
        target_room_pose = get_room_pose(target_room)
        set_base_movement_state(True)
        move_to_pose(target_room_pose)
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                [next_target_room, battery_low] = tm.update_ontology(now)
                log_msg = 'target room: ' + target_room
                rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
                if battery_low:
                    return 'battery_low'
                else:
                    target_reached = check_target_reached(target_room_pose)
                    if target_reached:
                        set_base_movement_state(False)
                        return 'room_reached'
            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class VisitRoom(smach.State):
    """
    Defines the state when robot has reached the target room and then visits it. It enables robot arm movement
    like the initial state using ``set_arm_movement_state(arm_movement_state)`` functinon, then returns ``room_visited``
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['room_visited'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                set_arm_movement_state(True)
                return 'room_visited'

            finally:
                mutex.release()

class MoveToCharger(smach.State):
    """
    Defines the state when battery level is low and moves to charger, first enables battery consumption
    using ``set_base_movement_state(movement_state)`` function and then moves to charger using 
    ``move_to_pose(pose)`` function. Additionally, it updates the ontology while it moves to charger
    using ``update_ontology(now)`` function, until it reaches the charger.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['docked'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        now = rospy.get_rostime()
        tm.update_ontology(now)
        target_room_pose = get_room_pose('E')
        set_base_movement_state(True)
        move_to_pose(target_room_pose)
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                tm.update_ontology(now)
                target_reached = check_target_reached(target_room_pose)
                if target_reached:
                    set_base_movement_state(False)
                    return 'docked'              

            finally:
                mutex.release()
            rospy.sleep(LOOP_TIME)

class Recharging(smach.State):
    """
    Defines the state when robot has reached the charger and chargers battery after some time using
    ``set_battery_level(battery_level)`` function and then returns ``robot_charged`` transition.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['recharged'])

    def execute(self, userdata):
        """
        Implements the execution of the tasks while this state gets active.
        """
        global mutex
        global tm
        
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            mutex.acquire()
            try:
                now = rospy.get_rostime()
                tm.update_ontology(now)
                rospy.sleep(10)
                set_battery_level(1000)
                return 'recharged'

            finally:
                mutex.release()


def main():
    """
    The main function for finite_state_machine node, initialises the node and takes an instance of
    ``TopologicalMap`` class in the time instance now, defines the subscriner to the ``/image_id`` topic
    , defines the states and transitions of the finite state machine for topological map and finally 
    starts the finite state machine process
    """
    # Initialise this node.
    global tm
    global pub
    global mutex

    rospy.init_node(anm.NODE_FINITE_STATE_MACHINE, log_level=rospy.INFO)
    now = rospy.get_rostime()
    tm = TopologicalMap(LOG_TAG, now)

    # Get or create a new mutex.
    if mutex is None:
        mutex = Lock()
    else:
        mutex = mutex

    # Subscribe image id to get rooms information
    rospy.Subscriber('/image_id', Int32, marker_id_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CREATE_TOPOLOGICAL_MAP', CreateTopologicalMap(), transitions={'map_created':'MOVE_TO_ROOM'})
        smach.StateMachine.add('MOVE_TO_ROOM', MoveToRoom(), transitions={'battery_low':'MOVE_TO_CHARGER', 'room_reached':'VISIT_ROOM'})
        smach.StateMachine.add('MOVE_TO_CHARGER', MoveToCharger(), transitions={'docked':'RECHARGING'})
        smach.StateMachine.add('VISIT_ROOM', VisitRoom(), transitions={'room_visited':'MOVE_TO_ROOM'})
        smach.StateMachine.add('RECHARGING', Recharging(), transitions={'recharged':'MOVE_TO_ROOM'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
