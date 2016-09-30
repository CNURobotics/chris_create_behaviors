#!/usr/bin/env python
import rospy
import tf

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxyTransformListener

from geometry_msgs.msg import TwistStamped,Point,PointStamped
from chris_create_node.msg import CreateSensorState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class CreateWaypointState(EventState):
    '''
    This state defines the starting point for a pure pursuit system..

       -- target_frame:        string    Coordinate frame of target point (default: 'map')
       -- target_x:            float     Target point x-coordinate (m)
       -- target_y:            float     Target point y-coordinate (m)
       -- marker_topic:        string    topic of the RViz marker used for visualization (default: '/pure_pursuit_marker')
       -- marker_size:         float     Size of RViz marker used for target (default: 0.05)
       #> target_point         object    The target point (becomes next node's prior)
       <= done                 Reached the end of target relevance
    '''

    def __init__(self,  target_frame='map', target_x=1.0, target_y=0.1,
                        marker_topic='/pure_pursuit_marker', marker_size=0.05):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(CreateWaypointState, self).__init__(outcomes = ['done'],
                                                     output_keys = ['target_point'])

        # Store state parameter for later use.
        self._target = PointStamped()
        self._target.header.stamp = rospy.Time.now()
        self._target.header.frame_id = target_frame
        self._target.point = Point(target_x,target_y,0.0)

        self._done       = None # Track the outcome so we can detect if transition is blocked

        self._marker_topic = marker_topic
        self._marker_pub   = ProxyPublisher(       {self._marker_topic: Marker})

        self._marker       = Marker()
        self._marker.header.frame_id = target_frame
        self._marker.header.stamp = rospy.Time.now()
        self._marker.ns = "pure_pursuit_waypoints"
        self._marker.id = int(target_x*1000000)+int(target_y*1000)
        self._marker.type   = Marker.SPHERE
        self._marker.action = Marker.ADD
        self._marker.pose.position.x = target_x
        self._marker.pose.position.y = target_y
        self._marker.pose.position.z = 0.0
        self._marker.pose.orientation.x = 0.0
        self._marker.pose.orientation.y = 0.0
        self._marker.pose.orientation.z = 0.0
        self._marker.pose.orientation.w = 1.0
        self._marker.scale.x = marker_size
        self._marker.scale.y = marker_size
        self._marker.scale.z = marker_size
        self._marker.color.a = 1.0  # Don't forget to set the alpha!
        self._marker.color.r = 0.0
        self._marker.color.g = 1.0
        self._marker.color.b = 1.0

    # This is only to pass data along to the next state
    def execute(self, userdata):
         userdata.target_point = self._target   # Set the user data for passing to next node
         return 'done'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._marker.action  = Marker.MODIFY
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 1.0 # Indicate this target is active
        self._marker.color.b = 0.0
        self._marker_pub.publish(self._marker_topic,self._marker)

    def on_exit(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._marker.color.a = 1.0 # Don't forget to set the alpha!
        self._marker.color.r = 0.8 # Indicate this target is no longer active
        self._marker.color.g = 0.0
        self._marker.color.b = 0.0
        self._marker_pub.publish(self._marker_topic,self._marker)

    def on_start(self):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._marker.action  = Marker.ADD
        self._marker.color.a = 1.0 # Set alpha otherwise the marker is invisible
        self._marker.color.r = 0.0
        self._marker.color.g = 1.0
        self._marker.color.b = 1.0 # Indicate this target is planned
        self._marker_pub.publish(self._marker_topic,self._marker)

