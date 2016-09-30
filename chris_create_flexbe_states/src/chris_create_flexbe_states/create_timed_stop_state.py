#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient

from geometry_msgs.msg import TwistStamped
from chris_create_node.msg import CreateSensorState
from nav_msgs.msg import Odometry


class CreateTimedStopState(EventState):
    '''
    This state publishes a constant zero TwistStamped command based on parameters.  The state monitors the iRobot Create odometry and
    returns a failed outcome if speed is not near zero within the timeout

    -- timeout         float     Time which needs to have passed since the behavior started. (default: 0.2)
    -- odom_topic      string    topic of the iRobot Create sensor state (default: '/create_node/odom')
    -- cmd_topic       string    topic name of the robot command (default: '/create_node/cmd_vel')
    <= done                 Given time has passed.
    <= failed               A bumper was activated prior to completion
    '''

    def __init__(self, timeout=0.2, cmd_topic='/create_node/cmd_vel', odom_topic='/create_node/odom'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(CreateTimedStopState, self).__init__(outcomes = ['done', 'failed'])

        # Store state parameter for later use.
        self._timeout           = rospy.Duration(timeout)
        self._twist             = TwistStamped()

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done       = None # Track the outcome so we can detect if transition is blocked

        self._odom_topic   = odom_topic
        self._cmd_topic    = cmd_topic
        self._odom_sub     = ProxySubscriberCached({self._odom_topic: Odometry})
        self._pub          = ProxyPublisher(       {self._cmd_topic: TwistStamped})

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if (rospy.Time.now() - self._start_time) > self._timeout:
            # Normal completion, verify stoppage
            if (self._sub.has_msg(self._odom_topic)):
                odom = self._sub.get_last_msg(self._odom_topic)
                speed = odom.twist.twist.linear.x*odom.twist.twist.linear.x + odom.twist.twist.angular.z*odom.twist.twist.angular.z
                if (speed > 5.0e-4):
                    Logger.logwarn('Stop failed twist: linear = %f,%f,%f angular=%f, %f, %f' %
                        (odom.twist.twist.linear.x,  odom.twist.twist.linear.y,  odom.twist.twist.linear.z,
                         odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z ))
                    self._done = 'failed'
                    return 'failed'
            self._done = 'done'
            return 'done'


        # Normal operation publish the zero twist
        self._twist.header.stamp = rospy.Time.now()
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._start_time = rospy.Time.now()
        self._done       = None # reset the completion flag
