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


class CreateTimedTwistState(EventState):
    '''
    This state publishes a constant TwistStamped command based on parameters.  The state monitors the iRobot Create bumper status, and
    stops and returns a failed outcome if a bumper is activated.

    -- target_time     float     Time which needs to have passed since the behavior started.
    -- velocity        float     Body velocity (m/s)
    -- rotation_rate   float     Angular rotation (radians/s)
    -- sensor_topic    string    topic of the iRobot Create sensor state (default: '/create_node/sensor_state')
    -- cmd_topic       string    topic name of the robot command (default: '/create_node/cmd_vel')
    <= done                 Given time has passed.
    <= failed               A bumper was activated prior to completion
    '''

    def __init__(self, target_time, velocity, rotation_rate, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(CreateTimedTwistState, self).__init__(outcomes = ['done', 'failed'])

        # Store state parameter for later use.
        self._target_time           = rospy.Duration(target_time)
        self._twist                 = TwistStamped()
        self._twist.twist.linear.x  = velocity
        self._twist.twist.angular.z = rotation_rate

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done       = None # Track the outcome so we can detect if transition is blocked

        self._sensor_topic = sensor_topic
        self._cmd_topic    = cmd_topic
        self._sensor_sub   = ProxySubscriberCached({self._sensor_topic: CreateSensorState})
        self._pub          = ProxyPublisher(       {self._cmd_topic: TwistStamped})

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if (self._done):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = TwistStamped()
            ts.header.stamp = rospy.Time.now()
            self._pub.publish(self._cmd_topic, ts)
            return self._done

        if rospy.Time.now() - self._start_time > self._target_time:
            # Normal completion, do not bother repeating the publish
            self._done = 'done'
            return 'done'

        if (self._sensor_sub.has_msg(self._sensor_topic)):
            # check the status of the bumpers
            sensors = self._sub.get_last_msg(self._sensor_topic)
            if ((sensors.bumps_wheeldrops > 0) or sensors.cliff_left or sensors.cliff_front_left or sensors.cliff_front_right or sensors.cliff_right):
                ts = TwistStamped()
                ts.header.stamp = rospy.Time.now()
                self._pub.publish(self._cmd_topic, ts)
                Logger.logwarn('Bumper contact = %d  cliff: left=%d %d %d %d = right ' %
                        (sensors.bumps_wheeldrops, sensors.cliff_left, sensors.cliff_front_left,sensors.cliff_front_right, sensors.cliff_right))
                return 'failed'




        # Normal operation
        self._twist.header.stamp = rospy.Time.now()
        self._pub.publish(self._cmd_topic, self._twist)
        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._start_time = rospy.Time.now()
        self._done       = None # reset the completion flag
