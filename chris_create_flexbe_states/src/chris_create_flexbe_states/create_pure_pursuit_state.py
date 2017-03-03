#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
from copy import deepcopy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient
from flexbe_core.proxy import ProxyTransformListener

from geometry_msgs.msg import TwistStamped,Point,PointStamped,Vector3
from chris_create_node.msg import CreateSensorState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class CreatePurePursuitState(EventState):
    '''
    This state calculates the twist required to follow a constant curvature arc to the pure pursuit intersection point.
    The command is published as a TwistStamped command based on parameters.
    The state monitors the iRobot Create bumper status, and stops and returns a failed outcome if a bumper is activated.

    If arc motion is used, the arc should be less than or equal to pi/2 radians.  Use multiple targets for longer arcs.

.......-- desired_velocity     float     Desired velocity in m/s (default: 0.2)
.......-- max_rotation_rate    float     Maximum rotation rate radians/s (default: 10.0)
       -- target_frame:        string    Coordinate frame of target point (default: 'map')
       -- target_x:            float     Target point x-coordinate (m)
       -- target_y:            float     Target point y-coordinate (m)
       -- target_type:         string    Desired motion ('line','arc') (default: 'line')
       -- lookahead_distance:  float     Lookahead distance (m) (default:  0.25)
       -- timeout              float     Transform timeout (seconds) (default: 0.08)
       -- recover_mode         bool      Recover path (typically on startup) (default: False)
       -- center_x:            float     Center point x-coordinate for circle defining arc motion (default: 0.0)
       -- center_y:            float     Center point y-coordinate for circle defining arc motion (default: 0.0)
       -- cmd_topic            string    topic name of the robot command (default: '/create_node/cmd_vel')
       -- sensor_topic         string    topic of the iRobot Create sensor state (default: '/create_node/sensor_state')
       -- odometry_topic:      string    topic of the iRobot Create sensor state (default:   '/create_node/odom'
       -- marker_topic:        string    topic of the RViz marker used for visualization (default: '/pure_pursuit_marker')
       -- marker_size:         float     Size of RViz marker used for target (default: 0.05)
       ># prior_point          object    Prior waypoint at start of this segment
       #> target_point         object    Target point at end of this calculation (becomes next node's prior)
       <= done                 Reached the end of target relevance
       <= failed               A bumper was activated prior to completion
    '''

    def __init__(self,  desired_velocity=0.2, max_rotation_rate=10.0,
                        target_frame='map', target_x=1.0, target_y=0.1, target_type='line',
                        lookahead_distance=0.25, timeout=0.08, recover_mode=False,
                        center_x=0.0, center_y=0.0,
                        cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom',
                        marker_topic='/pure_pursuit_marker', marker_size=0.05):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(CreatePurePursuitState, self).__init__(outcomes = ['done', 'failed'],
                                                     input_keys = ['prior_point'],
                                                     output_keys = ['target_point'])

        # Store state parameter for later use.
        self._twist                 = TwistStamped()
        self._twist.twist.linear.x  = desired_velocity
        self._twist.twist.angular.z = 0.0

        self._desired_velocity      = desired_velocity
        self._max_rotation_rate     = max_rotation_rate     # Used to clamp the rotation calculations


        self._current_position = PointStamped()
        self._current_position.header.stamp = rospy.Time.now()
        self._current_position.header.frame_id = target_frame

        self._target = PointStamped()
        self._target.header.stamp = rospy.Time.now()
        self._target.header.frame_id = target_frame
        self._target.point = Point(target_x,target_y,0.0)

        self._center = PointStamped()
        self._center.header = self._target.header
        self._center.point  = Point(center_x,center_y,0.0)

        self._lookahead    = lookahead_distance
        self._recover_mode = recover_mode
        self._target_type  = target_type
        self._target_frame = target_frame
        if (self._target_type == 'arc'):
            self._radius = np.sqrt((center_x - target_x)**2 + (center_y - target_y)**2)

            Logger.loginfo('Using arc type with center=(%d, %d) target=(%d,%d) radius=%d'%(self._center.point.x,self._center.point.y,
                                                                                           self._target.point.x,self._target.point.y,
                                                                                           self._radius))


        self._odom_frame   = 'odom' # Update with the first odometry message
        self._robot_frame  = 'base_footprint'
        self._failed       = False
        self._timeout      = rospy.Duration(timeout) # transform timeout

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

        self._done       = None # Track the outcome so we can detect if transition is blocked

        self._odom_topic   = odometry_topic
        self._sensor_topic = sensor_topic
        self._marker_topic = marker_topic
        self._cmd_topic    = cmd_topic

        self._listener     = ProxyTransformListener()

        self._sensor_sub   = ProxySubscriberCached({self._sensor_topic: CreateSensorState})
        self._odom_sub     = ProxySubscriberCached({self._odom_topic:  Odometry})
        self._pub          = ProxyPublisher(       {self._cmd_topic: TwistStamped})

        if (self._marker_topic != ""):
            self._marker_pub   = ProxyPublisher(       {self._marker_topic: Marker})
        else:
            self._marker_pub = None

        self._marker       = Marker()
        self._marker.header.frame_id = self._target_frame
        self._marker.header.stamp = rospy.Time.now()
        self._marker.ns     = "pure_pursuit_waypoints"
        self._marker.id     = int(target_x*1000000)+int(target_y*1000)
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
        self._marker.color.g = 0.0
        self._marker.color.b = 1.0

        self._reference_marker = Marker()
        self._reference_marker.header.frame_id = self._target_frame
        self._reference_marker.header.stamp = rospy.Time.now()
        self._reference_marker.ns = "pure_pursuit_reference"
        self._reference_marker.id = 1
        self._reference_marker.type   = Marker.SPHERE
        self._reference_marker.action = Marker.ADD
        self._reference_marker.pose.position.x = target_x
        self._reference_marker.pose.position.y = target_y
        self._reference_marker.pose.position.z = 0.0
        self._reference_marker.pose.orientation.x = 0.0
        self._reference_marker.pose.orientation.y = 0.0
        self._reference_marker.pose.orientation.z = 0.0
        self._reference_marker.pose.orientation.w = 1.0
        self._reference_marker.scale.x = marker_size*0.75
        self._reference_marker.scale.y = marker_size*0.75
        self._reference_marker.scale.z = marker_size*0.75
        self._reference_marker.color.a = 0.0  # Add, but make invisible at first
        self._reference_marker.color.r = 1.0
        self._reference_marker.color.g = 0.0
        self._reference_marker.color.b = 1.0

        self._local_target_marker = Marker()
        self._local_target_marker.header.frame_id = self._target_frame
        self._local_target_marker.header.stamp = rospy.Time.now()
        self._local_target_marker.ns = "pure_pursuit_target"
        self._local_target_marker.id = 1
        self._local_target_marker.type   = Marker.SPHERE
        self._local_target_marker.action = Marker.ADD
        self._local_target_marker.pose.position.x = target_x
        self._local_target_marker.pose.position.y = target_y
        self._local_target_marker.pose.position.z = 0.0
        self._local_target_marker.pose.orientation.x = 0.0
        self._local_target_marker.pose.orientation.y = 0.0
        self._local_target_marker.pose.orientation.z = 0.0
        self._local_target_marker.pose.orientation.w = 1.0
        self._local_target_marker.scale.x = marker_size
        self._local_target_marker.scale.y = marker_size
        self._local_target_marker.scale.z = marker_size
        self._local_target_marker.color.a = 0.0  # Add, but make invisible at first
        self._local_target_marker.color.r = 1.0
        self._local_target_marker.color.g = 0.0
        self._local_target_marker.color.b = 1.0

    # Transform point into odometry frame
    def transformOdom(self, point):
        try:
          # Get transform
          self._listener.listener().waitForTransform(self._odom_frame, point.header.frame_id,point.header.stamp, self._timeout)
          return self._listener.listener().transformPoint(self._odom_frame, point)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            Logger.logwarn('Failed to get the transformation\n   %s' % str(e))
            self._failed = True
            return None
        except Exception as e:
            Logger.logwarn('Failed to get the transformation due to unknown error\n    %s' % str(e) )
            self._failed = True
            return None

    # Transform point into map frame
    def transformMap(self, odometry):

        odom_position = PointStamped()
        odom_position.header = odometry.header
        odom_position.point  = odometry.pose.pose.position

        try:
          # Get transform
          self._listener.listener().waitForTransform(self._target_frame, odometry.header.frame_id, odometry.header.stamp, self._timeout)
          return self._listener.listener().transformPoint(self._target_frame, odom_position)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            Logger.logwarn('Failed to get the transformation to target_frame\n   %s' % str(e))
            self._failed = True
            return None
        except Exception as e:
            Logger.logwarn('Failed to get the transformation to target frame due to unknown error\n   %s' % str(e) )
            self._failed = True
            return None

    # Transform point into robot body frame
    def transformBody(self, point):
        try:
          # Get transform
          self._listener.listener().waitForTransform(self._robot_frame, point.header.frame_id,point.header.stamp, self._timeout)
          return self._listener.listener().transformPoint(self._robot_frame, point)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            Logger.logwarn('Failed to get the transformation\n   %s' % str(e))
            self._failed = True
            return None
        except:
            Logger.logwarn('Failed to get the transformation due to unknown error\n' )
            self._failed = True
            return None

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # If no outcome is returned, the state will stay active.

        if (self._done):
            # We have completed the state, and therefore must be blocked by autonomy level
            # Stop the robot, but and return the prior outcome
            ts = TwistStamped()
            ts.header.stamp = rospy.Time.now()
            self._pub.publish(self._cmd_topic, ts)
            userdata.target_point = self._target   # Set the user data for passing to next node
            return self._done

        # Check bumpers to see if we hit something
        if (self._sensor_sub.has_msg(self._sensor_topic)):
            # check the status of the bumpers
            sensors = self._sub.get_last_msg(self._sensor_topic)
            if ((sensors.bumps_wheeldrops > 0) or sensors.cliff_left or sensors.cliff_front_left or sensors.cliff_front_right or sensors.cliff_right):
                ts = TwistStamped()
                ts.header.stamp = rospy.Time.now()
                self._pub.publish(self._cmd_topic, ts)
                userdata.target_point = self._target   # Set the user data for passing to next node
                Logger.logwarn('%s  Bumper contact = %d  cliff: left=%d %d %d %d = right ' %
                        (self.name, sensors.bumps_wheeldrops, sensors.cliff_left, sensors.cliff_front_left,sensors.cliff_front_right, sensors.cliff_right))
                self._done = 'failed'
                return 'failed'


        # Get the latest odometry data
        if (self._sub.has_msg(self._odom_topic)):
            self._last_odom = self._sub.get_last_msg(self._odom_topic)

        # Update the current pose
        self._current_position = self.transformMap(self._last_odom)
        if (self._failed):
             userdata.target_point = self._target   # Set the user data for passing to next node
             self._done = 'failed'
             return 'failed'

        # Transform the target points into the current odometry frame
        self._target.header.stamp = self._last_odom.header.stamp
        local_target = self._target; #self.transformOdom(self._target)


        # If target point is withing lookahead distance then we are done
        dr = np.sqrt((local_target.point.x - self._current_position.point.x)**2 + (local_target.point.y - self._current_position.point.y)**2 )
        if (dr < self._lookahead):
            Logger.loginfo(' %s  Lookahead circle is past target - done!  target=(%f, %f, %f) robot=(%f,%f, %f)  dr=%f lookahead=%f ' %
                    (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                     self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                     dr,self._lookahead))
            userdata.target_point = self._target   # Set the user data for passing to next node
            self._done = 'done'
            return 'done'

        # Transform the prior target point into the current odometry frame
        self._prior_point.header.stamp = self._last_odom.header.stamp
        local_prior = self._prior_point#self.transformOdom(self._prior_point)
        if (self._failed):
             userdata.target_point = self._target   # Set the user data for passing to next node
             self._done = 'failed'
             return 'failed'

        # Assume we can go the desired velocity
        self._twist.twist.linear.x  = self._desired_velocity

        lookahead = None
        if (self._target_type == 'arc'):
            lookahead = self.calculateArcTwist(local_prior, local_target)
        else:
            lookahead = self.calculateLineTwist(local_prior, local_target)

        if (lookahead is None):
             # Did not get a lookahead point so we either failed, completed segment, or are deliberately holding the prior velocity (to recover from minor perturbations)
             if (self._done is not None):
                userdata.target_point = self._target   # Set the user data for passing to next node
             return self._done # return what was set (either 'failed' or 'done')

        # Sanity check the rotation rate
        if (math.fabs(self._twist.twist.angular.z) > self._max_rotation_rate):
            self._twist.twist.linear.x  = self._desired_velocity*self._max_rotation_rate/math.fabs(self._twist.twist.angular.z) # decrease the speed
            self._twist.twist.angular.z = math.copysign(self._max_rotation_rate, self._twist.twist.angular.z)

        # Normal operation - publish the latest calculated twist
        self._twist.header.stamp = rospy.Time.now()
        self._pub.publish(self._cmd_topic, self._twist)

        if (self._marker_pub):
            self._marker_pub.publish(self._marker_topic,self._reference_marker)
            self._marker_pub.publish(self._marker_topic,self._local_target_marker)
        return None

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        self._start_time = rospy.Time.now()
        self._done       = None  # reset the completion flag
        self._failed     = False # reset the failed flag

        self._prior_point    = userdata.prior_point

        if (self._marker_pub):
            self._marker.action  = Marker.MODIFY
            self._marker.color.a = 1.0
            self._marker.color.r = 0.0
            self._marker.color.g = 1.0 # Indicate this target is active
            self._marker.color.b = 0.0
            self. _marker_pub.publish(self._marker_topic,self._marker)

        #Logger.logdebug("  Moving toward  target=%f,%f  from position=%f,%f" %
        #        (self._target.point.x, self._target.point.y,
        #         self._current_position.point.x, self._current_position.point.y))

    def on_exit(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        elapsed_time = rospy.Time.now() - self._start_time
        userdata.target_point = self._target   # Set the user data for passing to next node

        #Logger.logdebug("  Spent %f seconds in this segment  target=%f,%f  position=%f,%f" %
        #        (elapsed_time.to_sec(),self._target.point.x, self._target.point.y,
        #         self._current_position.point.x, self._current_position.point.y))

        if (self._marker_pub):
            self._marker.color.a = 1.0 # Don't forget to set the alpha!
            self._marker.color.r = 0.8 # Indicate this target is no longer active
            self._marker.color.g = 0.0
            self._marker.color.b = 0.0
            self. _marker_pub.publish(self._marker_topic,self._marker)

    def on_start(self):


        # Wait for odometry message
        while (not self._odom_sub.has_msg(self._sensor_topic)):
            Logger.logwarn('Waiting for odometry message from the robot  ' )
            rospy.sleep(0.05)

        while (not self._sub.has_msg(self._odom_topic)):
            Logger.logwarn('Waiting for odometry message to become available from the robot ' )
            rospy.sleep(0.25)

        self._last_odom = self._sub.get_last_msg(self._odom_topic)
        self._odom_frame = self._last_odom.header.frame_id
        #Logger.loginfo('   odometry frame id <%s>' % (self._odom_frame))

        # Update the target transformation
        self._target.header.stamp = self._last_odom.header.stamp
        while (self.transformOdom(self._target) is None):
            Logger.logwarn('Waiting for tf transformations to odometry frame to become available from the robot ' )
            rospy.sleep(0.25)
            self._target.header.stamp = rospy.Time.now()

        while (self.transformMap(self._last_odom) is None):
            Logger.logwarn('Waiting for tf transformations to map frame become available from the robot ' )
            rospy.sleep(0.25)
            self._last_odom = self._sub.get_last_msg(self._odom_topic)

        self._current_position     = self.transformMap(self._last_odom)

        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        if (self._marker_pub):
            self._marker.action  = Marker.ADD
            self._marker.color.a = 1.0 # Set alpha otherwise the marker is invisible
            self._marker.color.r = 0.0
            self._marker.color.g = 0.0
            self._marker.color.b = 1.0 # Indicate this target is planned
            self. _marker_pub.publish(self._marker_topic,self._marker)
            self. _marker_pub.publish(self._marker_topic,self._reference_marker)
            self. _marker_pub.publish(self._marker_topic,self._local_target_marker)
            self._marker.action           = Marker.MODIFY
            self._reference_marker.action = Marker.MODIFY
            self._reference_marker.color.a = 1.0 # Set alpha so it will become visible on next publish
            self._local_target_marker.action = Marker.MODIFY
            self._local_target_marker.color.a = 1.0 # Set alpha so it will become visible on next publish

    # Attempt to recover the path for large error
    def recoverPath(self, local_target,local_prior,pv,qv):
        pp = pv.x*pv.x + pv.y*pv.y
        qq = qv.x*qv.x + qv.y*qv.y
        qp = -qv.x*pv.x - qv.y*pv.y # negate qv

        dp = qp/pp # fractional distance along the pv vector

        # Assume we steer toward the initial point
        control         = PointStamped()
        control.header  = local_prior.header
        control.point   = deepcopy(local_prior.point)

        if (dp > 1.0):
            # Steer toward the target point
            control = local_target
        elif (dp > 0.0):
            # Steer toward the closest point
            control.point.x = control.point.x + dp*pv.x # Control point in the odometry frame
            control.point.y = control.point.y + dp*pv.y # Control point in the odometry frame
            control.point.z = control.point.z + dp*pv.z # Control point in the odometry frame

        self._reference_marker.pose.position    = deepcopy( control.point )
        self._local_target_marker.pose.position = deepcopy( local_target.point )

        control_robot = self.transformBody(control)

        if (control_robot.point.x <= 0.001):
             control_robot = self.transformBody(local_target) # One last try
             if (control_robot.point.x <= 0.001):
                 dist = control_robot.point.x*control_robot.point.x + control_robot.point.y*control_robot.point.y;
                 if (dist > 2.5):
                    Logger.loginfo("recovery control point is behind the robot and far way   abort recovery!")
                    return False
                 else:
                    # Target is close enough - do a zero radius turn toward the target line until our closet point is ahead
                    self._twist.twist.linear.x   = 0.0
                    self._twist.twist.angular.z  = math.copysign(self._desired_velocity/0.13, control_robot.point.y)
                    return True
             else:
                 # Target is ahead - do a zero radius turn toward the target line until our closet point is ahead
                 self._twist.twist.linear.x   = 0.0
                 self._twist.twist.angular.z  = math.copysign(self._desired_velocity/0.13, control_robot.point.y)
                 return True

        # Assume lookahead tangent to the control point
        dc =  Vector3(control.point.x  - self._current_position.point.x, control.point.y  - self._current_position.point.y, 0.0)
        curvature = 2.0*control_robot.point.y/(dc.x*dc.x + dc.y*dc.y)
        if (np.isnan(curvature)):
            Logger.logerr("invalid curvature calculation   abort recovery!")
            return False

        self._twist.twist.angular.z  = curvature*self._desired_velocity
        return True

    # Method to calculate the lookahead point given line segment from prior to target
    def calculateLineTwist(self, local_prior, local_target):

        # Define a line segment from prior to the target (assume 2D)
        pv = Vector3(local_target.point.x - local_prior.point.x,                  local_target.point.y - local_prior.point.y,                  0.0)
        qv = Vector3(local_prior.point.x  - self._current_position.point.x, local_prior.point.y  - self._current_position.point.y, 0.0)

        # Find intersection of line segment with lookahead circle centered at the  robot
        a = pv.x*pv.x + pv.y*pv.y #
        b = 2.0*(qv.x*pv.x + qv.y*pv.y)
        c = (qv.x*qv.x + qv.y*qv.y) - self._lookahead*self._lookahead

        if (a < 0.001):
            Logger.logerr(' %s  Invalid prior and target for line  target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f ' %
                 (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                  local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                  self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                  pv.x, pv.y,qv.x,qv.y,a,b,c))
            self._done = 'failed'
            return None

        discrim = b*b - 4*a*c
        if (discrim < 0.0):
             # No intersection - this should not be unless bad start or localization perturbation
             if (self._recover_mode):
                 # Attempt to regain path
                 if (not self.recoverPath(local_target,local_prior,pv,qv)):
                     Logger.logwarn(' %s  Path recovery failed for line  target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                          (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                           local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                           self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                           pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                     self._done = 'failed'
                     return None

             else:
                Logger.logwarn(' %s  No path recovery - no intersection for line  target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                     (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                      local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                      self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                      pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._done = 'failed'
                return None
        else:
            # solve quadratic equation for intersection points
            sqd = math.sqrt(discrim)
            t1 = (-b - sqd)/(2*a)  # min value
            t2 = (-b + sqd)/(2*a)  # max value
            if (t2 < t1):
                Logger.logwarn(' %s  Say what!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._done = 'failed'
                return None

            if (t2 < 0.0):
                # all intersections are behind the segment - this should not be unless bad start or localization perturbation
                if (self._recover_mode):
                    # Attempt to regain path
                    if (not self.recoverPath(local_target,local_prior,pv,qv)):
                        Logger.logwarn(' %s  Path recovery failed with circle before segment target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                             (self.name, local_target.point.x,local_target.point.y,local_target.point.z,
                              local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                              self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                              pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                        self._done = 'failed'
                        return None
                elif (t2 > -0.1):
                    # likely due to localization perturbation
                    Logger.logwarn(' %s  Circle is before segment - continue prior motion! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                    return None
                else:
                    Logger.logwarn(' %s  Circle is before segment! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                    self._done = 'failed'
                    return None
            elif (t1 > 1.0):
                # all intersections are past the segment
                Logger.loginfo(' %s  Circle is past segment - done! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._done = 'done'
                return None
            elif (t1 < 0.0 and t2 > 1.0):
                # Segment is contained inside the lookahead circle
                Logger.loginfo(' %s  Circle contains segment - move along!: t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._done = 'done'
                return None
            elif (t2 > 1.0):
                # The lookahead circle extends beyond the target point - we are finished here
                Logger.loginfo(' %s  Circle extends past segment - done! t1=%f t2=%f sqd=%f target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  pv=(%f,%f) qv=(%f,%f) a=%f b=%f c=%f discrim=%f ' %
                        (self.name, t1, t2, sqd,
                         local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         pv.x, pv.y,qv.x,qv.y,a,b,c,discrim))
                self._done = 'done'
                return None
            else:
                # This is the normal case
                # Must be line segment
                control = deepcopy(local_prior)
                control.point.x = control.point.x + t2*pv.x # Control point in the odometry frame
                control.point.y = control.point.y + t2*pv.y # Control point in the odometry frame
                control.point.z = control.point.z + t2*pv.z # Control point in the odometry frame
                self._reference_marker.pose.position = control.point
                self._local_target_marker.pose.position = local_target.point

                control_robot = self.transformBody(control)

                curvature = 2.0*control_robot.point.y/(self._lookahead*self._lookahead)
                self._twist.twist.angular.z  = curvature*self._desired_velocity
                return control_robot

        return "Recovery"  # Must have be in a recovery mode

    # Method to calculate the lookahead point and twist given arc segment from prior to target
    def calculateArcTwist(self, local_prior, local_target):

        # Transform the relevant points into the odometry frame
        self._center.header.stamp = self._last_odom.header.stamp
        local_center = self._center #self.transformOdom(self._center)
        if (self._failed):
             self._done = 'failed'
             return None

        # Format for the equations
        xa = local_center.point.x
        ya = local_center.point.y
        ra = self._radius

        xr = self._current_position.point.x
        yr = self._current_position.point.y
        rl = self._lookahead


        # Calculate the discriminant used in x- and y- calculations
        xd  = ( - ((ra - rl)**2 - (xa - xr)**2 - (ya - yr)**2) * ((ra + rl)**2 - (xa - xr)**2 - (ya - yr)**2) * (ya - yr)**2) # discriminant for x solution

        yd  = ( - (ra**4 + (-rl**2 + (xa - xr)**2 + (ya - yr)**2)**2 - 2 * ra**2 * (rl**2 + (xa - xr)**2 + (ya - yr)**2)) * (ya - yr)**2)

        discrim = np.min((xd,yd))

        if (discrim < 0.0):
             # No intersection - this should not be unless bad start
             if (self._recover_mode):
                 # Attempt to regain path by driving toward the chord defining the arc
                 pv = Vector3(local_target.point.x - local_prior.point.x,                  local_target.point.y - local_prior.point.y,                  0.0)
                 qv = Vector3(local_prior.point.x  - self._current_position.point.x, local_prior.point.y  - self._current_position.point.y, 0.0)
                 if (not self.recoverPath(local_target,local_prior,pv,qv)):
                     Logger.logwarn(' Path recovery failed for arc  target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f) rrad=%f  center=(%f,%f) crad=%f discrim: xd=%f yd=%f  ' %
                          (local_target.point.x,local_target.point.y,local_target.point.z,
                           local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                           self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                           rl,xa,ya,ra, xd,yd))
                     self._done = 'failed'
                     return None

             else:
                Logger.logwarn(' No path recovery - no intersection for arc  target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  rrad=%f  center=(%f,%f) crad=%f  discrim: xd=%f yd=%f   ' %
                     (local_target.point.x,local_target.point.y,local_target.point.z,
                      local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                      self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                      rl,xa,ya,ra, xd,yd))
                self._done = 'failed'
                return None
        else:
            # solve quadratic equation for intersection points
            xp  = (1./(2 * ((xa - xr)**2 + (ya - yr)**2))) # x solution prefix

            x1 = xp*(rl**2 * (xa - xr) + ra**2 * (-xa + xr) + (xa + xr) * ((xa - xr)**2 + (ya - yr)**2) - np.sqrt(xd))
            x2 = xp*(rl**2 * (xa - xr) + ra**2 * (-xa + xr) + (xa + xr) * ((xa - xr)**2 + (ya - yr)**2) + np.sqrt(xd))

            y1 = (-ra**2 * (ya - yr)**2 + (xa - xr) * np.sqrt(yd) + (ya - yr) * (rl**2 * (ya - yr) + ((xa - xr)**2 + (ya - yr)**2) * (ya + yr)))/(2 * ((xa - xr)**2 + (ya - yr)**2) * (ya - yr))
            y2 = (-ra**2 * (ya - yr)**2 + (xr - xa) * np.sqrt(yd) + (ya - yr) * (rl**2 * (ya - yr) + ((xa - xr)**2 + (ya - yr)**2) * (ya + yr)))/(2 * ((xa - xr)**2 + (ya - yr)**2) * (ya - yr))

            # Choose the intersection closest to the target point
            # All vectors on circle should have same radius from center.  A*B = |A||B|Cos(theta)
            dt = np.array([local_target.point.x-xa, local_target.point.y-ya])
            dp = np.array([local_prior.point.x-xa,  local_prior.point.y-ya])
            d1 = np.array([x1-xa, y1-ya])
            d2 = np.array([x2-xa, y2-ya])

            ap = np.dot(dt,dp) # > implies smaller angle to target
            a1 = np.dot(dt,d1)
            a2 = np.dot(dt,d2)

            control = deepcopy(local_prior)
            if (ap > a1 and ap > a2):
                # Intersections must be before the prior point
                pv = Vector3(local_target.point.x - local_prior.point.x,                  local_target.point.y - local_prior.point.y,                  0.0)
                qv = Vector3(local_prior.point.x  - self._current_position.point.x, local_prior.point.y  - self._current_position.point.y, 0.0)
                if (self._recover_mode):
                    # Attempt to regain path by driving toward the chord defining the arc
                    if (not self.recoverPath(local_target,local_prior,pv,qv)):
                        Logger.logwarn(' Path recovery failed for arc target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)  center=(%f,%f)  pv=(%f,%f) qv=(%f,%f) discrim: xd=%f yd=%f   ' %
                             (local_target.point.x,local_target.point.y,local_target.point.z,
                              local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                              self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                              xa,ya,
                              pv.x, pv.y,qv.x,qv.y,xd,yd))
                        Logger.logwarn("  dt=(%f, %f)  dp=(%f, %f) d1=(%f, %f) d2=(%f, %f)  ap=%f a1=%f a2=%f" %(dt[0],dt[1],dp[0],dp[1],d1[0],d1[1],d2[0],d2[1],ap,a1,a2))
                        self._done = 'failed'
                        return None
                else:
                    # Check to see if we are close to prior in case of perturbation due to localization
                    dp1 = np.dot(dp,d1)
                    dp1 = dp1/(np.linalg.norm(dp)*np.linalg.norm(d1))
                    dp2 = np.dot(dp,d2)
                    dp2 = dp2/(np.linalg.norm(dp)*np.linalg.norm(d2))

                    dpm = np.max((dp1,dp2))

                    if (dpm > 0.9):
                       Logger.logwarn('Before prior but close, hold current motion!  target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)   center=(%f,%f)  pv=(%f,%f) qv=(%f,%f) discrim: xd=%f yd=%f ' %
                         (local_target.point.x,local_target.point.y,local_target.point.z,
                          local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                          self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                          xa,ya, pv.x, pv.y,qv.x,qv.y,xd,yd))
                       Logger.logwarn("  dt=(%f, %f)  dp=(%f, %f) d1=(%f, %f) d2=(%f, %f)  ap=%f a1=%f a2=%f dp1=%f dp2=%f (%f)" %(dt[0],dt[1],dp[0],dp[1],d1[0],d1[1],d2[0],d2[1],ap,a1,a2,dp1,dp2,dpm))
                       return None;
                    else:
                       Logger.logwarn(' No path recovery - no valid intersection for arc  target=(%f, %f, %f) prior=(%f, %f, %f) robot=(%f,%f, %f)   center=(%f,%f)  pv=(%f,%f) qv=(%f,%f) discrim: xd=%f yd=%f ' %
                        (local_target.point.x,local_target.point.y,local_target.point.z,
                         local_prior.point.x ,local_prior.point.y ,local_prior.point.z,
                         self._current_position.point.x, self._current_position.point.y, self._current_position.point.z,
                         xa,ya,
                         pv.x, pv.y,qv.x,qv.y,xd,yd))
                       Logger.logwarn("  dt=(%f, %f)  dp=(%f, %f) d1=(%f, %f) d2=(%f, %f)  ap=%f a1=%f a2=%f dp1=%f dp2=%f (%f)" %(dt[0],dt[1],dp[0],dp[1],d1[0],d1[1],d2[0],d2[1],ap,a1,a2,dp1,dp2,dpm))
                       self._done = 'failed'
                       return None
            elif (a1 > a2):
                # Use intersection 1
                control.point.x = x1
                control.point.y = y1

            else:
                # Use intersection 2
                control.point.x = x2
                control.point.y = y2

            self._reference_marker.pose.position    = deepcopy(control.point)
            self._local_target_marker.pose.position = deepcopy(local_target.point)

            control_robot = self.transformBody(control)

            curvature = 2.0*control_robot.point.y/(self._lookahead*self._lookahead)
            self._twist.twist.angular.z  = curvature*self._desired_velocity
            return control_robot

        return "Recovery"  # Must have be in a recovery mode
