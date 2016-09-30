#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_pure_pursuit_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from chris_create_flexbe_states.create_waypoint_state import CreateWaypointState
from chris_create_flexbe_states.create_pure_pursuit_state import CreatePurePursuitState
from chris_create_flexbe_states.create_timed_stop_state import CreateTimedStopState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 17 2016
@author: David Conner
'''
class pure_pursuit_testSM(Behavior):
	'''
	Test of the purepursuit system
	'''


	def __init__(self):
		super(pure_pursuit_testSM, self).__init__()
		self.name = 'pure_pursuit_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:549 y:446, x:1245 y:488
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('StartingPoint',
										CreateWaypointState(target_frame='map', target_x=0.0, target_y=0.0, marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint1'},
										autonomy={'done': Autonomy.High},
										remapping={'target_point': 'starting_point'})

			# x:228 y:41
			OperatableStateMachine.add('Waypoint1',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=0.3096, target_y=0, target_type='line', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0.0, center_y=0.0, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint2', 'failed': 'RecoverStart'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'prior_point': 'starting_point', 'target_point': 'waypoint1'})

			# x:940 y:484
			OperatableStateMachine.add('Stop',
										CreateTimedStopState(timeout=0.4, cmd_topic='/create_node/cmd_vel', odom_topic='/create_node/odom'),
										transitions={'done': 'failed', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off})

			# x:503 y:284
			OperatableStateMachine.add('EmergencyStop',
										CreateTimedStopState(timeout=0.2, cmd_topic='/create_node/cmd_vel', odom_topic='/create_node/odom'),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.High})

			# x:461 y:38
			OperatableStateMachine.add('Waypoint2',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=0.6096, target_y=0, target_type='line', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0.0, center_y=0.0, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint3', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'waypoint1', 'target_point': 'waypoint2'})

			# x:142 y:256
			OperatableStateMachine.add('RecoverStart',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=0.3096, target_y=0, target_type='line', lookahead_distance=0.5, timeout=0.08, recover_mode=True, center_x=0.0, center_y=0.0, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'RecoverTransition', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'prior_point': 'starting_point', 'target_point': 'target_point'})

			# x:335 y:137
			OperatableStateMachine.add('RecoverTransition',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=0.3096, target_y=0.0, target_type='line', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0.0, center_y=0.0, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint2', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'starting_point', 'target_point': 'target_point'})

			# x:942 y:408
			OperatableStateMachine.add('FinePosition',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=-0.6096, target_y=-3.048, target_type='arc', lookahead_distance=0.05, timeout=0.08, recover_mode=False, center_x=0, center_y=-3.048, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Stop', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'waypoint6', 'target_point': 'final_point'})

			# x:718 y:44
			OperatableStateMachine.add('Waypoint3',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=1.2192, target_y=-0.6096, target_type='arc', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0.6096, center_y=-0.6096, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint4', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'waypoint2', 'target_point': 'waypoint3'})

			# x:960 y:60
			OperatableStateMachine.add('Waypoint4',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=1.2192, target_y=-1.8288, target_type='line', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0.0, center_y=0.0, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint5', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'waypoint3', 'target_point': 'waypoint4'})

			# x:1127 y:104
			OperatableStateMachine.add('Waypoint5',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=0.6096, target_y=-2.4384, target_type='arc', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0.6096, center_y=-1.8288, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint6', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'waypoint4', 'target_point': 'waypoint5'})

			# x:1165 y:192
			OperatableStateMachine.add('Waypoint6',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=0.0, target_y=-2.4384, target_type='line', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0.0, center_y=-2.4384, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'Waypoint7', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'waypoint5', 'target_point': 'waypoint6'})

			# x:1168 y:348
			OperatableStateMachine.add('Waypoint7',
										CreatePurePursuitState(desired_velocity=0.2, max_rotation_rate=10.0, target_frame='map', target_x=-0.6096, target_y=-3.048, target_type='arc', lookahead_distance=0.25, timeout=0.08, recover_mode=False, center_x=0, center_y=-3.048, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state', odometry_topic='/create_node/odom', marker_topic='/pure_pursuit_marker', marker_size=0.05),
										transitions={'done': 'FinePosition', 'failed': 'EmergencyStop'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prior_point': 'waypoint6', 'target_point': 'waypoint7'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
