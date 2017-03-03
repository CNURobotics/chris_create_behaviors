#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from chris_create_flexbe_states.create_timed_stop_state import CreateTimedStopState
from chris_create_flexbe_states.create_timed_twist_state import CreateTimedTwistState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 15 2016
@author: David Conner
'''
class CreateTimedTwistTestSM(Behavior):
    '''
    Test of the CreateTimedTwistState that sends constant twist to the iRobot Create node
    '''


    def __init__(self):
        super(CreateTimedTwistTestSM, self).__init__()
        self.name = 'CreateTimedTwistTest'

        # parameters of this behavior

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:909 y:103, x:830 y:367
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

		# [/MANUAL_CREATE]


        with _state_machine:
            # x:105 y:53
            OperatableStateMachine.add('Entry',
                                        LogState(text="Entering the create twist demo", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'Twist1'},
                                        autonomy={'done': Autonomy.Off})

            # x:577 y:345
            OperatableStateMachine.add('EmergencyStop',
                                        CreateTimedStopState(timeout=0.2, cmd_topic='/create_node/cmd_vel', odom_topic='/create_node/odom'),
                                        transitions={'done': 'failed', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:577 y:145
            OperatableStateMachine.add('Stop',
                                        CreateTimedStopState(timeout=0.24, cmd_topic='/create_node/cmd_vel', odom_topic='/create_node/odom'),
                                        transitions={'done': 'Exit', 'failed': 'EmergencyStop'},
                                        autonomy={'done': Autonomy.High, 'failed': Autonomy.Off})

            # x:756 y:65
            OperatableStateMachine.add('Exit',
                                        LogState(text="Exiting the create twist demo", severity=Logger.REPORT_HINT),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.High})

            # x:73 y:166
            OperatableStateMachine.add('Twist1',
                                        CreateTimedTwistState(target_time=10.0, velocity=0.25, rotation_rate=0.05, cmd_topic='/create_node/cmd_vel', sensor_topic='/create_node/sensor_state'),
                                        transitions={'done': 'Stop', 'failed': 'EmergencyStop'},
                                        autonomy={'done': Autonomy.Low, 'failed': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

	# [/MANUAL_FUNC]
