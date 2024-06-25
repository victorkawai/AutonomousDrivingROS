#!/usr/bin/env python3

import rospy                            # Python client library.
import smach                            # State machine library.
import smach_ros                        # SMACH library extension for ROS.
import time                             # Handle time with sleep.
import random                           # Simulate the state machine in action.


class StartAtFirstCheckpoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized'])
    
    def execute(self, userdata):
        rospy.loginfo('Starting at first checkpoint')
        time.sleep(2)  # Simulate initialization time
        return 'initialized'

class FollowNextCheckpoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['checkpoint_reached', 'endpoint_reached', 'obstacle_detected', 'red_light_detected'])
        self.checkpoints = 5  # Total number of checkpoints
    
    def execute(self, userdata):
        rospy.loginfo(f'Following checkpoint. {self.checkpoints} checkpoints remaining.')
        time.sleep(3)  # Simulate time to reach next checkpoint
        
        if self.checkpoints == 0:
            return 'endpoint_reached'
        
        self.checkpoints -= 1
        
        # Simulate random events
        event = random.choices(['checkpoint_reached', 'obstacle_detected', 'red_light_detected', 'green_light_detected'], 
                               weights=[70, 10, 10, 10])[0]
        return event

class AvoidObstacle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_obstacle_detected'])
    
    def execute(self, userdata):
        rospy.loginfo('Avoiding obstacle')
        time.sleep(2)  # Simulate time to avoid obstacle
        return 'no_obstacle_detected'

class Break(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alternate_trajectory_calculated', 'velocity_is_0'])
    
    def execute(self, userdata):
        rospy.loginfo('Breaking')
        time.sleep(1)  # Simulate breaking time
        return random.choice(['alternate_trajectory_calculated', 'velocity_is_0'])

class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['green_light_detected'])
    
    def execute(self, userdata):
        rospy.loginfo('Waiting for green light')
        time.sleep(random.uniform(2, 5))  # Simulate random wait time for green light
        return 'green_light_detected'

def main():
    rospy.init_node('checkpoint_state_machine')

    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('START_AT_FIRST_CHECKPOINT', StartAtFirstCheckpoint(), 
                               transitions={'initialized':'FOLLOW_NEXT_CHECKPOINT'})
        
        smach.StateMachine.add('FOLLOW_NEXT_CHECKPOINT', FollowNextCheckpoint(), 
                               transitions={'checkpoint_reached':'FOLLOW_NEXT_CHECKPOINT',
                                            'obstacle_detected':'BREAK',
                                            'red_light_detected':'BREAK',
                                            'endpoint_reached':'finished'})
        
        smach.StateMachine.add('AVOID_OBSTACLE', AvoidObstacle(), 
                               transitions={'no_obstacle_detected':'FOLLOW_NEXT_CHECKPOINT'})
        
        smach.StateMachine.add('BREAK', Break(), 
                               transitions={'alternate_trajectory_calculated':'AVOID_OBSTACLE',
                                            'velocity_is_0':'WAIT'})
        
        smach.StateMachine.add('WAIT', Wait(), 
                               transitions={'green_light_detected':'FOLLOW_NEXT_CHECKPOINT'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()