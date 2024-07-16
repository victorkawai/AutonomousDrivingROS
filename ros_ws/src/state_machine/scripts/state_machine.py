#!/usr/bin/env python3
 
import rospy                            # Python client library.
import smach                            # State machine library.
import smach_ros                        # SMACH library extension for ROS.
import time                             # Handle time with sleep.
import threading


# Define state Perception
class Perception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_detected', 'traffic_light_red', 'clear'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Perception')
        # Simulate perception processing time
        time.sleep(5)
        # Add example logic for testing
        if self.counter == 0:
            self.counter += 1
            rospy.loginfo('Obstacle detected')
            return 'obstacle_detected'
        elif self.counter == 1:
            self.counter += 1
            rospy.loginfo('Traffic light is red')
            return 'traffic_light_red'
        else:
            self.counter = 0
            rospy.loginfo('Road is clear')
            return 'clear'

# Define state PathPlanning
class PathPlanning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['checkpoint_updated'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PathPlanning')
        # Simulate path planning processing time
        time.sleep(7)
        rospy.loginfo('Checkpoint updated')
        return 'checkpoint_updated'

# Define state TrajectoryPlanning
class TrajectoryPlanning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['path_provided'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TrajectoryPlanning')
        # Simulate trajectory planning processing time
        time.sleep(7)
        rospy.loginfo('Path provided to next checkpoint')
        return 'path_provided'

# Define state CarControl
class CarControl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['executed'])
        self.red_light_duration = 5  # Duration to wait at red light

    def execute(self, userdata):
        rospy.loginfo('Executing state CarControl')
        # Simulate car control processing time
        time.sleep(7)
        rospy.loginfo('Car control executed')
        return 'executed'

# Define state Dummy
class Dummy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dummy_done'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state dummy')
        return 'dummy_done'
    
# Gets called when ANY child state terminates
def child_term_cb(outcome_map):

    # Also terminate MAIN if PERCEPTION finds an obstacle or a red light.
    if outcome_map['PERCEPTION'] == 'obstacle_detected' or \
                            outcome_map['PERCEPTION'] == 'traffic_light_red':
        return True

    # In all other cases, just keep running and don't terminate anything.
    return False

def main():
    rospy.init_node('autonomous_driving_state_machine')

    # Create a StateMachine for the perception pipeline
    sm_perception = smach.StateMachine(outcomes=['obstacle_detected', 'traffic_light_red', 'continue'])

    with sm_perception:
        smach.StateMachine.add('PERCEPTION', Perception(),
                               transitions={'obstacle_detected': 'obstacle_detected',
                                            'traffic_light_red': 'traffic_light_red',
                                            'clear': 'PERCEPTION'})

    # Create a StateMachine for the main sequence of operations
    sm_main = smach.StateMachine(outcomes=['checkpoint_reached'])

    with sm_main:
        smach.StateMachine.add('PATH_PLANNING', PathPlanning(),
                               transitions={'checkpoint_updated': 'TRAJECTORY_PLANNING'})
        smach.StateMachine.add('TRAJECTORY_PLANNING', TrajectoryPlanning(),
                               transitions={'path_provided': 'CAR_CONTROL'})
        smach.StateMachine.add('CAR_CONTROL', CarControl(),
                               transitions={'executed': 'checkpoint_reached'})

    # Create the top-level Concurrence state to run Perception concurrently with the main sequence
    top_concurrence = smach.Concurrence(
        outcomes=['obstacle_detected', 'traffic_light_red', 'finished_main'],
        default_outcome='finished_main',
        child_termination_cb=child_term_cb,
        outcome_map={
            'obstacle_detected': {'PERCEPTION': 'obstacle_detected'},
            'traffic_light_red': {'PERCEPTION': 'traffic_light_red'},
            'finished_main': {'MAIN': 'checkpoint_reached'}
        }
    )

    with top_concurrence:
        smach.Concurrence.add('PERCEPTION', sm_perception)
        smach.Concurrence.add('MAIN', sm_main)

    # Create the main state machine
    sm = smach.StateMachine(outcomes=['path_completed'])

    with sm:
        smach.StateMachine.add('TOP_CONCURRENCE', top_concurrence,
                               transitions={'obstacle_detected': 'CAR_CONTROL_OBSTACLE',
                                            'traffic_light_red': 'CAR_CONTROL_TRAFFIC_LIGHT',
                                            'finished_main': 'TOP_CONCURRENCE'})
        smach.StateMachine.add('CAR_CONTROL_OBSTACLE', CarControl(),
                               transitions={'executed': 'TOP_CONCURRENCE'},
                               remapping={'traffic_light_red': 'False'})
        smach.StateMachine.add('CAR_CONTROL_TRAFFIC_LIGHT', CarControl(),
                               transitions={'executed': 'TOP_CONCURRENCE'},
                               remapping={'traffic_light_red': 'True'})

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