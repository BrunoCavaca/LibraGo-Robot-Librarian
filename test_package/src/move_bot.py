#!/usr/bin/env python3
import rospy
import smach
import sys

from common.mediator import Mediator
from phase_1.set_initial_position import SetRobot
from phase_1.nlp import RetrieveBooks
from phase_1.listener import Listener
from phase_1.greet import Greet
from phase_2.set_robot_alignment import SetRobotAlignment
from phase_2.detect_books import DetectBooks
from phase_2.detection_rerun import DetectBookReRun
from phase_3.begin_pickup import BeginPickup
from phase_3.detect_aruco import DetectAruco
from phase_3.reach_gripper import ReachGripper
from phase_4.move_to_desk import ReturnToDesk
from phase_4.drop_book import DropBook
from phase_4.goodbye import Goodbye

def stateMachine(mediator,communicationType):
    # Define the state machine with possible outcomes
    sm = smach.StateMachine(outcomes=['listenSuccess', 'navigationSuccess', 'failure'])

    sm.userdata.textToProcess = ""

    with sm:
        smach.StateMachine.add('SET_BASE_POSITION', SetRobot(mediator),
                               transitions={'moveSuccess': 'INIT_GREETING',
                                            'moveFailure': 'SET_BASE_POSITION'})

        smach.StateMachine.add('INIT_GREETING', Greet(),
                               transitions={'greetSuccess': 'INIT_LISTENING',
                                            'greetFailure': 'INIT_GREETING'})
        # Adding the Listening state
        smach.StateMachine.add('INIT_LISTENING', Listener(mediator,communicationType),
                               transitions={'listenSuccess': 'INIT_NLP',
                                            'listenFailure': 'INIT_LISTENING'},
                               )  # Stay in the same state on failure

        smach.StateMachine.add('INIT_NLP', RetrieveBooks(mediator),
                               transitions={'listenSuccess' : 'INIT_LISTENING',
                                            'bookObtained' : 'NAVIGATE_AND_DETECT', #_AND_DETECT
                                            'listenFailure': 'INIT_GREETING'},     
                               )

        concurrent_nav_and_detection = smach.Concurrence(outcomes=['found','notFound'],
                                                         default_outcome= 'notFound',
                                                         outcome_map={'notFound':
                                                                      {'DETECT_BOOKS':'detectFailure'},
                                                                      'found':{'INIT_NAVIGATION':'navigationSuccess',
                                                                               'DETECT_BOOKS':'detectSuccess'}},
                                                        )
        
        with concurrent_nav_and_detection:
            concurrent_nav_and_detection.userdata.textToProcess = sm.userdata.textToProcess
            smach.Concurrence.add('INIT_NAVIGATION', SetRobotAlignment(mediator))
            smach.Concurrence.add('DETECT_BOOKS', DetectBooks(mediator))
        
        smach.StateMachine.add('NAVIGATE_AND_DETECT',concurrent_nav_and_detection,
                               transitions={'found':'NAVIGATE_TO_DETECTION_POINT',
                                            'notFound':'MOVE_TO_DESK'}) #change found to set to picking

        smach.StateMachine.add('NAVIGATE_TO_DETECTION_POINT',BeginPickup(mediator),
                                        transitions={'navigateSuccess': 'BOOK_DETECTION_RERUN',
                                            'navigateFailure': 'failure'})
    
        smach.StateMachine.add('DETECT_ARUCO',DetectAruco(mediator),
                                    transitions={'detectSuccess': 'REACH_GRIPPER',
                                        'detectFailure': 'failure',
                                        'greaterThanOneDetection':'BOOK_DETECTION_RERUN'})
        
        smach.StateMachine.add('BOOK_DETECTION_RERUN',DetectBookReRun(mediator),
                                    transitions={'detectSuccess': 'REACH_GRIPPER',
                                        'detectFailure': 'NAVIGATE_TO_DETECTION_POINT',})
        
        smach.StateMachine.add('REACH_GRIPPER',ReachGripper(mediator),
        transitions={'gripperSuccess':'MOVE_TO_DESK',
                     'gripperFailure':'failure'})

        smach.StateMachine.add('MOVE_TO_DESK',ReturnToDesk(mediator),
        transitions={'moveSuccess':'DROP_BOOK',
                     'moveFailure':'failure'})

        smach.StateMachine.add('DROP_BOOK',DropBook(mediator),
                            transitions={'gripperSuccess':'GOODBYE',
                                        'gripperFailure':'GOODBYE'})

        smach.StateMachine.add('GOODBYE',Goodbye(mediator),
                            transitions={'goodbyeSuccess':'INIT_GREETING'})   
    
    
        # Add Goodbye Message
    # Execute the state machine
    sm.execute()


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('begin_state_machine')
    
    # Retrieve the command-line arguments passed to the node
    args = rospy.myargv()

    # Check if there is at least one argument after the script name
    if len(args) < 2:
        rospy.logerr("ERROR: No communication type argument provided. Exiting...")
        sys.exit(1)  # Exit the program with a non-zero status

    # Extract the communication type argument
    communication_type = args[1]
    rospy.loginfo(f"Communication type: {communication_type}")
    
    # Instantiate the Mediator
    med = Mediator()
    
    # Execute the state machine
    stateMachine(med,communication_type)  
    rospy.spin()  # Keep the node alive