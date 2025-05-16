#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, TriggerResponse # Example for checking status externally
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cavity_detection_api.api import *
from cavity_detection_msgs.msg import Roi
from mission_manager.srv import Trigger, TriggerResponse
from mission_manager.msg import Context
from mission_manager.node_manager import NodeManager
from mission_manager.filling_manager import FillingManager


class SMContext:
    def __init__(self):
        self.current_state = "STARTUP"
        self.last_state = None
        self.status_string = "Not started."
        self.current_target = None
        self.exploration_progress = 0
        self.start_confirmed = False
        self.exploration_confirmed = False
        self.exploration_estop = False
        self.target_confirmed = False
        self.start_filling_confirmed = False
        self.target_filled_confirmed = False
        self.manual_override_requested = False
        self.node_manager = None
        self.exporation_manager = None
        self.filling_manager = None

        rospy.Service('/smach_gui_confirmation', Trigger, self.handle_trigger)
        rospy.Timer(rospy.Duration(1.0), self.publish_context)
        self.context_pub = rospy.Publisher('/smach_context', Context, queue_size=10)

    def add_target(self, target_id):
        self.current_target = target_id
        self.target_confirmed = False
        self.start_filling_confirmed = False
        self.target_filled_confirmed = False

    def publish_context(self, event):        
        # Publish the context message
        context_msg = Context()
        context_msg.current_state = self.current_state
        context_msg.status_string = self.status_string                                                                                      # string current_state
        context_msg.current_target = "None" if self.current_target is None else self.current_target                                               # string current_target
        context_msg.exploration_progress = self.exploration_progress # int16 exploration_progress
        context_msg.start_confirmed = self.start_confirmed                                                                                      # bool start_confirmed                                                                        
        context_msg.exploration_confirmed = self.exploration_confirmed                                                                       # bool exploration_confirmed
        context_msg.exploration_estop = self.exploration_estop                                                                              # bool exploration_estop
        context_msg.target_confirmed = self.target_confirmed
        context_msg.start_filling_confirmed = self.start_filling_confirmed
        context_msg.target_filled_confirmed = self.start_filling_confirmed
        context_msg.manual_override_requested = self.manual_override_requested                                                                      # bool manual_override_requested
        self.context_pub.publish(context_msg)
    


    def handle_trigger(self, req):
        if req.button == "START EXPLORING":
            if self.current_state != "EXPLORING":
                rospy.loginfo("State machine is not in EXPLORING state.")
                return TriggerResponse(success=False, message="State machine is not in EXPLORING state.")
            else:
                rospy.loginfo("Starting exploration.")
                self.start_confirmed = True
                self.exploration_progress = 0
                return TriggerResponse(success=True, message="Starting exploration.")        
        elif req.button == "CONFIRM EXPLORING":
            if self.current_state != "EXPLORING":
                rospy.loginfo("State machine is not in EXPLORING state.")
                return TriggerResponse(success=False, message="State machine is not in EXPLORING state.")
            elif self.exploration_progress < 100:
                rospy.loginfo("Robot is not done exploring.")
                return TriggerResponse(success=False, message="Robot is not done exploring.")
            else:
                rospy.loginfo("Robot is done exploring.")
                self.exploration_confirmed = True
                return TriggerResponse(success=True, message="Robot is done exploring.")
        elif req.target != self.current_target:
                return TriggerResponse(success=False, message="Target mismatch! Not proceeding.")
        elif req.button == "CONFIRM TARGET":
            if self.current_state != "WAITING_FOR_TARGET":
                rospy.loginfo("State machine is not in WAITING_FOR_TARGET state.")
                return TriggerResponse(success=False, message="State machine is not in WAITING_FOR_TARGET state.")
            else:
                rospy.loginfo("Target confirmed.")
                self.target_confirmed = True
                return TriggerResponse(success=True, message="Target confirmed.")
        elif req.button == "CONFIRM BLOWING":
            if self.current_state != "FILLING_TARGET":
                rospy.loginfo("State machine is not in FILLING_TARGET state.")
                return TriggerResponse(success=False, message="State machine is not in FILLING_TARGET state.")
            else:
                rospy.loginfo("Filling operation started.")
                self.start_filling_confirmed = True
                return TriggerResponse(success=True, message="Filling operation started.")
        elif req.button == "CONFIRM FILLED":
            if self.current_state != "FILLING_TARGET":
                rospy.loginfo("State machine is not in FILLING_TARGET state.")
                return TriggerResponse(success=False, message="State machine is not in FILLING_TARGET state.")
            else:
                rospy.loginfo("Filling operation completed.")
                self.target_filled_confirmed = True
                return TriggerResponse(success=True, message="Filling operation completed.")
        elif req.button == "CONFIRM MANUAL OVERRIDE":
                rospy.loginfo("Manual override requested.")
                self.manual_override_requested = True
                return TriggerResponse(success=True, message="Manual override requested.")


class SMState(smach.State):
    def __init__(self, context):
        smach.State.__init__(self)
        self.context = context
        self.name = "STATE"

    def execute(self, userdata):
        # Update the current state in the context
        self.on_enter()
        return self.exit_to(self.name)
    
    def on_enter(self):
        # Called when entering the state
        if self.context.current_state != self.name:
            rospy.loginfo(f"Entering state: {self.name}")
            self.context.last_state = self.context.current_state
            self.context.current_state = self.name
    
    def exit_to(self, next_state):
        # Called when exiting the state
        if next_state != self.name:
            rospy.loginfo(f"Exiting state: {self.name}")
        return next_state
    
    def stay(self):
        # Called when staying in the state
        return self.name
                        
class Startup(SMState):
    def __init__(self, context):
        smach.State.__init__(self, input_keys=[], 
                             outcomes=['STARTUP', 'EXPLORING'],
                             output_keys=['explore_status'])
        self.context = context
        self.context.node_manager = NodeManager("init")
        self.started = False
        self.name = "STARTUP"

    def execute(self, userdata):
        if not self.started:
            self.context.node_manager.start_all()
            rospy.loginfo("All nodes started.")
            self.context.status_string = "All nodes started."
            return 'EXPLORING'

class Exploring(SMState):
    def __init__(self, context):
        smach.State.__init__(self, input_keys=['explore_status', 'end_exploration_flag'], 
                             outcomes=['EXPLORING', 'WAITING', 'manual_override'],
                             output_keys=['explore_status'])
        self.context = context
        self.started = False
        self.node_manager = NodeManager("exploring")
        self.context.exploration_manager = self.node_manager
        self.progress = 0
        self.name = "EXPLORING"


    def execute(self, userdata):
        self.on_enter()
        if self.preempt_requested():
            rospy.loginfo("Preemption requested!")
            if self.started:
                self.node_manager.kill_all()
            self.service_preempt()
            return 'manual_override'
        
        if not self.context.start_confirmed:
            # Wait for confirmation to start exploration
            if self.context.status_string != "Waiting for confirmation to start exploration.":
                self.context.status_string = "Waiting for confirmation to start exploration."
            rospy.sleep(0.1)
            return self.stay()
        
        if not self.started:
            # Start exploration nodes
            self.node_manager.start_all()
            self.started = True

        self.context.status_string = "Exploring..."

        # Listen for exploration confirmation
        if self.context.exploration_progress >= 100 and self.context.exploration_confirmed:
            rospy.loginfo("Done exploring!")
            self.context.status_string = "Done exploring!"
            return self.exit_to('WAITING')
        
        else:
            # Simulate exploration progress
            if self.progress < 100:
                self.progress += 20 
                self.context.exploration_progress = self.progress
            rospy.sleep(0.1)
            return self.stay()

class WaitingForTarget(SMState):
    def __init__(self, context):
        smach.State.__init__(self, 
                             outcomes=['WAITING', 'MOVING','no_targets', 'manual_override'],
                             output_keys=['target_details'])
        self.context = context
        self.target_details = None
        self.name = "WAITING"

    def execute(self, userdata):
        self.on_enter()
        if self.preempt_requested():
            rospy.loginfo("Preemption requested!")
            self.service_preempt()
            return 'manual_override'
        if not self.target_details:
            rospy.loginfo('Finding next target...')
            target_details = get_nearest_roi()
            if target_details is None:
                rospy.loginfo("No targets found.")
                return 'no_targets'
            else:
                userdata.target_details = target_details
                self.context.add_target(target_details.id)
                rospy.loginfo(f"Target found: {self.target_details}")
        if self.context.target_confirmed:
            rospy.loginfo(f"Target confirmed: {self.target_details}")
            return self.exit_to('MOVING')
        else:
            return self.stay()

class FillingTarget(SMState):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['FILLING', 'WAITING', 'manual_override'],
                             input_keys=['target_details'])
        self.context = context
        self.progress = 0
        self.filling_manager = None
        self.done = False
        self.name = "FILLING"

    def execute(self, userdata):
        self.on_enter()

        if self.preempt_requested():
            rospy.loginfo("Preemption requested!")
            self.service_preempt()
            return 'manual_override'

        if not self.context.start_filling_confirmed:
            rospy.loginfo("Waiting for confirmation to start filling.")
            self.context.status_string = "Waiting for confirmation to start filling."
            return self.stay()

        if self.filling_manager is None:
            self.filling_manager = FillingManager(userdata.target_details)
            self.context.filling_manager = self.filling_manager 
            self.filling_manager.start()

        if not self.filling_manager.is_done():
            self.context.status_string = "Filling in progress..."
            return self.stay()

        if not self.context.target_filled_confirmed:
            rospy.loginfo("Filling operation completed. Waiting for confirmation.")
            self.context.status_string = "Filling operation completed. Waiting for confirmation."
            return self.stay()

        # If we have received confirmation that the target is filled
        self.context.status_string = "Filling operation completed. Finding next target."
        return self.exit_to('WAITING')

def create_nav_goal_cb(userdata, goal):
    cavity_id = userdata.target_details.id
    rospy.loginfo(f"Creating navigation goal for cavity ID: {cavity_id}")
    goal.target_pose.header.frame_id = cavity_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = -1.0
    goal.target_pose.pose.position.y = 0.5
    goal.target_pose.pose.orientation.w = 1.0
    return goal

def no_log(msg):
    pass

# --- Main Function ---
def main():
    rospy.init_node('robot_manager_smach')
    rospy.loginfo("Task Manager Smach Node Started")

    # Create the top level SMACH state machine
    sm = smach.StateMachine(outcomes=['task_complete', 'task_failed', 'task_preempted'])
    sm.userdata.target_details = {"id": "horiz_roi_0", "location": "room_1"}
    sm.userdata.explore_status = "Not started"
    context = SMContext()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STARTUP', Startup(context),
                               transitions={'STARTUP':'STARTUP',
                                            'EXPLORING':'EXPLORING'})
        smach.StateMachine.add('EXPLORING', Exploring(context),
                               transitions={'WAITING':'WAITING',
                                            'EXPLORING':'EXPLORING',
                                            'manual_override':'task_preempted'},
                               remapping={'explore_status':'explore_status'}) # Pass status out

        smach.StateMachine.add('WAITING', WaitingForTarget(context),
                               transitions={'MOVING':'MOVING', # Go to moving if target confirmed
                                            'WAITING':'WAITING', # Wait for confirmation
                                            'no_targets':'task_complete',
                                            'manual_override':'task_preempted'}) 
        
        smach.StateMachine.add('MOVING', smach_ros.SimpleActionState('move_base',MoveBaseAction,        
                                                                                goal_cb=create_nav_goal_cb,
                                                                                input_keys=['target_details']),
                               transitions={'succeeded':'FILLING',
                                            'aborted':'EXPLORING', # Go back to exploring on failure
                                            'preempted':'task_preempted'},
                               remapping={'target_details':'target_details'}) # Pass target in

        smach.StateMachine.add('FILLING', FillingTarget(context),
                               transitions={'WAITING':'WAITING', # Go back to exploring after filling
                                            'FILLING':'FILLING', # Wait for filling to finish
                                            'manual_override':'task_preempted'},
                               remapping={'target_details':'target_details'}) # Pass target in

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('robot_manager_server', sm, '/SM_ROOT')
    sis.start()

    smach.set_loggers(no_log, no_log, smach.logwarn, smach.logerr)  

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.loginfo(f"State machine execution finished with outcome: {outcome}")

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()