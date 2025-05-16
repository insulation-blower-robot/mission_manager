import rospy
import actionlib
from cavity_detection_msgs.msg import Roi
from mission_manager.msg import BaseTrajectoryProcessingAction, BaseTrajectoryProcessingGoal
from mission_manager.msg import ArmTrajectoryProcessingAction, ArmTrajectoryProcessingGoal
from mission_manager.node_manager import NodeManager
from actionlib_msgs.msg import GoalStatus

class FillingManager:
    def __init__(self, roi: Roi):
        self.roi = roi
        self.nodes = NodeManager("filling")
        self.traj_client = actionlib.SimpleActionClient('trajectory_generator', BaseTrajectoryProcessingAction)
        self.arm_client = actionlib.SimpleActionClient('arm_controller', ArmTrajectoryProcessingAction)
        self.traj_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.started = False

    def start_monitoring(self):

        rospy.set_param(f'/fill_speed_controller/cavity_dim', [-2.0, 0.0, self.roi.length, self.roi.width])
        rospy.set_param(f'/fill_speed_controller/channel_spacing', self.roi.spacing)
        rospy.set_param(f'/fill_speed_controller/target_fill_height', self.roi.height)

        self.nodes.start_all()
        rospy.loginfo("Insul monit and elevation mapping launched.")

    def send_goals(self):
        traj_goal = BaseTrajectoryProcessingGoal(roi=self.roi)
        arm_goal = ArmTrajectoryProcessingGoal(action="TrackOrientation", frame_id=self.roi.id, target_angle=0.0)
        self.traj_client.send_goal(traj_goal)
        self.arm_client.send_goal(arm_goal)
    
    def cancel(self):
        self.traj_client.cancel_goal()
        self.arm_client.cancel_goal()
        self.nodes.kill_all()
        self.started = False

    def start(self):
        if not self.started:
            self.start_monitoring()
            self.send_goals()
            self.started = True
        
    def cleanup(self):
        self.nodes.kill_all()
        self.arm_client.cancel_goal()

    def is_done(self):
        if not self.started:
            return False
        return self.traj_client.get_state() in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED]
    