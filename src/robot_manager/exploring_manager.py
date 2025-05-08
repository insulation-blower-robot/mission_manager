import rospy
import subprocess
import actionlib
import roslaunch
from sensor_msgs.msg import RegionOfInterest
from my_package.msg import TrajectoryGoalAction, TrajectoryGoalGoal
from my_package.msg import ArmControlAction, ArmControlGoal
from robot_manager.node_manager import NodeManager

class ExploringManager:
    def __init__(self):
        self.method = None
        rospy.get_param("/exploration/method")
        self.node_manager = NodeManager("exploring")

    def start(self):
        self.node_manager.nodes['detectors'].start()
        self.node_manager.nodes['observation_map'].start()
        if self.method == "autonomous":
            self.node_manager.nodes['autonomous'].start()
        # elif method == "teleop":
        #     self.node_manager.nodes['teleop'].start()

    def stop(self):
        if self.node_manager.kill_all():
            rospy.loginfo("All nodes killed successfully.")

    def run(self):
        self.start()
        self.stop()
        rospy.loginfo("FillingManager done.")
