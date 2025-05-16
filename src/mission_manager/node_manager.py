import yaml
import rospy
import rospkg
from mission_manager.node_wrapper import TmuxNode, RoslaunchNode

class NodeManager:
    def __init__(self, name):
        self.name = name
        self.nodes = {}
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('mission_manager')
        self.load_nodes_from_config(f"{pkg_path}/config/node_info.yaml")
        self.fake = True

    def add_node(self, node: TmuxNode):
        self.nodes[node.node_name] = node

    def start_all(self):
        for node in self.nodes.values():
            if self.fake: 
                rospy.loginfo(f"Fake starting node: {node.node_name}")
            else:
                rospy.loginfo(f"Starting node: {node.node_name}")
                node.start()
            rospy.sleep(5)

    def kill_all(self):
        for node in self.nodes.values():
            if self.fake: 
                rospy.loginfo(f"Fake killing node: {node.node_name}")
            else:
                rospy.loginfo(f"Killing node: {node.node_name}")
                node.kill()

    def restart_node(self, node_name):
        for node_name in self.nodes:
            if self.fake: 
                rospy.loginfo(f"Fake restarting node: {node_name}")
            else:
                rospy.loginfo(f"Restarting node: {node_name}")
                self.nodes[node_name].restart()

    def monitor_logs(self, error_strings=None):
        """Check logs for specific errors.
        
        Args:
            error_strings (list of str): list of substrings to look for in the logs
        """
        if error_strings is None:
            error_strings = ["[ERROR]", "[FATAL]", "CRASH"]

        for node in self.nodes.values():
            log = node.read_log()
            for err in error_strings:
                if err in log:
                    print(f"Detected error in {node.node_name}: {err}")
    

    def load_nodes_from_config(self, config_path):
        with open(config_path, "r") as file:
            config = yaml.safe_load(file)

        for node_cfg in config.get(self.name, []):
            if node_cfg.get("method") == "roslaunch":
                node = RoslaunchNode(
                    node_name=node_cfg["name"],
                    launch_command=node_cfg["launch_command"]
                )                
                self.add_node(node)

            else:
                node = TmuxNode(
                    node_name=node_cfg["name"],
                    launch_command=node_cfg["launch_command"],
                    ssh_host=node_cfg["ssh_host"],
                    ssh_user=node_cfg["ssh_user"],
                    local=node_cfg["local"]
                )
                self.add_node(node)
