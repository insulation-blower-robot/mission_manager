import paramiko
import rospy
import rospkg
import roslaunch
import subprocess

class NodeWrapper:
    def __init__(self, node_name, launch_command):
        self.node_name = node_name
        self.launch_command = launch_command
        self.running = False

    def start(self):
        """Start the node."""
        rospy.loginfo(f"Starting node: {self.node_name}")
        # Here you would implement the logic to start the node
        self.running = True

    def kill(self):
        """Kill the node."""
        rospy.loginfo(f"Killing node: {self.node_name}")
        # Here you would implement the logic to kill the node
        self.running = False

    def restart(self):
        """Restart the node."""
        rospy.loginfo(f"Restarting node: {self.node_name}")
        self.kill()
        self.start()

class TmuxNode(NodeWrapper):
    def __init__(self, node_name, launch_command, ssh_host, ssh_user, local=False, log_dir="/tmp/ros_logs"):
        self.node_name = node_name
        self.launch_command = launch_command
        self.session_name = f"{node_name}_session"
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.log_dir = log_dir
        self.log_path = f"{self.log_dir}/{self.node_name}.log"
        self.running = False
        self.local = local

    def _execute_ssh_command(self, command):
        try:
            ssh = paramiko.SSHClient()
            ssh.load_system_host_keys()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            
            ssh.connect(self.ssh_host, username=self.ssh_user)
            stdin, stdout, stderr = ssh.exec_command(command)
            return stdout.read().decode('utf-8'), stderr.read().decode('utf-8')
        except Exception as e:
            print(f"SSH command failed: {e}")
            return "", str(e)

    def start(self):
        """Start the node in a tmux session, outputting to a log file."""
        rospy.loginfo(f"Starting node: {self.node_name} on {self.ssh_host}")

        if self.local:
            subprocess.run([
                "tmux", "new-session", "-d", "-s", self.session_name, self.launch_command
            ])
        else:        
            self._execute_ssh_command(f"mkdir -p {self.log_dir}")  # Make sure log dir exists
            command = (
                f"tmux new-session -d -s {self.session_name} "
                f"'bash -c \"{self.launch_command} |& tee {self.log_path}\"'"
            )
            output, error = self._execute_ssh_command(command)
            if error:
                print(f"Error starting node: {error}")
            else:
                print(f"Node started: {self.node_name}")
                self.running = True

    def kill(self):
        """Kill the tmux session for the node."""
        print(f"Killing node session: {self.node_name} on {self.ssh_host}")
        if self.local:
            subprocess.run(["tmux", "kill-session", "-t", self.session_name])
        else:
            command = f"tmux kill-session -t {self.session_name}"
            output, error = self._execute_ssh_command(command)
            if error:
                print(f"Error killing node: {error}")
            else:
                print(f"Node session killed: {self.node_name}")

    def restart(self):
        """Restart the tmux session."""
        print(f"Restarting node: {self.node_name} on {self.ssh_host}")
        self.kill()
        self.start()

    def get_status(self):
        """Check if the tmux session is running."""
        command = "tmux list-sessions -F '#{session_name}'"
        output, error = self._execute_ssh_command(command)
        if error:
            print(f"Error checking status: {error}")
            return False
        return self.session_name in output

    def read_log(self, num_lines=50):
        """Fetch the last lines of the log file from the remote machine."""
        command = f"tail -n {num_lines} {self.log_path}"
        output, error = self._execute_ssh_command(command)
        if error:
            print(f"Error reading log: {error}")
            return ""
        return output
    
import rospy
import roslaunch
import time

class RoslaunchNode(NodeWrapper):
    def __init__(self, node_name, launch_command):
        super().__init__(node_name, launch_command)

        self.launch = None
        self.running = False

        # Setup the roslaunch object during initialization
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # Parse launch_command into package and launchfile
        try:
            package, launchfile = self.launch_command.split()
        except ValueError:
            rospy.logerr(f"Invalid launch command format: {self.launch_command}")
            return

        launch_file_path = roslaunch.rlutil.resolve_launch_arguments([package, launchfile])[0]
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])

    def start(self):
        """Start the node using roslaunch."""
        if not self.running:
            rospy.loginfo(f"Starting node: {self.node_name}")
            self.launch.start()
            self.running = True
            time.sleep(1)  # Allow time to properly start

    def kill(self):
        """Kill the node."""
        if self.running:
            rospy.loginfo(f"Killing node: {self.node_name}")
            self.running = False

        if self.launch:
            self.launch.shutdown()


    def restart(self):
        """Restart the node."""
        rospy.loginfo(f"Restarting node: {self.node_name}")
        self.kill()
        self.start()
