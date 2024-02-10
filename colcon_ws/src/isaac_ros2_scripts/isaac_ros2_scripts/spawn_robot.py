import os
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')

        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        if urdf_path == '':
            return
        
        self.sensor_proc = None

        spawn_command_path = os.path.join(
                    get_package_share_directory('isaac_ros2_scripts'), 'spawn_command.sh')
        temp_spawn_command_path = os.path.join("/tmp", 'spawn_command.sh')

        with open(spawn_command_path, encoding="utf-8") as f:
            data_lines = f.read()

            data_lines = data_lines.replace("URDF_PATH", urdf_path)

        with open(temp_spawn_command_path, mode="w", encoding="utf-8") as f:
            f.write(data_lines)

        command = ["bash", temp_spawn_command_path]
        print(command)
        self.get_logger().info("command start")
        self.sensor_proc = subprocess.Popen(command)
        self.sensor_proc.wait()
        self.get_logger().info("command end")

    def __del__(self):
        if not self.sensor_proc == None:
            if self.sensor_proc.poll() is None:
                killcmd = "kill {pid}".format(pid=self.sensor_proc.pid)
                subprocess.run(killcmd,shell=True)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()
    minimal_publisher.get_logger().info("node start")

    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
