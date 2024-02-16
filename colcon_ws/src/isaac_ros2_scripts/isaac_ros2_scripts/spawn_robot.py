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
        self.declare_parameter('x', 0.0)
        robot_x = self.get_parameter('x').get_parameter_value().double_value
        self.declare_parameter('y', 0.0)
        robot_y = self.get_parameter('y').get_parameter_value().double_value
        self.declare_parameter('z', 0.0)
        robot_z = self.get_parameter('z').get_parameter_value().double_value
        self.declare_parameter('R', 0.0)
        robot_roll = self.get_parameter('R').get_parameter_value().double_value
        self.declare_parameter('P', 0.0)
        robot_pitch = self.get_parameter('P').get_parameter_value().double_value
        self.declare_parameter('Y', 0.0)
        robot_yaw = self.get_parameter('Y').get_parameter_value().double_value
        self.declare_parameter('fixed', False)
        robot_fixed = self.get_parameter('fixed').get_parameter_value().bool_value

        self.sensor_proc = None

        spawn_command_path = os.path.join(
                    get_package_share_directory('isaac_ros2_scripts'), 'spawn_command.sh')
        temp_spawn_command_path = os.path.join("/tmp", 'spawn_command.sh')

        with open(spawn_command_path, encoding="utf-8") as f:
            data_lines = f.read()

            data_lines = data_lines.replace("URDF_PATH", urdf_path)
            data_lines = data_lines.replace("ROBOT_ROLL", str(robot_roll))
            data_lines = data_lines.replace("ROBOT_PITCH", str(robot_pitch))
            data_lines = data_lines.replace("ROBOT_YAW", str(robot_yaw))
            data_lines = data_lines.replace("ROBOT_X", str(robot_x))
            data_lines = data_lines.replace("ROBOT_Y", str(robot_y))
            data_lines = data_lines.replace("ROBOT_Z", str(robot_z))
            data_lines = data_lines.replace("ROBOT_FIXED", str(robot_fixed))

        with open(temp_spawn_command_path, mode="w", encoding="utf-8") as f:
            f.write(data_lines)

        command = ["bash", temp_spawn_command_path]
        print(command)
        self.get_logger().info("command start")
        self.sensor_proc = subprocess.Popen(command)
        self.sensor_proc.wait()
        if not self.sensor_proc.stdout == None:
            lines = self.sensor_proc.stdout.read()
            for line in lines:
                print(line)
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

