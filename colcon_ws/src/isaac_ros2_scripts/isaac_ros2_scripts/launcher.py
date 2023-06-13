import os
import mmap
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
        
        self.proc = None
        
        python_script = os.path.join(
                    get_package_share_directory('isaac_ros2_scripts'), 'isaac_python.sh')
        start_script = os.path.join(
                    get_package_share_directory('isaac_ros2_scripts'), 'start_sim.py')
        command = ["bash", python_script, start_script, 
                    "-p", urdf_path]
        print(command)
        self.proc = subprocess.Popen(command)        
    
        timer_period = 0.01  # sec
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __del__(self):
        if not self.proc == None:
            if self.proc.poll() is None:
                killcmd = "kill {pid}".format(pid=self.proc.pid)
                subprocess.run(killcmd,shell=True)
            
    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

