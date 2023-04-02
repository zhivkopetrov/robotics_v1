from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
     ld = LaunchDescription()
     node_name = 'ur_control_bloom'
     config = os.path.join(
          get_package_share_directory(node_name),
          'config',
          'params.yaml'
     )

     print('[launch.py] - loading node ({0}) params from: ({1})'.format(node_name, config))

     node = Node(
          package = node_name,
          executable = node_name,
          output = 'screen',
          emulate_tty = True,
          parameters = [config]
     )

     ld.add_action(node)
     return ld