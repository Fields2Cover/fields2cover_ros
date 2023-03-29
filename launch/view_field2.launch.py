import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   rviz_config = os.path.join(
      get_package_share_directory('fields2cover_ros'),
      'launch/',
      'demo.rviz'
      )

   field_path = os.path.join(
      get_package_share_directory('fields2cover_ros'),
      'data/',
      'example1.xml'
      )

   return LaunchDescription([
      Node(
         package='tf2_ros',
         executable='static_transform_publisher',
         name='static_transform_publisher',
         arguments=["0", "0", "0", "0.015", "0", "0", "map", "base_link"]
      ),
      Node(
         package='rviz2',
         executable='rviz2',
         name='rviz2',
         #arguments=['-d', rviz_config]
      ),
      Node(
         package='rqt_reconfigure',
         executable='rqt_reconfigure',
         name='rqt_reconfigure',
      ),
      Node(
         package='fields2cover_ros',
         executable='visualizer_node',
         name='visualizer',
         parameters=[{'field_file': field_path}]
      )
   ])
