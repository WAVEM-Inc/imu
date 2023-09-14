# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a Phidgets spatial in a component container."""
import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import ament_index_python.packages

def generate_launch_description():
    """Generate launch description with multiple components."""
    #config_directory = os.path.join(
    #    ament_index_python.packages.get_package_share_directory('phidgets_spatial'),
    #    'config')
    #params = os.path.join(config_directory, 'phidgets_spatial_param.yaml')
    #print('!!!')
    #print(params)
    container = ComposableNodeContainer(
            name='phidget_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_spatial',
                    plugin='phidgets::SpatialRosI',
                    name='phidgets_spatial',
                 ),
            ],
           # parameters=[params],
            output='both',
    )

    return launch.LaunchDescription([container])
