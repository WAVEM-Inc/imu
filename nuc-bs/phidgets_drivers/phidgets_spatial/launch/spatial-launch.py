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
import launch
import launch_ros.actions
import ament_index_python.packages
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

"""Load Configuration File"""
#config = os.path.join(
#    get_package_share_directory('phidgets_spatial'),
#    'config',
#    'phidgets_spatial_param.yaml'
#)
#config_directory = os.path.join(
#        ament_index_python.packages.get_package_share_directory('phidgets_spatial'),
#        'config')
#print(config_directory)
#params = os.path.join(config_directory, 'phidgets_spatial_param.yaml')
params = os.path.join(get_package_share_directory('phidgets_spatial'),  'config', 'phidgets_spatial_param.yaml' )

#print(config)
def generate_launch_description():
    container = ComposableNodeContainer(
            node_name='phidget_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_spatial',
                    node_plugin='phidgets::SpatialRosI',
                    node_name='phidgets_spatial',
                    parameters=[{'cc_mag_field':1.18772,
                                'cc_offset0': -0.01963,
                                'cc_offset1': 0.01095,
                                'cc_offset2': 0.0,
                                'cc_gain0': 0.82156,
                                'cc_gain1': 0.86235,
                                'cc_gain2': 0.84195,
                                'cc_t0':-0.01557,
                                'cc_t1':0.0,
                                'cc_t2':-0.01529,
                                'cc_t3':0.0,
                                'cc_t4':0.0,
                                'cc_t5':0.0
                                }]
                    #parameters =[{params}]
                    ),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])