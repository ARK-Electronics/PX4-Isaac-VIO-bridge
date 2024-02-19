# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Realsense node
    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': True,
                'rgb_camera.profile': '640x360x30',
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x90',
                'enable_gyro': False,
                'enable_accel': False,
                # 'gyro_fps': 200,
                # 'accel_fps': 200,
                # 'unite_imu_method': 2
        }]
    )

    # Static transform publisher PX4 to ROS2
    # Apply ROLL_180 for NED to FLU
    camera_link_gyro_tf_node = Node(
        name='camera_link_gyro_tf',
        namespace='camera_link_gyro_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        # x y z yaw pitch roll frame_id child_frame_id
        arguments = ['0', '0', '0', '0', '0', '3.14159265359', 'camera_link', 'camera_gyro_frame']
    )

    # Static transform publisher ROS2 to RS Optical
    # https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems
    gyro_optical_tf_node = Node(
        name='gyro_optical_tf',
        namespace='gyro_optical_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '-0.5', '0.5', '-0.5', '0.5', 'camera_gyro_frame', 'camera_gyro_optical_frame']
    )

    # Converts VIO solution to PX4 topic
    vio_transform_node = Node(
        name='vio_transform',
        namespace='vio_transform',
        package='px4_vslam',
        executable='vio_transform'
    )

    # Isaac ROS VSLAM
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera_link',
                    'input_imu_frame': 'camera_gyro_optical_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.001,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.003,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 100.0,
                    'img_jitter_threshold_ms': 22.00
                    }],
        remappings=[('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'vio_transform/imu')]
    )

    # Foxglove
    foxglove_bridge_node = Node(
        name='foxglove_bridge',
        namespace='foxglove_bridge',
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
                    'port': 8765,
                    'address': '0.0.0.0',
                    'tls': False,
                    'certfile': '',
                    'keyfile': '',
                    'topic_whitelist': ['.*'],
                    'service_whitelist': ['.*'],
                    'param_whitelist': ['.*'],
                    'client_topic_whitelist': ['.*'],
                    'min_qos_depth': 1,
                    'max_qos_depth': 10,
                    'num_threads': 0,
                    'send_buffer_limit': 10000000,
                    'use_sim_time': False,
                    'capabilities': ['clientPublish','parameters','parametersSubscribe','services','connectionGraph','assets'],
                    'include_hidden': False,
                    'asset_uri_allowlist': ['package://(\\w+/?)+\\.(dae|stl|urdf|xacro)']
                    }]
    )

    # Launch all nodes in the same process
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )
    # return launch.LaunchDescription([visual_slam_launch_container, realsense_camera_node, imu_transform_node, vio_transform_node, foxglove_bridge_node])
    return launch.LaunchDescription([visual_slam_launch_container, realsense_camera_node, camera_link_gyro_tf_node, gyro_optical_tf_node, vio_transform_node, foxglove_bridge_node])
