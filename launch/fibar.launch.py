# -----------------------------------------------------------------------------
# Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    cam_name = LaunchConfig("camera_name")
    cam_str = cam_name.perform(context)
    container = ComposableNodeContainer(
        name="fibar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        # prefix=["xterm -e gdb -ex run --args"],
        composable_node_descriptions=[
            ComposableNode(
                package="event_image_reconstruction_fibar",
                plugin="event_image_reconstruction_fibar::Fibar",
                namespace=cam_name,
                name="fibar",
                parameters=[
                    {
                        "fps": PythonExpression(["float(", LaunchConfig("fps"), ")"]),
                        "use_trigger_events": LaunchConfig("use_trigger_events"),
                        "trigger_edge": LaunchConfig("trigger_edge"),
                        "frame_path": LaunchConfig("frame_path"),
                        "sync_mode": LaunchConfig("sync_mode"),
                        "use_sim_time": LaunchConfig("use_sim_time"),
                    }
                ],
                remappings=[
                    ("~/events", LaunchConfig("event_topic")),
                    ("~/frame_image", LaunchConfig("frame_image")),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        output="screen",
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg(
                "camera_name", default_value=["event_camera"], description="camera name"
            ),
            LaunchArg(
                "fps",
                default_value=["25.0"],
                description="frame rate (negative to disable)",
            ),
            LaunchArg(
                "use_sim_time", default_value="False", description="use_sim_time"
            ),
            LaunchArg(
                "use_trigger_events",
                default_value="False",
                description="if trigger events should be used",
            ),
            LaunchArg(
                "trigger_edge",
                default_value="up",
                description="use up or down edge of trigger signal",
            ),
            LaunchArg(
                "frame_image",
                default_value="/cam_sync/cam0/image_raw",  # modify as needed
                description="topic of frame camera images to sync to",
            ),
            LaunchArg(
                "event_topic",
                default_value="camera/events",  # modify as needed
                description="topic of event camera events",
            ),
            LaunchArg(
                "frame_path",
                default_value="",  # no frames written by default
                description="output file path for frames",
            ),
            LaunchArg(
                "sync_mode",
                default_value="free_running",
                description="synchronization mode (free_running, trigger_events, camera_image, time_reference)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
