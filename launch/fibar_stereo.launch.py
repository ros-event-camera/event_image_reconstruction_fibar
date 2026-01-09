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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EqualsSubstitution

cam_names = ("cam_0_", "cam_1_")


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    nodes = [
        ComposableNode(
            package="event_image_reconstruction_fibar",
            plugin="event_image_reconstruction_fibar::Fibar",
            namespace=LaunchConfig(cam + "camera_name"),
            name="fibar",
            parameters=[
                {
                    "fps": PythonExpression(["float(", LaunchConfig(cam + "fps"), ")"]),
                    "use_trigger_events": LaunchConfig(cam + "use_trigger_events"),
                    "publish_time_reference": LaunchConfig(
                        cam + "publish_time_reference"
                    ),
                    "trigger_edge": LaunchConfig(cam + "trigger_edge"),
                    "frame_path": LaunchConfig(cam + "frame_path"),
                    "sync_mode": LaunchConfig(cam + "sync_mode"),
                    "use_sim_time": LaunchConfig("use_sim_time"),
                }
            ],
            remappings=[
                ("~/events", LaunchConfig(cam + "event_topic")),
                ("~/frame_image", LaunchConfig(cam + "frame_image")),
                ("~/time_reference", LaunchConfig(cam + "time_reference")),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam in cam_names
    ]
    topic_names = [
        "/" + LaunchConfig(cam + "camera_name").perform(context) + "/fibar/image"
        for cam in cam_names
    ]
    nodes.append(
        ComposableNode(
            package="rosbag2_transport",
            plugin="rosbag2_transport::Recorder",
            name="recorder",
            parameters=[
                {
                    "record.topics": topic_names,
                    "record.start_paused": False,
                    "storage.uri": LaunchConfig("output_bag"),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
            condition=UnlessCondition(
                EqualsSubstitution(LaunchConfig("output_bag"), "")
            ),
        ),
    )

    container = ComposableNodeContainer(
        name="fibar_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        # prefix=["xterm -e gdb -ex run --args"],
        composable_node_descriptions=nodes,
        output="screen",
    )
    return [container]


params = [
    ("camera_name", ["event_camera"], "camera name"),
    ("fps", ["25.0"], "frame rate (negative to disable)"),
    ("publish_time_reference", "False", "whether to publish time reference"),
    ("use_trigger_events", "False", "if trigger events should be used"),
    ("trigger_edge", "up", "use up or down edge of trigger signal"),
    (
        "frame_image",
        "/cam_sync/cam0/image_raw",
        "topic of frame camera images to sync to",
    ),
    (
        "time_reference",
        "fibar/time_reference",
        "topic of time reference",
    ),
    ("event_topic", "camera/events", "topic of event camera events"),
    ("frame_path", "", "output file path for frames"),
    (
        "sync_mode",
        "free_running",
        "synchronization mode (free_running, trigger_events, camera_image, time_reference)",
    ),
]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    sim_time_arg = LaunchArg(
        "use_sim_time", default_value="False", description="use_sim_time"
    )
    bag_arg = LaunchArg(
        "output_bag", default_value="", description="name of output bag (or empty)"
    )
    per_cam_args = [
        LaunchArg(cam + p[0], default_value=p[1], description=p[2])
        for p in params
        for cam in cam_names
    ]

    return launch.LaunchDescription(
        [sim_time_arg, bag_arg] + per_cam_args + [OpaqueFunction(function=launch_setup)]
    )
