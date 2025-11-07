# event\_image\_reconstruction\_fibar

This repository contains a ROS package for event image reconstruction by
means of a temporal and spatial filtering algorithm
[described here](https://arxiv.org/abs/2510.20071). It depends
on the [fibar library](https://github.com/ros-event-camera/fibar_lib).

## Supported platforms

Continuous integration testing for ROS Humble and later distros.

## How to build

Set the following shell variables:
```bash
repo=event_image_reconstruction_fibar
url=https://github.com/ros-event-camera/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## Node Parameters

- ``fps``: Frequency (in hz) at which images are reconstructed in free running mode.
  Set to negative to disable free running mode. Default: 25.
- ``cutoff_num_events``: The cutoff period (in number of events) for the reconstruction algorithm, see [the FIBAR paper](https://arxiv.org/abs/2510.20071). Default: 40
- ``statistics_period``: Time period in seconds between statistics printouts. Default: 5.
- ``event_queue_memory_limit``: How many bytes of event data to keep in the incoming queue before dropping data. Default: 10MB.
- ``use_trigger_events``: Set this to true to use trigger events in the event data stream (hardware sync'ed setup). Default: False.
- ``edge``: Whether to use the ``up`` or ``down`` edge of the hardware trigger signal. Default: ``up``.
- ``frame_path``: output directory for reconstructed frames and frame-based camera images. Set to empty string to suppress frame writing. Default: ``""``.

## Node Topics

Publishers:

- ``~/image_raw``: the reconstructed image frame

Subscribers:

- ``~/events``: input event camera events (includes triggers!)
- ``~/frame_image``: image of frame-based camera for synchronization (hardware or software)

## How to use

This ROS node takes events from an event camera running a ROS driver, and reconstructs a log(intensity) image
from it. There are several options to choose the time of image reconstruction:

1) Free-running mode. This means there is no synchronization, and the produced frames will be at a fixed
   frame rate, equidistant in sensor time, not ROS or system time.
2) Software synchronized with external camera. The reconstruction node subscribes to a camera topic. When an image frame
   arrives, it translates the ROS header time stamp of image message to sensor time using its internally
   estimated offset between sensor time and ROS time. This sensor time is then used for image reconstruction.
3) Hardware synchronized with external camera (currently Metavision-based cameras only).
   This requires a trigger-in hardware sync pulse be sent to the event camera whenever a frame-based camera frame is triggered. The reconstruction node will use the trigger event's sensor time to reconstruct the intensity image,
   then translate the trigger time to ROS time to look up the corresponding header stamp of the frame-based camera image
   that likely corresponds to this trigger event. This header stamp will be used for the published reconstructed image.

The topics in the below example must be adjusted to work for the specific setup.
As for any ROS-based project, check that the topics are connected correctly by using ``ros2 node list``, ``ros2 node info`` and ``ros2 topic list``. Also bear in mind that the FIBAR node operates with lazy subscribe. In order for it to do anything, you *must subscribe to the reconstructed image* (rqt_gui is good tool for that)

In all the below cases ``use_sim_time`` is set to ``true`` because it is assumed that the data is played back from a rosbag that drives the clock:

```bash
ros2 bag play --clock-topics-all my_bag_with_data
```

Example launch for case 1: free-running (unsynchronized) mode:

```bash
ros2 launch event_image_reconstruction_fibar fibar.launch.py camera_name:=event_cam_0 fps:=25 frame_path:=./frames use_sim_time:=true
```

Example launch for case 2: software synchronized setup. The free running must be explicitly disabled by passing ``fps:=-1``.

```bash
ros2 launch event_image_reconstruction_fibar fibar.launch.py camera_name:=event_cam_0 frame_camera_topic:=/cam_sync/cam0/image_raw fps:=-1 use_trigger_events:=false frame_path:=./frames use_sim_time:=true
```

Example launch for case 3: hardware-synced setup, triggering on the up edge of the signal. The free running must be explicitly disabled by passing ``fps:=-1``.

```bash
ros2 launch event_image_reconstruction_fibar fibar.launch.py camera_name:=event_cam_0 frame_camera_topic:=/cam_sync/cam0/image_raw fps:=-1 use_trigger_events:=true edge:=up frame_path:=./frames use_sim_time:=true
```

In this hardware synchronized case, the FIBAR node will output statistics showing event rate, frame-based camera rate, and trigger event rate. The frame-based camera rate and trigger rate must be very close for the frame-to-trigger association to work. The last column gives the estimated time delay of the frame-based camera image with respect to the event camera trigger pulse.

```text
[INFO] [1762533207.007511785] [event_cam_0.fibar]:   7.49 Mevs, frame:  39.36(est:  38.13)Hz trig:  39.36(est:  38.10)Hz delay: -3.242 ms
[INFO] [1762533211.999424224] [event_cam_0.fibar]:   7.28 Mevs, frame:  38.06(est:  38.35)Hz trig:  38.06(est:  38.10)Hz delay: -3.060 ms
[INFO] [1762533217.007415855] [event_cam_0.fibar]:   7.54 Mevs, frame:  38.14(est:  38.22)Hz trig:  37.94(est:  38.10)Hz delay: -3.057 ms
```

There is also tool (``bag_to_frames``) for reconstruction of frames from bags with events but alas, it has not been documented yet.

## License

This software is issued under the Apache License Version 2.0.
