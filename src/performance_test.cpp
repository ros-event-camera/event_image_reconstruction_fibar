// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <event_image_reconstruction_fibar/fibar.hpp>
#include <filesystem>
#include <rosbag2_transport/player.hpp>
#include <sensor_msgs/msg/image.hpp>

using Parameter = rclcpp::Parameter;
using Player = rosbag2_transport::Player;

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("performance_test"));
}

static void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "performance_test -i input_bag -t topic [-n (no spatial "
               "filter)] [-f fps]"
            << std::endl;
}

static std::shared_ptr<Player> makePlayerNode(const std::string in_uri)
{
  if (in_uri.empty()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("player"), "must have in_bag parameter set!");
    throw std::runtime_error("in_bag parameter nost set!");
  }
  if (!std::filesystem::exists(in_uri)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("player"), "cannot find input bag: " << in_uri);
    throw std::runtime_error("cannot find input bag!");
  }

  rclcpp::NodeOptions player_options;
  player_options.parameter_overrides(
    {Parameter("storage.uri", in_uri),  // Parameter("play.topics", in_topics),
     Parameter("play.clock_publish_on_topic_publish", true),
     Parameter("play.start_paused", true), Parameter("play.rate", 1000.0),
     Parameter("play.progress_bar_update_rate", 0),
     Parameter("play.disable_keyboard_controls", true)});
  auto player_node = std::make_shared<Player>("rosbag_player", player_options);
  player_node->get_logger().set_level(rclcpp::Logger::Level::Warn);
  return (player_node);
}

class ImageSubscriber : public rclcpp::Node
{
public:
  explicit ImageSubscriber(const rclcpp::NodeOptions & options)
  : Node("image_subscriber", options)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/fibar/image", rclcpp::QoS(2),
      std::bind(&ImageSubscriber::imageMsg, this, std::placeholders::_1));
  }
  void imageMsg(sensor_msgs::msg::Image::ConstSharedPtr /*msg*/)
  {
    num_images_++;
  }

private:
  // ---------- variables
  size_t num_images_{0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  int opt;
  rclcpp::init(argc, argv);
  std::string in_file;
  std::string topic;
  double fps = 20.0;
  bool use_spatial_filter = true;
  while ((opt = getopt(argc, argv, "i:t:f:nh")) != -1) {
    switch (opt) {
      case 'i':
        in_file = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'n':
        use_spatial_filter = false;
        break;
      case 'f':
        fps = std::stod(optarg);
        break;
      case 'h':
        usage();
        return (-1);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (optind != argc) {
    std::cout << "extra arguments not allowed!" << std::endl;
    usage();
    return (-1);
  }
  if (in_file.empty() || topic.empty()) {
    std::cout << "input bag and topic must be specified!" << std::endl;
    usage();
    return (-1);
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  // rclcpp::executors::MultiThreadedExecutor exec;
  const bool use_ipc = false;
  std::vector<std::string> remap{"--ros-args", "--remap", "~/events:=" + topic};
  rclcpp::NodeOptions fibar_options;
  fibar_options.use_intra_process_comms(use_ipc);
  fibar_options.parameter_overrides(
    {Parameter("use_sim_time", true), Parameter("fps", fps),
     Parameter("use_spatial_filter", use_spatial_filter),
     Parameter("event_queue_memory_limit", 1024 * 1024 * 1024),
     Parameter("ros_event_queue_size", 100000)});
  fibar_options.arguments(remap);

  auto fibar_node =
    std::make_shared<event_image_reconstruction_fibar::Fibar>(fibar_options);
  rclcpp::NodeOptions subscriber_options;
  subscriber_options.use_intra_process_comms(use_ipc);
  subscriber_options.parameter_overrides({Parameter("use_sim_time", true)});
  auto subscriber_node = std::make_shared<ImageSubscriber>(subscriber_options);

  auto player_node = makePlayerNode(in_file);
  while (player_node->play_next() && rclcpp::ok()) {
    exec.spin_node_some(subscriber_node);
    exec.spin_node_some(player_node);
    exec.spin_node_some(fibar_node);
  }
  RCLCPP_INFO_STREAM(get_logger(), "finished reconstruction in 0 seconds");
  player_node.reset();
  fibar_node.reset();
  return 0;
}
