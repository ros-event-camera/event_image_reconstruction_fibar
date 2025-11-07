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

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <event_camera_msgs/msg/event_packet.hpp>
#include <event_image_reconstruction_fibar/check_endian.hpp>
#include <event_image_reconstruction_fibar/fibar.hpp>
#include <filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

namespace event_image_reconstruction_fibar
{
Fibar::Fibar(const rclcpp::NodeOptions & options)
: Node(
    "fibar", rclcpp::NodeOptions(options)
               .automatically_declare_parameters_from_overrides(true))
{
  img_msg_template_.height = 0;
#ifdef IMAGE_TRANSPORT_USE_QOS
  const auto qosProf = rclcpp::SystemDefaultsQoS();
#else
  const auto qosProf = rmw_qos_profile_default;
#endif

  image_pub_ = image_transport::create_publisher(
#ifdef IMAGE_TRANSPORT_USE_NODEINTERFACE
    *this,
#else
    this,
#endif
    "~/image_raw", qosProf);
  this->get_parameter_or("cutoff_num_events", cutoff_num_events_, 40);
  double fps;
  this->get_parameter_or("fps", fps, -1.0);
  time_slice_ = (fps <= 0.0) ? -1.0 : (1.0 / fps);
  // Poll since the ROS2 image transport does not yet call back when subscribers come and go.
  subscription_check_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&Fibar::subscriptionCheckTimerExpired, this));
  if (time_slice_ >= 0) {
    RCLCPP_INFO_STREAM(
      get_logger(), "in free running mode with frame rate " << fps << " Hz");
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(), "driving frame times from topic "
                      << this->get_node_topics_interface()->resolve_topic_name(
                           "~/frame_image"));
  }
  this->get_parameter_or("statistics_period", statistics_period_, 5.0);
  last_statistics_time_ = this->now();
  statistics_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(statistics_period_),
    std::bind(&Fibar::statisticsTimerExpired, this));
  this->get_parameter_or(
    "event_queue_memory_limit", event_queue_memory_limit_, 10 * 1024 * 1024);
  this->get_parameter_or("use_trigger_events", use_trigger_events_, false);
  std::string edge;
  this->get_parameter_or<std::string>("trigger_edge", edge, "up");
  trigger_events_edge_ = (edge == "up" || edge == "UP") ? 1 : 0;
  if (use_trigger_events_ && time_slice_ > 0) {
    RCLCPP_WARN(
      get_logger(),
      "cannot use trigger events in free running mode, disabling triggers!");
    use_trigger_events_ = false;
  }
  RCLCPP_INFO_STREAM(
    get_logger(),
    "using trigger events: " << (use_trigger_events_ ? edge : "NONE"));
  this->get_parameter_or<std::string>("frame_path", frame_path_, "");
  if (!frame_path_.empty()) {
    // Create the directory (and any necessary parent directories)
    if (!std::filesystem::create_directories(frame_path_)) {
      if (
        !std::filesystem::exists(frame_path_) &&
        std::filesystem::is_directory(frame_path_)) {
        RCLCPP_ERROR_STREAM(
          get_logger(), "cannot write frames to path: " << frame_path_);
        throw std::runtime_error("cannot create frame output directory");
      }
    }
    RCLCPP_INFO_STREAM(
      get_logger(), "frames will be written to: " << frame_path_);
  }
}

Fibar::~Fibar()
{
  if (statistics_timer_) {
    statistics_timer_->cancel();
  }
  if (frame_timer_) {
    frame_timer_->cancel();
  }
  if (subscription_check_timer_) {
    subscription_check_timer_->cancel();
  }
  if (image_sub_) {
    image_sub_.reset();
  }
}

void Fibar::subscriptionCheckTimerExpired()
{
  // this silly dance is only necessary because ROS2 at this time does not support
  // callbacks when subscribers come and go
  if (image_pub_.getNumSubscribers()) {
    if (!event_sub_) {
      RCLCPP_INFO(this->get_logger(), "subscribing to events!");
      const int qsize = 1000;
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(qsize))
                         .best_effort()
                         .durability_volatile();
      event_sub_ = this->create_subscription<EventPacket>(
        "~/events", qos,
        std::bind(&Fibar::eventMsg, this, std::placeholders::_1));
    }
    if (time_slice_ > 0) {
      if (!frame_timer_) {
        frame_timer_ = rclcpp::create_timer(
          this, get_clock(), rclcpp::Duration::from_seconds(time_slice_),
          std::bind(&Fibar::frameTimerExpired, this));
      }
    } else if (!image_sub_) {
      image_sub_ = this->create_subscription<Image>(
        "~/frame_image", rclcpp::SystemDefaultsQoS(),
        std::bind(&Fibar::imageMsg, this, std::placeholders::_1));
      RCLCPP_INFO_STREAM(get_logger(), "subscribed to image topic for frames!");
    }
  } else {
    // -------------- no subscribers -------------------
    if (event_sub_) {
      RCLCPP_INFO(this->get_logger(), "unsubscribing from events!");
      event_sub_.reset();
    }
    if (frame_timer_) {
      // if nobody is listening, stop publishing frames if this is currently happening
      frame_timer_->cancel();
      frame_timer_.reset();
    }
    if (image_sub_) {
      image_sub_.reset();
      RCLCPP_INFO(
        this->get_logger(), "unsubscribed from image topic for frames!");
    }
  }
}

void Fibar::handleFirstMessage(const EventPacket::ConstSharedPtr & msg)
{
  if (encoding_.empty()) {
    encoding_ = msg->encoding;
    img_msg_template_.header = msg->header;
    img_msg_template_.width = msg->width;
    img_msg_template_.height = msg->height;
    img_msg_template_.encoding = "mono8";
    img_msg_template_.is_bigendian = check_endian::isBigEndian();
    img_msg_template_.step = img_msg_template_.width;
    RCLCPP_INFO_STREAM(
      get_logger(), "initializing reconstructor for sensor size "
                      << msg->width << " x " << msg->height
                      << " encoding: " << encoding_);
    reconstructor_.initialize(
      msg->width, msg->height,
      static_cast<uint32_t>(std::abs(cutoff_num_events_)), 0.5);
    decoder_ = decoder_factory_.getInstance(*msg);
  }
  // create temporary decoder to find the correspondence between
  // ros time and sensor time
  auto tmp_decoder = decoder_factory_.newInstance(*msg);
  uint64_t sensor_time{0};
  if (tmp_decoder->findFirstSensorTime(*msg, &sensor_time)) {
    RCLCPP_INFO_STREAM(
      get_logger(), "initializing: ROS time "
                      << rclcpp::Time(msg->header.stamp).nanoseconds()
                      << " corresponds to sensor time: " << sensor_time);
    updateRosToSensorTimeOffset(
      rclcpp::Time(msg->header.stamp), static_cast<int64_t>(sensor_time));
  }
}

void Fibar::eventMsg(EventPacket::ConstSharedPtr msg)
{
  if (t0_ == std::numeric_limits<int64_t>::lowest()) {
    handleFirstMessage(msg);
  } else {
    if (
      img_msg_template_.height != msg->height ||
      img_msg_template_.width != msg->width || encoding_ != msg->encoding) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cannot change encoding type or sensor size on the fly!");
      return;
    }
  }
  if (
    event_queue_memory_ + msg->events.size() <
    static_cast<size_t>(event_queue_memory_limit_)) {
    event_msg_queue_.push(msg);
    event_queue_memory_ += msg->events.size();
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "event message queue full, dropping incoming event message!");
  }
  processEventMessages();
}

bool Fibar::eventExtTrigger(uint64_t t, uint8_t edge, uint8_t)
{
  if (!use_trigger_events_ || trigger_events_edge_ != edge) {
    return (true);
  }
  num_trigger_events_++;
  trigger_period_.update(t);
  emitFramesForTrigger(t);
  // keep going if there are still frames to be processed
  return (!frames_.empty());
}

template <class T>
static bool frameIsCloseToTrigger(
  const T & frame, uint64_t trigger_time, int64_t tolerance)
{
  const int64_t dt = static_cast<int64_t>(frame.sensor_time) -
                     static_cast<int64_t>(trigger_time);
  return (std::abs(dt) <= tolerance);
}

template <class T>
static bool frameIsOlderThanTrigger(
  const T & frame, uint64_t trigger_time, int64_t tolerance)
{
  return (frame.sensor_time + tolerance < trigger_time);
}

void Fibar::emitFramesForTrigger(uint64_t t_sensor_trigger)
{
  const auto & T_frame = frame_period_.getPeriod();
  const auto & T_trigger = trigger_period_.getPeriod();
  if (T_frame <= 0 || T_trigger <= 0) {
    return;
  }
  if (std::abs(T_frame - T_trigger) > 0.25 * T_frame) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "frame freq " << 1e9 / T_frame << " != trigger freq " << 1e9 / T_trigger
                    << ", ignoring triggers!");
    return;
  }
  int64_t tol = static_cast<int64_t>(T_frame * 0.25);
  // purge old frames or gap in frames
  while (!frames_.empty() &&
         frameIsOlderThanTrigger(frames_.front(), t_sensor_trigger, tol)) {
    frames_.pop_front();
  }
  // now the front frame should be younger than the trigger time
  if (!frames_.empty()) {
    const auto & frame = frames_.front();
    if (frameIsCloseToTrigger(frame, t_sensor_trigger, tol)) {
      // front().sensor_time has the sensor time corresponding
      // to the ROS header time that the frame camera driver put on the image.
      // If the sensor time of the trigger event is smaller (older), it means
      // that the frame camera driver is slower than the event camera driver
      // in handling the frames.
      // Irrespective of that, the published frame has the header stamp of the
      // frame camera image such that it's clear which frames belong together.
      const int64_t delay = static_cast<int64_t>(frame.sensor_time) -
                            static_cast<int64_t>(t_sensor_trigger);
      frame_delay_ = frame_delay_ * 0.9 + static_cast<double>(delay) * 0.1;
      publishFrame(frame.ros_time);
      frames_.pop_front();
    } else {
      RCLCPP_WARN_STREAM(
        get_logger(), "no frame found for ext trigger at sensor time "
                        << t_sensor_trigger * 1e-9 << " dropping it!");
    }
  }
}

void Fibar::processEventMessagesWithTriggers()
{
  // Note that this will not process event messages unless
  // there are frames avaiable. This
  while (!event_msg_queue_.empty() && !frames_.empty()) {
    const auto & msg = event_msg_queue_.front();
    ros_header_time_ = rclcpp::Time(msg->header.stamp);
    // stay in this loop until either all frames are used up,
    // or the event message is completely decoded
    while (!frames_.empty() && decoder_->decode(*msg, this)) {
      // the extTriggerEvent() called during decode() will publish the frames
    }
    if (!frames_.empty()) {  // means the message has been completely decoded
      event_queue_memory_ -= msg->events.size();
      event_msg_queue_.pop();
      is_first_time_in_packet_ = true;  // for next event message
    }
  }
}

void Fibar::processEventMessages()
{
  if (use_trigger_events_) {
    processEventMessagesWithTriggers();
    return;
  }
  if (t0_ == std::numeric_limits<int64_t>::lowest()) {
    return;
  }
  auto & frames = frames_;
  while (!event_msg_queue_.empty() && !frames.empty()) {
    const auto & msg = event_msg_queue_.front();
    ros_header_time_ = rclcpp::Time(msg->header.stamp);
    while (!frames.empty()) {
      const uint64_t time_limit = frames.front().sensor_time;
      uint64_t next_time = 0;
      if (!decoder_->decodeUntil(*msg, this, time_limit, &next_time)) {
        // event message was completely decoded. Cannot emit frame yet
        // because more events may arrive that are before the frame time
        event_queue_memory_ -= msg->events.size();
        event_msg_queue_.pop();
        is_first_time_in_packet_ = true;  // for next event message
        break;
      }
      while (!frames.empty() && frames.front().sensor_time <= next_time) {
        publishFrame(frames.front().ros_time);
        frames.pop_front();
      }
    }
  }
}

void Fibar::publishFrame(const rclcpp::Time & t)
{
  auto msg = std::make_unique<Image>(img_msg_template_);
  msg->header.stamp = t;
  const bool need_img =
    image_pub_.getNumSubscribers() != 0 || !frame_path_.empty();
  if (need_img) {
    msg->data.resize(msg->step * msg->height);
    reconstructor_.getImage(msg->data.data(), msg->step);
  }
  // must first write to disk before invalidating the msg with std::move
  if (!frame_path_.empty()) {
    const std::string filename =
      frame_path_ + "/fibar_" + std::to_string(t.nanoseconds()) + ".png";
    const cv_bridge::CvImageConstPtr cv_img =
      cv_bridge::toCvCopy(*msg, "mono8");
    cv::imwrite(filename, cv_img->image);
  }
  if (image_pub_.getNumSubscribers() != 0) {
    image_pub_.publish(std::move(msg));
  }
}

void Fibar::updateRosToSensorTimeOffset(
  const rclcpp::Time & t_ros, int64_t t_sens)
{
  if (t0_ == std::numeric_limits<int64_t>::lowest()) {
    t0_ = static_cast<int64_t>(t_ros.nanoseconds()) - t_sens;
    t0_init_ = t0_;
  } else {
    // to avoid rounding errors, first subtract off the large t0_init_,
    // which contains the time since epoch.
    const double dt_meas = static_cast<double>(
      static_cast<int64_t>(t_ros.nanoseconds()) - t0_init_ -
      t_sens);  // measured
    const double dt_k =
      static_cast<double>(t0_ - t0_init_);  // current estimate
    constexpr double alpha = 1.0 / 100.0;
    t0_ = t0_init_ + static_cast<int64_t>(dt_k * (1 - alpha) + dt_meas * alpha);
  }
}

void Fibar::addNewFrame(const FrameTime & ft)
{
  num_frames_generated_++;
  if (frames_.size() >= 100) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "frames dropped due to frame queue overflow!");
  } else {
    frames_.push_back(ft);
  }
  processEventMessages();
}

void Fibar::PeriodEstimator::update(uint64_t t)
{
  if (num_initial_ >= 1) {
    const double dt =
      static_cast<int64_t>(t) - static_cast<int64_t>(last_time_);
    if (num_initial_ > 1) {
      est_period_ = est_period_ * 0.9 + 0.1 * dt;
    } else {
      est_period_ = dt;
    }
  } else {
    num_initial_++;
  }
  last_time_ = t;
}

void Fibar::frameTimerExpired()
{
  const rclcpp::Time t = this->get_clock()->now();
  frame_period_.update(t.nanoseconds());
  if (image_pub_.getNumSubscribers() == 0) {
    return;
  }
  if (t0_ == std::numeric_limits<int64_t>::lowest()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no event messages received yet, cannot produce frames!");
    return;
  }
  addNewFrame(FrameTime(t, rosToSensorTime(t)));
}

void Fibar::imageMsg(const Image::ConstSharedPtr msg)
{
  frame_period_.update(rclcpp::Time(msg->header.stamp).nanoseconds());
  if (t0_ == std::numeric_limits<int64_t>::lowest()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no event messages received yet, cannot produce frames!");
    return;
  }
  const rclcpp::Time t(msg->header.stamp);
  addNewFrame(FrameTime(t, rosToSensorTime(t)));
  if (!frame_path_.empty()) {
    const std::string filename =
      frame_path_ + "/frame_" + std::to_string(t.nanoseconds()) + ".png";
    const cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(msg);
    cv::imwrite(filename, cv_img->image);
  }
}

void Fibar::statisticsTimerExpired()
{
  const rclcpp::Time now = this->get_clock()->now();
  if (now < last_statistics_time_) {
    // happens when ros2 bag play is restarted with use_sim_time = true
    last_statistics_time_ = now;
    return;
  }
  const double dt = std::max((now - last_statistics_time_).seconds(), 1e-6);
  const double ev_rate = static_cast<double>(num_events_processed_) / dt;
  const double fr_rate = static_cast<double>(num_frames_generated_) / dt;
  const double fr_est = frame_period_.getRate();
  const double tr_rate = num_trigger_events_ / dt;
  const double tr_est = trigger_period_.getRate();
  RCLCPP_INFO(
    get_logger(),
    "%6.2f Mevs, frame: %6.2f(est: %6.2f)Hz trig: %6.2f(est: %6.2f)Hz delay: "
    "%6.3f ms",
    ev_rate * 1e-6, fr_rate, fr_est, tr_rate, tr_est, frame_delay_ * 1e-6);
  num_events_processed_ = 0;
  num_frames_generated_ = 0;
  num_trigger_events_ = 0;
  last_statistics_time_ = now;
}
std::ostream & operator<<(std::ostream & os, const Fibar::FrameTime & ft)
{
  os << "ros: " << ft.ros_time.nanoseconds() * 1e-9
     << " sensor: " << ft.sensor_time;
  return (os);
}
}  // namespace event_image_reconstruction_fibar

RCLCPP_COMPONENTS_REGISTER_NODE(event_image_reconstruction_fibar::Fibar)
