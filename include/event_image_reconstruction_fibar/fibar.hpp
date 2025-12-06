// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef EVENT_IMAGE_RECONSTRUCTION_FIBAR__FIBAR_HPP_
#define EVENT_IMAGE_RECONSTRUCTION_FIBAR__FIBAR_HPP_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>

#include <deque>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <fibar_lib/image_reconstructor.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <string>

namespace event_image_reconstruction_fibar
{
class Fibar : public rclcpp::Node, public event_camera_codecs::EventProcessor
{
public:
  using EventPacket = event_camera_msgs::msg::EventPacket;
  using Image = sensor_msgs::msg::Image;
  using TimeReference = sensor_msgs::msg::TimeReference;
  explicit Fibar(const rclcpp::NodeOptions & options);
  ~Fibar();

  // ---------- inherited from EventProcessor
  inline void eventCD(
    uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    num_events_processed_++;
    if (use_spatial_filter_) {
      reconstructor_with_filter_.event(sensor_time, ex, ey, polarity);
    } else {
      reconstructor_no_filter_.event(sensor_time, ex, ey, polarity);
    }
    updateFirstSensorTime(sensor_time);
  }

  bool eventExtTrigger(uint64_t t, uint8_t edge, uint8_t) override;

  // clang-format off
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // clang-format on
  // --------- end of inherited from EventProcessor

private:
  struct FrameTime
  {
    explicit FrameTime(rclcpp::Time ht, uint64_t st)
    : host_time(ht), sensor_time(st)
    {
    }
    rclcpp::Time host_time;
    uint64_t sensor_time{0};
  };
  friend std::ostream & operator<<(
    std::ostream & os, const Fibar::FrameTime & ft);

  class PeriodEstimator
  {
  public:
    PeriodEstimator() = default;
    double getRate() const
    {
      return (est_period_ <= 0 ? -1.0 : 1e9 / est_period_);
    }
    const auto & getPeriod() const { return est_period_; }
    void update(uint64_t t);

  private:
    uint8_t num_initial_{0};
    uint64_t last_time_{0};
    double est_period_{-1.0};
  };

  inline uint64_t hostToSensorTime(const rclcpp::Time & t) const
  {
    return (t.nanoseconds() - t0_);
  }
  inline rclcpp::Time sensorToHostTime(uint64_t t) const
  {
    return (rclcpp::Time(t + t0_, RCL_ROS_TIME));
  }
  inline void updateFirstSensorTime(uint64_t sensor_time)
  {
    if (is_first_time_in_packet_) {
      updateHostToSensorTimeOffset(
        header_time_, static_cast<int64_t>(sensor_time));
      is_first_time_in_packet_ = false;
    }
  }
  void frameTimerExpired();
  void subscriptionCheckTimerExpired();
  void statisticsTimerExpired();
  void eventMsg(EventPacket::ConstSharedPtr msg);
  void imageMsg(const Image::ConstSharedPtr msg);
  void timeReferenceMsg(const TimeReference::ConstSharedPtr msg);
#ifdef USE_MATCHED_EVENTS
  void subscriberChangedCallback(rclcpp::MatchedInfo & info);
#endif
  void checkSubscriptions();
  void processEventMessagesWithTriggersOnly();
  void processEventMessagesWithTriggers();
  void processEventMessagesWithoutTriggers();
  void processEventMessages();
  void updateHostToSensorTimeOffset(
    const rclcpp::Time & t_host, int64_t t_sens);
  void handleFirstMessage(const EventPacket::ConstSharedPtr & msg);
  void publishFrame(const rclcpp::Time & t);
  void publishTimeReference(const FrameTime & ft);
  void addNewFrame(const FrameTime & ft);
  void emitFramesForTrigger(uint64_t t_sensor_trigger);
  void startFrameStreaming();
  void stopFrameStreaming();
  void configure();
  void activate();
  void deconfigure();
  void deactivate();
  // ------------------------  variables ------------------------------
  rclcpp::TimerBase::SharedPtr frame_timer_;
  rclcpp::TimerBase::SharedPtr subscription_check_timer_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  rclcpp::Subscription<EventPacket>::SharedPtr event_sub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<TimeReference>::SharedPtr time_ref_sub_;
  rclcpp::Publisher<TimeReference>::SharedPtr time_reference_pub_;
  image_transport::Publisher image_pub_;
  std::string sync_mode_{"free_running"};
  double time_slice_{-1};  // duration of one frame
  Image img_msg_template_;
  std::string encoding_;             // currently used incoming message encoding
  size_t event_queue_memory_{0};     // currently used event queue memory
  int event_queue_memory_limit_{0};  // max event queue memory
  std::queue<EventPacket::ConstSharedPtr> event_msg_queue_;
  std::deque<FrameTime> frames_;
  int64_t t0_{
    std::numeric_limits<int64_t>::lowest()};  // host to sensor time offset
  int64_t t0_init_{0};
  event_camera_codecs::Decoder<EventPacket, Fibar> * decoder_{nullptr};
  event_camera_codecs::DecoderFactory<EventPacket, Fibar> decoder_factory_;
  rclcpp::Time header_time_;
  bool is_first_time_in_packet_{true};
  fibar_lib::ImageReconstructor<true, 2> reconstructor_with_filter_;
  fibar_lib::ImageReconstructor<false, 2> reconstructor_no_filter_;
  bool use_spatial_filter_{true};
  int cutoff_num_events_{40};
  size_t num_trigger_events_ = 0;
  bool publish_time_reference_{false};
  bool use_trigger_events_{false};
  bool trigger_generates_frames_{false};
  uint8_t trigger_events_edge_{0};
  double statistics_period_{5.0};
  size_t num_events_processed_{0};
  size_t num_frames_generated_{0};
  int64_t lag_sum_{0};
  size_t lag_num_{0};
  rclcpp::Time last_statistics_time_;
  rclcpp::Time last_statistics_w_;  // wall time
  PeriodEstimator frame_period_;
  PeriodEstimator trigger_period_;
  double frame_delay_{0.0};
  std::string frame_path_{""};
};
std::ostream & operator<<(std::ostream & os, const Fibar::FrameTime & ft);
}  // namespace event_image_reconstruction_fibar
#endif  // EVENT_IMAGE_RECONSTRUCTION_FIBAR__FIBAR_HPP_
