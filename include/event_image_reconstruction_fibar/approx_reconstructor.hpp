// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@eventvisionresearch.com>
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

#ifndef EVENT_IMAGE_RECONSTRUCTION_FIBAR__APPROX_RECONSTRUCTOR_HPP_
#define EVENT_IMAGE_RECONSTRUCTION_FIBAR__APPROX_RECONSTRUCTOR_HPP_

#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/event_processor.h>

#include <event_image_reconstruction_fibar/check_endian.hpp>
#include <event_image_reconstruction_fibar/frame_handler.hpp>
#include <event_image_reconstruction_fibar/ros_compat.hpp>
#include <event_image_reconstruction_fibar/time_offset.hpp>
#include <fibar_lib/image_reconstructor.hpp>
#include <memory>
#include <queue>
#include <string>

namespace event_image_reconstruction_fibar
{
template <
  typename EventPacketT, typename EventPacketConstSharedPtrT, typename ImageT,
  typename ImageConstPtrT, typename RosTimeT, int tile_size>
class ApproxReconstructor : public event_camera_codecs::EventProcessor
{
public:
  struct FrameTime
  {
    FrameTime(uint64_t s, const RosTimeT & r) : sensor_time(s), ros_time(r) {}
    uint64_t sensor_time{0};
    RosTimeT ros_time;
  };

  using EventPacket = EventPacketT;
  explicit ApproxReconstructor(
    FrameHandler<ImageConstPtrT> * fh, const std::string & topic,
    int cutoff_num_events = 30, double fill_ratio = 0.5,
    bool active_pixels = false, const std::string & scale_file = std::string())
  : frame_handler_(fh),
    topic_(topic),
    active_pixels_(active_pixels),
    scale_file_(scale_file),
    cutoff_num_events_(cutoff_num_events),
    fill_ratio_(fill_ratio)
  {
    image_msg_template_.height = 0;
  }

  // ---------- inherited from EventProcessor
  inline void eventCD(
    uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    number_of_events_++;
    if (__builtin_expect(update_ros_to_sensor_time_offset_, false)) {
      const auto & msg = buffered_messages_.front();
      updateROSTimeOffset(
        ros_compat::to_nanoseconds(RosTimeT(msg->header.stamp)), t);
    }
// #define TEST_WITHOUT_RECONSTRUCTING
#ifdef TEST_WITHOUT_RECONSTRUCTING
    (void)ex;
    (void)t;
    (void)ey;
    (void)polarity;
#else
    reconstructor_.event(t, ex, ey, polarity);
#endif
  }
  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override { return (true); }
  void finished() override {}
  void rawData(const char *, size_t) override {}
  // --------- end of inherited from EventProcessor

  const auto & getDecodeTime() const { return decode_time_; }
  const auto & getNumberOfEvents() const { return number_of_events_; }

  void addFrameTime(uint64_t sensor_time, const RosTimeT t)
  {
    // if there is no sync cable between the event cameras,
    // compute the sensor time from ros time and offset
    const uint64_t st =
      sync_on_sensor_time_
        ? sensor_time
        : (ros_compat::to_nanoseconds(t) - time_offset_.getOffset());
    frame_times_.push(FrameTime(st, t));
    process();
  }

  // Set this to true if the event cams are hw synced to each other.
  void setSyncOnSensorTime(bool s) { sync_on_sensor_time_ = s; }
  void processMsg(EventPacketConstSharedPtrT msg)
  {
    if (image_msg_template_.height == 0) {
      image_msg_template_.header = msg->header;
      image_msg_template_.width = msg->width;
      image_msg_template_.height = msg->height;
      image_msg_template_.encoding = "mono8";
      image_msg_template_.is_bigendian = check_endian::isBigEndian();
      image_msg_template_.step = image_msg_template_.width;
      reconstructor_.initialize(
        msg->width, msg->height,
        static_cast<uint32_t>(std::abs(cutoff_num_events_)), fill_ratio_);
      decoder_ = decoder_factory_.getInstance(*msg);
      if (!decoder_) {
        std::cerr << "invalid encoding: " << msg->encoding << std::endl;
        throw(std::runtime_error("invalid encoding!"));
      }
#ifdef RESCALE
      if (scaleFile_.empty()) {
        std::cerr << "WARNING: NO SCALE FILE PROVIDED!" << std::endl;
      } else {
        reconstructor_.readScaleFile(scaleFile_);
      }
#endif
      auto decoder =
        event_camera_codecs::DecoderFactory<EventPacketT>().getInstance(*msg);
      uint64_t first_ts{0};
      bool found_time = decoder->findFirstSensorTime(*msg, &first_ts);
      if (!found_time) {
        std::cout << "WARNING: first message does not contain time stamp!"
                  << std::endl;
      } else {
        updateROSTimeOffset(
          ros_compat::to_nanoseconds(RosTimeT(msg->header.stamp)), first_ts);
      }
    }
    buffered_messages_.push(msg);
    process();
  }

  void process()
  {
    while (!buffered_messages_.empty() && !frame_times_.empty()) {
      auto msg = buffered_messages_.front();
      // this loop will run until the message is completely decoded or
      // the last frame is used up
      uint64_t next_time{0};
      bool message_exhausted(false);
      while (!frame_times_.empty() && !message_exhausted) {
        const auto t0 = std::chrono::high_resolution_clock::now();
        message_exhausted = !decoder_->decodeUntil(
          *msg, this, frame_times_.front().sensor_time, &next_time);
        decode_time_ += std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::high_resolution_clock::now() - t0)
                          .count();
        emitFramesOlderThan(next_time);
      }

      if (message_exhausted) {
        buffered_messages_.pop();
      }
    }
  }

  bool hasValidSensorTimeOffset() const { return (time_offset_.isValid()); }

  // returns  t_ros - t_sensor in nanoseconds
  int64_t getSensorTimeOffset() const { return (time_offset_.getOffset()); }

  void updateROSTimeOffset(uint64_t tros, uint64_t tsens)
  {
    time_offset_.update(tros, tsens);
    update_ros_to_sensor_time_offset_ =
      false;  // only update on first event after stamp
  }

private:
  void emitFramesOlderThan(uint64_t current_time)
  {
    while (!frame_times_.empty() &&
           frame_times_.front().sensor_time <= current_time) {
      const auto & frame_time = frame_times_.front();
      auto msg = std::make_unique<ImageT>(image_msg_template_);
      msg->data.resize(msg->height * msg->step);
      reconstructor_.getImage(&(msg->data[0]), msg->step);
      msg->header.stamp = frame_time.ros_time;
      frame_handler_->frame(frame_time.sensor_time, std::move(msg), topic_);
      if (active_pixels_) {
        auto active_msg = std::make_unique<ImageT>(image_msg_template_);
        active_msg->data.resize(active_msg->height * active_msg->step);
        reconstructor_.getActivePixelImage(
          &(active_msg->data[0]), active_msg->step);
        const double fr = reconstructor_.getCurrentFillRatio();
        const size_t qs = reconstructor_.getCurrentQueueSize();
        active_msg->header.stamp = frame_time.ros_time;
        frame_handler_->activePixels(
          frame_time.sensor_time, std::move(active_msg), topic_, qs, fr);
      }
      frame_times_.pop();
    }
  }

  // ------------------------  variables ------------------------------
  FrameHandler<ImageConstPtrT> * frame_handler_{nullptr};
  std::string topic_;
  bool active_pixels_;
  std::string scale_file_;
  ImageT image_msg_template_;
  int cutoff_num_events_{0};
  double fill_ratio_{0};
  TimeOffset time_offset_;
  bool update_ros_to_sensor_time_offset_{true};
  bool sync_on_sensor_time_{false};
  event_camera_codecs::Decoder<EventPacket, ApproxReconstructor> * decoder_{
    nullptr};
  event_camera_codecs::DecoderFactory<EventPacket, ApproxReconstructor>
    decoder_factory_;
  fibar_lib::ImageReconstructor<tile_size> reconstructor_;
  std::queue<FrameTime> frame_times_;
  std::queue<EventPacketConstSharedPtrT> buffered_messages_;
  uint64_t decode_time_{0};
  uint64_t number_of_events_{0};
};
}  // namespace event_image_reconstruction_fibar
#endif  // EVENT_IMAGE_RECONSTRUCTION_FIBAR__APPROX_RECONSTRUCTOR_HPP_
