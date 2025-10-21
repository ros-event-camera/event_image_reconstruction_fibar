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

#include <memory>
#include <queue>
#include <simple_image_recon_lib/simple_image_reconstructor.hpp>
#include <string>

#include "event_image_reconstruction_fibar/check_endian.hpp"
#include "event_image_reconstruction_fibar/frame_handler.hpp"
#include "event_image_reconstruction_fibar/ros_compat.hpp"
#include "event_image_reconstruction_fibar/time_offset.hpp"

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
    FrameTime(uint64_t s, const RosTimeT & r) : sensorTime(s), rosTime(r) {}
    uint64_t sensorTime{0};
    RosTimeT rosTime;
  };

  using EventPacket = EventPacketT;
  explicit ApproxReconstructor(
    FrameHandler<ImageConstPtrT> * fh, const std::string & topic,
    int cutoffNumEvents = 30, double fillRatio = 0.5, bool activePixels = false,
    const std::string & scaleFile = std::string())
  : frameHandler_(fh),
    topic_(topic),
    activePixels_(activePixels),
    scaleFile_(scaleFile),
    cutoffNumEvents_(cutoffNumEvents),
    fillRatio_(fillRatio)
  {
    imageMsgTemplate_.height = 0;
  }

  // ---------- inherited from EventProcessor
  inline void eventCD(
    uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    numberOfEvents_++;
    if (__builtin_expect(updateROStoSensorTimeOffset_, false)) {
      const auto & msg = bufferedMessages_.front();
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
    simpleReconstructor_.event(t, ex, ey, polarity);
#endif
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override {}
  void rawData(const char *, size_t) override {}
  // --------- end of inherited from EventProcessor

  const auto & getDecodeTime() const { return decodeTime_; }
  const auto & getNumberOfEvents() const { return numberOfEvents_; }

  void addFrameTime(uint64_t sensorTime, const RosTimeT t)
  {
    // if there is no sync cable between the event cameras,
    // compute the sensor time from ros time and offset
    const uint64_t st =
      syncOnSensorTime_
        ? sensorTime
        : (ros_compat::to_nanoseconds(t) - timeOffset_.getOffset());
    frameTimes_.push(FrameTime(st, t));
    process();
  }

  // Set this to true if the event cams are hw synced to each other.
  void setSyncOnSensorTime(bool s) { syncOnSensorTime_ = s; }
  void processMsg(EventPacketConstSharedPtrT msg)
  {
    if (imageMsgTemplate_.height == 0) {
      imageMsgTemplate_.header = msg->header;
      imageMsgTemplate_.width = msg->width;
      imageMsgTemplate_.height = msg->height;
      imageMsgTemplate_.encoding = "mono8";
      imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
      imageMsgTemplate_.step = imageMsgTemplate_.width;
      simpleReconstructor_.initialize(
        msg->width, msg->height,
        static_cast<uint32_t>(std::abs(cutoffNumEvents_)), fillRatio_);
      decoder_ = decoderFactory_.getInstance(*msg);
      if (!decoder_) {
        std::cerr << "invalid encoding: " << msg->encoding << std::endl;
        throw(std::runtime_error("invalid encoding!"));
      }
#ifdef RESCALE
      if (scaleFile_.empty()) {
        std::cerr << "WARNING: NO SCALE FILE PROVIDED!" << std::endl;
      } else {
        simpleReconstructor_.readScaleFile(scaleFile_);
      }
#endif
      auto decoder =
        event_camera_codecs::DecoderFactory<EventPacketT>().getInstance(*msg);
      uint64_t firstTS{0};
      bool foundTime = decoder->findFirstSensorTime(*msg, &firstTS);
      if (!foundTime) {
        std::cout << "WARNING: first message does not contain time stamp!"
                  << std::endl;
      } else {
        updateROSTimeOffset(
          ros_compat::to_nanoseconds(RosTimeT(msg->header.stamp)), firstTS);
      }
    }
    bufferedMessages_.push(msg);
    process();
  }

  void process()
  {
    while (!bufferedMessages_.empty() && !frameTimes_.empty()) {
      auto msg = bufferedMessages_.front();
      // this loop will run until the message is completely decoded or
      // the last frame is used up
      uint64_t nextTime{0};
      bool messageExhausted(false);
      while (!frameTimes_.empty() && !messageExhausted) {
        const auto t0 = std::chrono::high_resolution_clock::now();
        messageExhausted = !decoder_->decodeUntil(
          *msg, this, frameTimes_.front().sensorTime, &nextTime);
        decodeTime_ += std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::high_resolution_clock::now() - t0)
                         .count();
        emitFramesOlderThan(nextTime);
      }

      if (messageExhausted) {
        bufferedMessages_.pop();
      }
    }
  }

  bool hasValidSensorTimeOffset() const { return (timeOffset_.isValid()); }

  // returns  t_ros - t_sensor in nanoseconds
  int64_t getSensorTimeOffset() const { return (timeOffset_.getOffset()); }

  void updateROSTimeOffset(uint64_t tros, uint64_t tsens)
  {
    timeOffset_.update(tros, tsens);
    updateROStoSensorTimeOffset_ =
      false;  // only update on first event after stamp
  }

private:
  void emitFramesOlderThan(uint64_t currentTime)
  {
    while (!frameTimes_.empty() &&
           frameTimes_.front().sensorTime <= currentTime) {
      const auto & frameTime = frameTimes_.front();
      auto msg = std::make_unique<ImageT>(imageMsgTemplate_);
      msg->data.resize(msg->height * msg->step);
      simpleReconstructor_.getImage(&(msg->data[0]), msg->step);
      msg->header.stamp = frameTime.rosTime;
      frameHandler_->frame(frameTime.sensorTime, std::move(msg), topic_);
      if (activePixels_) {
        auto activeMsg = std::make_unique<ImageT>(imageMsgTemplate_);
        activeMsg->data.resize(activeMsg->height * activeMsg->step);
        simpleReconstructor_.getActivePixelImage(
          &(activeMsg->data[0]), activeMsg->step);
        const double fr = simpleReconstructor_.getCurrentFillRatio();
        const size_t qs = simpleReconstructor_.getCurrentQueueSize();
        activeMsg->header.stamp = frameTime.rosTime;
        frameHandler_->activePixels(
          frameTime.sensorTime, std::move(activeMsg), topic_, qs, fr);
      }
      frameTimes_.pop();
    }
  }

  // ------------------------  variables ------------------------------
  FrameHandler<ImageConstPtrT> * frameHandler_{nullptr};
  std::string topic_;
  bool activePixels_;
  std::string scaleFile_;
  ImageT imageMsgTemplate_;
  int cutoffNumEvents_{0};
  double fillRatio_{0};
  TimeOffset timeOffset_;
  bool updateROStoSensorTimeOffset_{true};
  bool syncOnSensorTime_{false};
  event_camera_codecs::Decoder<EventPacket, ApproxReconstructor> * decoder_{
    nullptr};
  event_camera_codecs::DecoderFactory<EventPacket, ApproxReconstructor>
    decoderFactory_;
  simple_image_recon_lib::SimpleImageReconstructor<tile_size>
    simpleReconstructor_;
  std::queue<FrameTime> frameTimes_;
  std::queue<EventPacketConstSharedPtrT> bufferedMessages_;
  uint64_t decodeTime_{0};
  uint64_t numberOfEvents_{0};
};
}  // namespace event_image_reconstruction_fibar
#endif  // EVENT_IMAGE_RECONSTRUCTION_FIBAR__APPROX_RECONSTRUCTOR_HPP_
