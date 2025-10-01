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

#ifndef EVENT_IMAGE_RECONSTRUCTION_FIBAR__UTILS_HPP_
#define EVENT_IMAGE_RECONSTRUCTION_FIBAR__UTILS_HPP_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>

#include "event_image_reconstruction_fibar/ros_compat.hpp"

namespace event_image_reconstruction_fibar
{
namespace utils
{
template <typename MsgPtrT, class RosTimeT>
bool findNextFrameTime(
  MsgPtrT m, uint64_t * nextSensorFrameTime, RosTimeT * nextROSFrameTime,
  uint64_t sliceInterval)
{
  event_camera_codecs::DecoderFactory<typename MsgPtrT::element_type>
    decoderFactory;
  auto decoder = decoderFactory.getInstance(*m);
  if (!decoder) {
    std::cout << "invalid encoding: " << m->encoding << std::endl;
    throw(std::runtime_error("invalid encoding!"));
  }
  uint64_t firstSensorTime{0};
  if (decoder->findFirstSensorTime(*m, &firstSensorTime)) {
    *nextSensorFrameTime = (firstSensorTime / sliceInterval) * sliceInterval;
    *nextROSFrameTime =
      RosTimeT(m->header.stamp) +
      ros_compat::duration_from_nanoseconds(firstSensorTime % sliceInterval);
    return (true);
  }
  std::cout << "WARNING: no time stamp found in packet!" << std::endl;
  return (false);
}

}  // namespace utils
}  // namespace event_image_reconstruction_fibar
#endif  // EVENT_IMAGE_RECONSTRUCTION_FIBAR__UTILS_HPP_
