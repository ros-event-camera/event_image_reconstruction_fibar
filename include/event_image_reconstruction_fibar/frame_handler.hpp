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

#ifndef EVENT_IMAGE_RECONSTRUCTION_FIBAR__FRAME_HANDLER_HPP_
#define EVENT_IMAGE_RECONSTRUCTION_FIBAR__FRAME_HANDLER_HPP_

namespace event_image_reconstruction_fibar
{
template <typename ImageConstSharedPtrT>
class FrameHandler
{
public:
  virtual void frame(
    uint64_t sensor_time, const ImageConstSharedPtrT & img,
    const std::string & topic) = 0;
  virtual void activePixels(
    uint64_t sensor_time, const ImageConstSharedPtrT & img,
    const std::string & topic, size_t queue_size, double fill_ratio)
  {
    (void)sensor_time;
    (void)img;
    (void)topic;
    (void)queue_size;
    (void)fill_ratio;
  }
  virtual ~FrameHandler() {}
};
}  // namespace event_image_reconstruction_fibar
#endif  // EVENT_IMAGE_RECONSTRUCTION_FIBAR__FRAME_HANDLER_HPP_
