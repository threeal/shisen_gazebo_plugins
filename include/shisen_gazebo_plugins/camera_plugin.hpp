// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef SHISEN_GAZEBO_PLUGINS__CAMERA_PLUGIN_HPP_
#define SHISEN_GAZEBO_PLUGINS__CAMERA_PLUGIN_HPP_

#include <gazebo/plugins/CameraPlugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <shisen_interfaces/msg/raw_image.hpp>
#include <shisen_interfaces/msg/compressed_image.hpp>

#include <string>

namespace shisen_gazebo_plugins
{

class CameraPlugin : public gazebo::CameraPlugin
{
public:
  CameraPlugin();

protected:
  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

  void OnNewFrame(
    const unsigned char * image, unsigned int width, unsigned int height,
    unsigned int depth, const std::string & format) override;

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<shisen_interfaces::msg::RawImage>::SharedPtr
    raw_image_publisher;

  rclcpp::Publisher<shisen_interfaces::msg::CompressedImage>::SharedPtr
    compressed_image_publisher;
};

}  // namespace shisen_gazebo_plugins

#endif  // SHISEN_GAZEBO_PLUGINS__CAMERA_PLUGIN_HPP_
