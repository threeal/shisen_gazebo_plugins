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

#include "shisen_gazebo_plugins/camera_plugin.hpp"

#include <gazebo_ros/node.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>

namespace shisen_gazebo_plugins
{

const double PI = atan(1) * 4;

CameraPlugin::CameraPlugin()
: gazebo::CameraPlugin()
{
}

void CameraPlugin::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  gazebo::CameraPlugin::Load(sensor, sdf);

  // Initialize the node
  {
    node = gazebo_ros::Node::Get(sdf);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Node initialized with name " << node->get_name() << "!"
    );
  }

  // Initialize the raw image publisher
  {
    using RawImage = shisen_interfaces::msg::RawImage;

    raw_image_publisher = node->create_publisher<RawImage>(
      std::string(node->get_name()) + "/raw_image", 10
    );

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Raw image publisher initialized on " <<
        raw_image_publisher->get_topic_name() << "!"
    );
  }

  // Initialize the compressed image publisher
  {
    using CompressedImage = shisen_interfaces::msg::CompressedImage;

    compressed_image_publisher = node->create_publisher<CompressedImage>(
      std::string(node->get_name()) + "/compressed_image", 10
    );

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Compressed image publisher initialized on " <<
        compressed_image_publisher->get_topic_name() << "!"
    );
  }
}

void CameraPlugin::OnNewFrame(
  const unsigned char * image, unsigned int width, unsigned int height,
  unsigned int depth, const std::string & format)
{
  gazebo::CameraPlugin::OnNewFrame(image, width, height, depth, format);

  // Publish images
  {
    shisen_interfaces::msg::RawImage message;

    message.type = CV_8UC3;
    message.cols = width;
    message.rows = height;

    auto byte_size = width * height * depth;
    message.data.assign(image, image + byte_size);

    raw_image_publisher->publish(message);

    cv::Mat mat_image(
      cv::Size(message.cols, message.rows),
      message.type, message.data.data());

    // Publish compressed image
    {
      shisen_interfaces::msg::CompressedImage message;

      cv::imencode(".jpg", mat_image, message.data, {cv::IMWRITE_JPEG_QUALITY, 50});

      compressed_image_publisher->publish(message);
    }
  }
}

GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

}  // namespace shisen_gazebo_plugins
