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

#include <shisen_gazebo_plugins/camera_plugin.hpp>
#include <gazebo_ros/node.hpp>

#include <memory>
#include <string>

namespace shisen_gazebo_plugins
{

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
      "Node initialized with name " << node->get_name() << "!");
  }

  // Initialize the mat provider
  {
    shisen_opencv::MatProvider::Options options;
    options.compression_quality = sdf->Get<int>("compression_quality", -1).first;

    RCLCPP_INFO_STREAM(
      node->get_logger(), "\nUsing the following parameters:" <<
        "\n- compression_quality\t: " << options.compression_quality);

    mat_provider = std::make_shared<shisen_opencv::MatProvider>(node, options);
  }
}

void CameraPlugin::OnNewFrame(
  const unsigned char * image, unsigned int width, unsigned int height,
  unsigned int depth, const std::string & format)
{
  gazebo::CameraPlugin::OnNewFrame(image, width, height, depth, format);

  // Determine the mat type from the depth size
  auto type = CV_8UC1;
  if (depth == 2) {
    type = CV_8UC2;
  } else if (depth == 3) {
    type = CV_8UC3;
  } else if (depth == 4) {
    type = CV_8UC4;
  }

  // Create a mat from the new frame data
  cv::Mat mat(height, width, type);

  // Copy the mat data from the new frame data
  memcpy(mat.data, image, width * height * depth);

  if (mat_provider) {
    mat_provider->set_mat(mat);
  }
}

GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

}  // namespace shisen_gazebo_plugins
