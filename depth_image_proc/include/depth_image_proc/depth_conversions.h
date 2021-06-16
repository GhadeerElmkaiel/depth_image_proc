/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS
#define DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS

#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>

#include <limits>

namespace depth_image_proc {

typedef sensor_msgs::PointCloud2 PointCloud;

// Handles float or uint16 depths
template<typename T>
void convert(
    const sensor_msgs::ImageConstPtr& depth_msg,
    PointCloud::Ptr& cloud_msg,
    const image_geometry::PinholeCameraModel& model,
    double range_max = 0.0)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);

  std::vector<std::array<float, 3>> pointsVector;

  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        if (range_max != 0.0)
        {
          depth = DepthTraits<T>::fromMeters(range_max);
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }
      std::array<float, 3> point_3d;

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
      point_3d = {(u - center_x) * depth * constant_x, (v - center_y) * depth * constant_y, DepthTraits<T>::toMeters(depth)};
      pointsVector.push_back(point_3d);
    }
  }

  int n_points = pointsVector.size();
  sensor_msgs::PointCloud2 cloud_msg_2;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg_2);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(n_points);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(cloud_msg_2, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(cloud_msg_2, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(cloud_msg_2, "z");

  cloud_msg_2.height = 1;
  cloud_msg_2.width = n_points;
  // cloud_msg_2.header.frame_id = cloud_msg->header.frame_id;
  // cloud_msg_2.header.seq = cloud_msg->header.seq;
  // cloud_msg_2.header.stamp = cloud_msg->header.stamp;

  for(size_t i=0; i<n_points; ++i, ++iter_x_, ++iter_y_, ++iter_z_){
      *iter_x_ = pointsVector[i][0];
      *iter_y_ = pointsVector[i][1];
      *iter_z_ = pointsVector[i][2];

      // cerr << *iter_x << " " << *iter_y << " " << *iter_z << endl;
  }
  cloud_msg->data = cloud_msg_2.data;
  cloud_msg->height = cloud_msg_2.height;
  cloud_msg->width = cloud_msg_2.width;
  cloud_msg->is_dense = true;
}

} // namespace depth_image_proc

#endif
