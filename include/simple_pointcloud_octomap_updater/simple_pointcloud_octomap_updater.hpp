/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Jon Binney, Ioan Sucan */

#pragma once

#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#if RCLCPP_VERSION_GTE( 28, 3, 3 ) // Rolling
  #include <message_filters/subscriber.hpp>
#else
  #include <message_filters/subscriber.h>
#endif
#include <moveit/occupancy_map_monitor/occupancy_map_updater.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace occupancy_map_monitor
{
class SimplePointCloudOctomapUpdater : public OccupancyMapUpdater
{
public:
  SimplePointCloudOctomapUpdater();
  ~SimplePointCloudOctomapUpdater() override { };

  bool setParams( const std::string &name_space ) override;

  bool initialize( const rclcpp::Node::SharedPtr &node ) override;
  void start() override;
  void stop() override;
  ShapeHandle excludeShape( const shapes::ShapeConstPtr &shape ) override;
  void forgetShape( ShapeHandle handle ) override;

private:
  void cloudMsgCallback( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg );

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Initialize clock type to RCL_ROS_TIME to prevent exception about time sources mismatch
  rclcpp::Time last_update_time_ = rclcpp::Time( 0, 0, RCL_ROS_TIME );

  /* params */
  std::string point_cloud_topic_;
  double min_range_sq_;
  double max_range_sq_;
  double max_range_;
  unsigned int point_subsample_;
  double max_update_rate_;
  std::string ns_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> *point_cloud_subscriber_;
  tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> *point_cloud_filter_;
  /* used to store all cells in the map which a given ray passes through during raycasting.
     we cache this here because it dynamically pre-allocates a lot of memory in its constructor */
  octomap::KeyRay key_ray_;

  rclcpp::Logger logger_;
};
} // namespace occupancy_map_monitor