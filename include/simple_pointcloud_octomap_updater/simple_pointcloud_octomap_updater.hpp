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
/* Modified by Aljoscha Schmidt:
 * - Added GetDistanceToObstacle service.
 * - Removed internal self-filtering (assumes input point cloud is already self-filtered).
 */
/* Modified (2D map integration):
 * - Added optional 2D OccupancyGrid generation from OctoMap height slice.
 *   Activated via parameter `generate_2d_map` (bool, default false).
 */

#pragma once

#include <atomic>
#include <memory>

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

#include <hector_worldmodel_msgs/srv/get_distance_to_obstacle.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace occupancy_map_monitor
{

class SimplePointCloudOctomapUpdater : public OccupancyMapUpdater
{
public:
  SimplePointCloudOctomapUpdater();
  ~SimplePointCloudOctomapUpdater() override = default;

  bool setParams( const std::string &name_space ) override;
  bool initialize( const rclcpp::Node::SharedPtr &node ) override;
  void start() override;
  void stop() override;
  ShapeHandle excludeShape( const shapes::ShapeConstPtr &shape ) override;
  void forgetShape( ShapeHandle handle ) override;

  void handleGetDistance(
      const hector_worldmodel_msgs::srv::GetDistanceToObstacle::Request::SharedPtr req,
      const hector_worldmodel_msgs::srv::GetDistanceToObstacle::Response::SharedPtr res );

  void publishMarker( const geometry_msgs::msg::Point &start,
                      const geometry_msgs::msg::Point &end ) const;

private:
  // ── Point-cloud callback ────────────────────────────────────────────────
  void cloudMsgCallback( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg );

  // ── OctoMap publishing ──────────────────────────────────────────────────
  void publishOctomap();

  // ── 2D map generation & publishing ─────────────────────────────────────
  /**
   * Project the OctoMap voxels whose Z lies in [map_2d_z_min_, map_2d_z_max_) onto a
   * nav_msgs::msg::OccupancyGrid and publish it.
   *
   * Occupied cells → 100, free cells → 0, never-observed cells → -1 (unknown).
   * Called from cloudMsgCallback (if no timer) or from map_2d_timer_ (if frequency > 0).
   */
  void publish2DMap();

  // ── Node / TF ───────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time last_update_time_{ 0, 0, RCL_ROS_TIME };

  // ── Core params ─────────────────────────────────────────────────────────
  std::string point_cloud_topic_;
  double min_range_{ 0.0 };
  double max_range_{ std::numeric_limits<double>::max() };
  double min_range_sq_{ 0.0 };
  double max_range_sq_{ std::numeric_limits<double>::infinity() };
  long point_subsample_{ 1 };
  double max_update_rate_{ 0.0 };
  std::string ns_;
  double tf_timeout_{ 0.5 };
  bool publish_service_{ false };

  // ── OctoMap publisher params ────────────────────────────────────────────
  double publish_frequency_{ 0.0 };
  std::string octomap_topic_{ "octomap_binary" };

  // ── 2D map params ───────────────────────────────────────────────────────
  bool generate_2d_map_{ false };       ///< Master switch – read from param `generate_2d_map`
  double map_2d_z_min_{ 0.0 };         ///< Lower Z bound of the slice (metres)
  double map_2d_z_max_{ 1.5 };         ///< Upper Z bound of the slice (metres)
  std::string map_2d_topic_{ "map_2d" }; ///< Topic name for the OccupancyGrid
  double map_2d_publish_frequency_{ 0.0 }; ///< 0 = publish on every cloud update

  // ── Subscriptions / Filters ─────────────────────────────────────────────
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> point_cloud_subscriber_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> point_cloud_filter_;
  octomap::KeyRay key_ray_;

  // ── Services ────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<hector_worldmodel_msgs::srv::GetDistanceToObstacle>::SharedPtr distance_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_octomap_service_;
  std::atomic<bool> enabled_{ true };
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // ── Publishers & Timers ─────────────────────────────────────────────────
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_2d_pub_;
  rclcpp::TimerBase::SharedPtr map_2d_timer_;

  // ── Logger ──────────────────────────────────────────────────────────────
  rclcpp::Logger logger_;
};

} // namespace occupancy_map_monitor