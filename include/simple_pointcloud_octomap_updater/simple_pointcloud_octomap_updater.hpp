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
 * - Added low-bandwidth local octomap visualization publisher.
 */

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
#include <atomic>
#include <hector_worldmodel_msgs/srv/get_distance_to_obstacle.hpp>
#include <memory>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace occupancy_map_monitor
{
/* Helpers for the local octomap visualization, exposed for unit testing. */

/** An occupied leaf of an octree: an axis-aligned cube given by center and edge length. */
struct OccupiedBox {
  octomap::point3d center;
  double size;
};

/** Collect all occupied leaves of \p tree whose center lies inside the axis-aligned box
 *  spanned by \p min and \p max. */
std::vector<OccupiedBox> collectOccupiedLeavesBBX( const octomap::OcTree &tree,
                                                   const octomap::point3d &min,
                                                   const octomap::point3d &max );

/** Build an octree with the given \p resolution that marks the given boxes as occupied.
 *  Boxes larger than \p resolution (pruned coarse leaves) are stamped over their full extent. */
std::unique_ptr<octomap::OcTree> buildOccupiedTree( const std::vector<OccupiedBox> &boxes,
                                                    double resolution );

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
  void cloudMsgCallback( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg );
  void publishOctomap();      // Lazy publisher callback (full map)
  void publishLocalOctomap(); // Lazy publisher callback (cropped/coarsened local map)

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time last_update_time_ = rclcpp::Time( 0, 0, RCL_ROS_TIME );

  /* params */
  std::string point_cloud_topic_;
  double min_range_;
  double max_range_;
  double min_range_sq_;
  double max_range_sq_;
  long point_subsample_;
  double max_update_rate_;
  std::string ns_;
  double tf_timeout_{ 0.5 };
  bool publish_service_{ false };

  /* Global octomap visualization params (full map) */
  double global_viz_frequency_{ 0.0 };
  std::string global_viz_topic_{ "global_octomap" };

  /* Local octomap visualization params */
  double local_viz_frequency_{ 0.0 };
  std::string local_viz_topic_{ "local_octomap" };
  std::string local_viz_robot_frame_{ "base_link" };
  double local_viz_range_xy_{ 5.0 };
  double local_viz_range_z_{ 2.0 };
  double local_viz_resolution_{ 0.0 };

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> point_cloud_subscriber_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> point_cloud_filter_;
  octomap::KeyRay key_ray_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<hector_worldmodel_msgs::srv::GetDistanceToObstacle>::SharedPtr distance_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_octomap_service_;
  std::atomic<bool> enabled_{ true };
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  /* Octomap publishers and timers */
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr global_viz_pub_;
  rclcpp::TimerBase::SharedPtr global_viz_timer_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr local_viz_pub_;
  rclcpp::TimerBase::SharedPtr local_viz_timer_;

  rclcpp::Logger logger_;
};
} // namespace occupancy_map_monitor
