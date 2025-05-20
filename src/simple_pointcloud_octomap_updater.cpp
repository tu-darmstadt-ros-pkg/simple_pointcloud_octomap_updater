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

#include <cmath>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>
#include <simple_pointcloud_octomap_updater/simple_pointcloud_octomap_updater.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// TODO: Remove conditional includes when released to all active distros.
#if __has_include( <tf2/LinearMath/Vector3.hpp>)
  #include <tf2/LinearMath/Vector3.hpp>
#else
  #include <tf2/LinearMath/Vector3.h>
#endif
#if __has_include( <tf2/LinearMath/Transform.hpp>)
  #include <tf2/LinearMath/Transform.hpp>
#else
  #include <tf2/LinearMath/Transform.h>
#endif
#include <moveit/utils/logger.hpp>
#include <rclcpp/version.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

#include <memory>

namespace occupancy_map_monitor
{
SimplePointCloudOctomapUpdater::SimplePointCloudOctomapUpdater()
    : OccupancyMapUpdater( "PointCloudUpdater" ), min_range_sq_( 0.0 ),
      max_range_sq_( std::numeric_limits<double>::infinity() ),
      max_range_( std::numeric_limits<double>::max() ), point_subsample_( 1 ),
      max_update_rate_( 0 ), point_cloud_subscriber_( nullptr ), point_cloud_filter_( nullptr ),
      logger_( moveit::getLogger( "moveit.ros.pointcloud_octomap_updater" ) )
{
}

bool SimplePointCloudOctomapUpdater::setParams( const std::string &name_space )
{
  // These parameters are optional
  node_->get_parameter_or( name_space + ".ns", ns_, std::string() );
  node_->get_parameter_or( name_space + ".min_range", min_range_sq_, 0.0 );
  node_->get_parameter_or( name_space + ".max_range", max_range_,
                           std::numeric_limits<double>::infinity() );
  // These parameters are required
  bool ok = node_->get_parameter_or( name_space + ".point_subsample", point_subsample_ );
  ok &= node_->get_parameter( name_space + ".max_update_rate", max_update_rate_ );
  ok &= node_->get_parameter( name_space + ".point_cloud_topic", point_cloud_topic_ );
  // Internally we use squared distances
  min_range_sq_ *= min_range_sq_;
  max_range_sq_ = max_range_ * max_range_;
  if ( min_range_sq_ >= max_range_sq_ ) {
    RCLCPP_ERROR( logger_, "Minimum range (%f) must be less than maximum range (%f)", min_range_sq_,
                  max_range_sq_ );
    min_range_sq_ = 0.0;
  }
  return ok;
}

bool SimplePointCloudOctomapUpdater::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>( node_->get_clock() );
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface() );
  tf_buffer_->setCreateTimerInterface( create_timer_interface );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>( "distance_ray_marker",
                                                                          rclcpp::QoS( 10 ) );
  distance_service_ = node_->create_service<hector_worldmodel_msgs::srv::GetDistanceToObstacle>(
      "get_distance_to_obstacle", std::bind( &SimplePointCloudOctomapUpdater::handleGetDistance,
                                             this, std::placeholders::_1, std::placeholders::_2 ) );

  return true;
}

void SimplePointCloudOctomapUpdater::handleGetDistance(
    const hector_worldmodel_msgs::srv::GetDistanceToObstacle::Request::SharedPtr req,
    const hector_worldmodel_msgs::srv::GetDistanceToObstacle::Response::SharedPtr res )
{
  // get position of point stamped header frame in map frame
  tf2::Stamped<tf2::Transform> map_h_sensor;
  if ( monitor_->getMapFrame() == req->point.header.frame_id ) {
    map_h_sensor.setIdentity();
  } else {
    if ( tf_buffer_ ) {
      try {
        tf2::fromMsg( tf_buffer_->lookupTransform( monitor_->getMapFrame(), req->point.header.frame_id,
                                                   req->point.header.stamp ),
                      map_h_sensor );
      } catch ( tf2::TransformException &ex ) {
        RCLCPP_ERROR_STREAM( logger_, "Transform error of sensor data: " << ex.what()
                                                                         << "; quitting callback" );
        return;
      }
    } else
      return;
  }
  // transform given direction in point msg to octomap frame
  tf2::Vector3 direction_tf =
      map_h_sensor.getBasis() *
      tf2::Vector3( req->point.point.x, req->point.point.y, req->point.point.z );
  // cast ray from sensor origin in the given direction
  octomap::point3d sensor_origin( map_h_sensor.getOrigin().getX(), map_h_sensor.getOrigin().getY(),
                                  map_h_sensor.getOrigin().getZ() );
  octomap::point3d direction( direction_tf.getX(), direction_tf.getY(), direction_tf.getZ() );
  direction.normalize(); // Normalize the direction vector
  octomap::point3d end_ray;
  bool hit = tree_->castRay( sensor_origin, direction, end_ray );

  if ( hit ) {
    // Compute distance to end ray if an obstacle is hit
    res->distance = ( sensor_origin - end_ray ).norm();
  } else {
    // No obstacle hit: set distance to -1
    res->distance = -1;
    end_ray = { 0, 0, 0 };
  }
  res->end_point.header.frame_id = monitor_->getMapFrame();
  res->end_point.header.stamp = node_->now();
  res->end_point.point.x = end_ray.x();
  res->end_point.point.y = end_ray.y();
  res->end_point.point.z = end_ray.z();

  // Publish marker
  geometry_msgs::msg::Point start_point;
  start_point.x = map_h_sensor.getOrigin().getX();
  start_point.y = map_h_sensor.getOrigin().getY();
  start_point.z = map_h_sensor.getOrigin().getZ();
  publishMarker( start_point, res->end_point.point );
}

void SimplePointCloudOctomapUpdater::publishMarker( const geometry_msgs::msg::Point &start,
                                                    const geometry_msgs::msg::Point &end )
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = monitor_->getMapFrame(); // octomap frame
  m.header.stamp = node_->now();
  m.ns = "get_distance_to_obstacle";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  // line width
  m.scale.x = 0.02;

  // color it red
  m.color.r = 1.0f;
  m.color.g = 0.0f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;

  m.points.push_back( start );
  m.points.push_back( end );

  // publish
  marker_pub_->publish( m );
}

void SimplePointCloudOctomapUpdater::start()
{
  std::string prefix = "";
  if ( !ns_.empty() )
    prefix = ns_ + "/";

  if ( point_cloud_subscriber_ )
    return;

  rclcpp::SubscriptionOptions options;
  options.callback_group =
      node_->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );
  /* subscribe to point cloud topic using tf filter*/
  auto qos_profile =
#if RCLCPP_VERSION_GTE( 28, 3, 0 )
      rclcpp::SensorDataQoS();
#else
      rmw_qos_profile_sensor_data;
#endif
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(
      node_, point_cloud_topic_, qos_profile, options );
  if ( tf_listener_ && tf_buffer_ && !monitor_->getMapFrame().empty() ) {
    point_cloud_filter_ = new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(
        *point_cloud_subscriber_, *tf_buffer_, monitor_->getMapFrame(), 5, node_ );
    point_cloud_filter_->registerCallback(
        [this]( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud ) {
          cloudMsgCallback( cloud );
        } );
    RCLCPP_INFO( logger_, "Listening to '%s' using message filter with target frame '%s'",
                 point_cloud_topic_.c_str(), point_cloud_filter_->getTargetFramesString().c_str() );
  } else {
    point_cloud_subscriber_->registerCallback(
        [this]( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud ) {
          cloudMsgCallback( cloud );
        } );
    RCLCPP_INFO( logger_, "Listening to '%s'", point_cloud_topic_.c_str() );
  }
}

void SimplePointCloudOctomapUpdater::stop()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
  point_cloud_filter_ = nullptr;
  point_cloud_subscriber_ = nullptr;
}

void SimplePointCloudOctomapUpdater::cloudMsgCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg )
{
  RCLCPP_DEBUG( logger_, "Received a new point cloud message" );
  rclcpp::Time start = rclcpp::Clock( RCL_ROS_TIME ).now();

  if ( max_update_rate_ > 0 ) {
    // ensure we are not updating the octomap representation too often
    if ( ( node_->now() - last_update_time_ ) <=
         rclcpp::Duration::from_seconds( 1.0 / max_update_rate_ ) )
      return;
    last_update_time_ = node_->now();
  }

  if ( monitor_->getMapFrame().empty() )
    monitor_->setMapFrame( cloud_msg->header.frame_id );

  /* get transform for cloud into map frame */
  tf2::Stamped<tf2::Transform> map_h_sensor;
  if ( monitor_->getMapFrame() == cloud_msg->header.frame_id ) {
    map_h_sensor.setIdentity();
  } else {
    if ( tf_buffer_ ) {
      try {
        tf2::fromMsg( tf_buffer_->lookupTransform( monitor_->getMapFrame(), cloud_msg->header.frame_id,
                                                   cloud_msg->header.stamp ),
                      map_h_sensor );
      } catch ( tf2::TransformException &ex ) {
        RCLCPP_ERROR_STREAM( logger_, "Transform error of sensor data: " << ex.what()
                                                                         << "; quitting callback" );
        return;
      }
    } else
      return;
  }

  /* compute sensor origin in map frame */
  const tf2::Vector3 &sensor_origin_tf = map_h_sensor.getOrigin();
  octomap::point3d sensor_origin( sensor_origin_tf.getX(), sensor_origin_tf.getY(),
                                  sensor_origin_tf.getZ() );
  Eigen::Vector3d sensor_origin_eigen( sensor_origin_tf.getX(), sensor_origin_tf.getY(),
                                       sensor_origin_tf.getZ() );

  if ( !updateTransformCache( cloud_msg->header.frame_id, cloud_msg->header.stamp ) )
    return;

  octomap::KeySet free_cells, occupied_cells, clip_cells;
  tree_->lockRead();

  try {
    /* do ray tracing to find which cells this point cloud indicates should be free, and which it
     * indicates should be occupied */
    for ( unsigned int row = 0; row < cloud_msg->height; row += point_subsample_ ) {
      unsigned int row_c = row * cloud_msg->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter( *cloud_msg, "x" );
      // set iterator to point at start of the current row
      pt_iter += row_c;

      for ( unsigned int col = 0; col < cloud_msg->width;
            col += point_subsample_, pt_iter += point_subsample_ ) {
        // if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        //  continue;

        /* check for NaN */
        if ( !std::isnan( pt_iter[0] ) && !std::isnan( pt_iter[1] ) && !std::isnan( pt_iter[2] ) ) {

          const auto pt = tf2::Vector3( pt_iter[0], pt_iter[1], pt_iter[2] );
          const auto length2 = pt.length2();
          if ( length2 < min_range_sq_ ) {
            continue;
          }
          tf2::Vector3 point_tf = map_h_sensor * pt;

          if ( length2 > max_range_sq_ ) {
            tf2::Vector3 clipped_point_tf =
                map_h_sensor *
                ( tf2::Vector3( pt_iter[0], pt_iter[1], pt_iter[2] ).normalize() * max_range_ );
            clip_cells.insert( tree_->coordToKey( clipped_point_tf.getX(), clipped_point_tf.getY(),
                                                  clipped_point_tf.getZ() ) );
          } else {
            occupied_cells.insert(
                tree_->coordToKey( point_tf.getX(), point_tf.getY(), point_tf.getZ() ) );
          }
        }
      }
    }

    /* compute the free cells along each ray that ends at an occupied cell */
    for ( const octomap::OcTreeKey &occupied_cell : occupied_cells ) {
      if ( tree_->computeRayKeys( sensor_origin, tree_->keyToCoord( occupied_cell ), key_ray_ ) )
        free_cells.insert( key_ray_.begin(), key_ray_.end() );
    }

    /* compute the free cells along each ray that ends at a clipped cell */
    for ( const octomap::OcTreeKey &clip_cell : clip_cells ) {
      free_cells.insert( clip_cell );
      if ( tree_->computeRayKeys( sensor_origin, tree_->keyToCoord( clip_cell ), key_ray_ ) )
        free_cells.insert( key_ray_.begin(), key_ray_.end() );
    }
  } catch ( ... ) {
    tree_->unlockRead();
    return;
  }

  tree_->unlockRead();

  /* occupied cells are not free */
  for ( const octomap::OcTreeKey &occupied_cell : occupied_cells )
    free_cells.erase( occupied_cell );

  tree_->lockWrite();

  try {
    /* mark free cells only if not seen occupied in this cloud */
    for ( const octomap::OcTreeKey &free_cell : free_cells ) tree_->updateNode( free_cell, false );

    /* now mark all occupied cells */
    for ( const octomap::OcTreeKey &occupied_cell : occupied_cells )
      tree_->updateNode( occupied_cell, true );
  } catch ( ... ) {
    RCLCPP_ERROR( logger_, "Internal error while updating octree" );
  }
  tree_->unlockWrite();
  RCLCPP_DEBUG( logger_, "Processed point cloud in %lf ms",
                ( node_->now() - start ).seconds() * 1000.0 );
  tree_->triggerUpdateCallback();
}

ShapeHandle SimplePointCloudOctomapUpdater::excludeShape( const shapes::ShapeConstPtr &shape )
{
  (void)shape;
  return ShapeHandle();
}

void SimplePointCloudOctomapUpdater::forgetShape( ShapeHandle handle ) { (void)handle; }

} // namespace occupancy_map_monitor