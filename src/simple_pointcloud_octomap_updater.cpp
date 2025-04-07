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
    : OccupancyMapUpdater( "PointCloudUpdater" ), scale_( 1.0 ), padding_( 0.0 ),
      max_range_( std::numeric_limits<double>::infinity() ), point_subsample_( 1 ),
      max_update_rate_( 0 ), point_cloud_subscriber_( nullptr ), point_cloud_filter_( nullptr ),
      logger_( moveit::getLogger( "moveit.ros.pointcloud_octomap_updater" ) )
{
}

bool SimplePointCloudOctomapUpdater::setParams( const std::string &name_space )
{
  // This parameter is optional
  node_->get_parameter_or( name_space + ".ns", ns_, std::string() );
  return node_->get_parameter( name_space + ".point_cloud_topic", point_cloud_topic_ ) &&
         node_->get_parameter( name_space + ".max_range", max_range_ ) &&
         node_->get_parameter( name_space + ".padding_offset", padding_ ) &&
         node_->get_parameter( name_space + ".padding_scale", scale_ ) &&
         node_->get_parameter( name_space + ".point_subsample", point_subsample_ ) &&
         node_->get_parameter( name_space + ".max_update_rate", max_update_rate_ );
}

bool SimplePointCloudOctomapUpdater::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>( node_->get_clock() );
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface() );
  tf_buffer_->setCreateTimerInterface( create_timer_interface );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

  return true;
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

  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells;
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

          tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3( pt_iter[0], pt_iter[1], pt_iter[2] );
          occupied_cells.insert(
              tree_->coordToKey( point_tf.getX(), point_tf.getY(), point_tf.getZ() ) );
        }
      }
    }

    /* compute the free cells along each ray that ends at an occupied cell */
    for ( const octomap::OcTreeKey &occupied_cell : occupied_cells ) {
      if ( tree_->computeRayKeys( sensor_origin, tree_->keyToCoord( occupied_cell ), key_ray_ ) )
        free_cells.insert( key_ray_.begin(), key_ray_.end() );
    }

    /* compute the free cells along each ray that ends at a model cell */
    for ( const octomap::OcTreeKey &model_cell : model_cells ) {
      if ( tree_->computeRayKeys( sensor_origin, tree_->keyToCoord( model_cell ), key_ray_ ) )
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

  /* cells that overlap with the model are not occupied */
  for ( const octomap::OcTreeKey &model_cell : model_cells ) occupied_cells.erase( model_cell );

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

    // set the logodds to the minimum for the cells that are part of the model
    const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for ( const octomap::OcTreeKey &model_cell : model_cells ) tree_->updateNode( model_cell, lg );
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