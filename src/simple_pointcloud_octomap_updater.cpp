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
 * - New parameters (all under the same namespace as other params):
 *     generate_2d_map          bool    false   Master on/off switch
 *     map_2d_z_min             double  0.0     Lower bound of height slice (m)
 *     map_2d_z_max             double  1.5     Upper bound of height slice (m)
 *     map_2d_topic             string  "map_2d" Published OccupancyGrid topic
 *     map_2d_publish_frequency double  0.0     Hz; 0 = publish on every cloud update
 */

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>
#include <moveit/utils/logger.hpp>
#include <octomap_msgs/conversions.h>
#include <rclcpp/version.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <simple_pointcloud_octomap_updater/simple_pointcloud_octomap_updater.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

namespace occupancy_map_monitor
{

// =============================================================================
// Construction
// =============================================================================

SimplePointCloudOctomapUpdater::SimplePointCloudOctomapUpdater()
    : OccupancyMapUpdater( "PointCloudUpdater" ), min_range_( 0.0 ),
      max_range_( std::numeric_limits<double>::max() ), min_range_sq_( 0.0 ),
      max_range_sq_( std::numeric_limits<double>::infinity() ), point_subsample_( 1 ),
      max_update_rate_( 0 ), point_cloud_subscriber_( nullptr ), point_cloud_filter_( nullptr ),
      logger_( moveit::getLogger( "moveit.ros.pointcloud_octomap_updater" ) )
{
}

// =============================================================================
// setParams  –  reads ROS2 parameters
// =============================================================================

bool SimplePointCloudOctomapUpdater::setParams( const std::string &name_space )
{
  // Optional: Namespace (default "")
  node_->get_parameter_or( name_space + ".ns", ns_, std::string() );

  // Optional: Min Range
  node_->get_parameter_or( name_space + ".min_range", min_range_, min_range_ );
  if ( min_range_ < 0.0 ) min_range_ = 0.0;
  min_range_sq_ = min_range_ * min_range_;

  // Optional: Max Range
  node_->get_parameter_or( name_space + ".max_range", max_range_, max_range_ );
  if ( max_range_ < 0.0 ) max_range_ = 0.0;
  max_range_sq_ = max_range_ * max_range_;

  // Required: Point Subsample
  if ( !node_->get_parameter( name_space + ".point_subsample", point_subsample_ ) ) {
    RCLCPP_ERROR( logger_, "Parameter '%s.point_subsample' is required but not found.",
                  name_space.c_str() );
    return false;
  }
  if ( point_subsample_ < 1 ) {
    RCLCPP_WARN( logger_, "Parameter '%s.point_subsample' must be >= 1. Setting to 1.",
                 name_space.c_str() );
    point_subsample_ = 1;
  }

  // Required: Max Update Rate
  if ( !node_->get_parameter( name_space + ".max_update_rate", max_update_rate_ ) ) {
    RCLCPP_ERROR( logger_, "Parameter '%s.max_update_rate' is required but not found.",
                  name_space.c_str() );
    return false;
  }
  if ( max_update_rate_ < 0.0 ) {
    RCLCPP_WARN( logger_, "Parameter '%s.max_update_rate' must be >= 0.0. Setting to 0.0.",
                 name_space.c_str() );
    max_update_rate_ = 0.0;
  }

  // Required: Point Cloud Topic
  if ( !node_->get_parameter( name_space + ".point_cloud_topic", point_cloud_topic_ ) ) {
    RCLCPP_ERROR( logger_, "Parameter '%s.point_cloud_topic' is required but not found.",
                  name_space.c_str() );
    return false;
  }

  // Optional: TF Timeout
  node_->get_parameter_or( name_space + ".tf_timeout", tf_timeout_, tf_timeout_ );
  if ( tf_timeout_ < 0.0 ) tf_timeout_ = 0.0;

  // Optional: Publish Distance Service
  node_->get_parameter_or( name_space + ".publish_distance_service", publish_service_, false );

  // Optional: OctoMap publisher
  node_->get_parameter_or( name_space + ".publish_frequency", publish_frequency_, 0.0 );
  node_->get_parameter_or( name_space + ".octomap_topic", octomap_topic_,
                           std::string( "octomap_binary" ) );

  // ── 2D map parameters (all optional) ─────────────────────────────────────
  node_->get_parameter_or( name_space + ".generate_2d_map", generate_2d_map_, false );

  if ( generate_2d_map_ ) {
    node_->get_parameter_or( name_space + ".map_2d_z_min", map_2d_z_min_, 0.0 );
    node_->get_parameter_or( name_space + ".map_2d_z_max", map_2d_z_max_, 1.5 );
    node_->get_parameter_or( name_space + ".map_2d_topic", map_2d_topic_,
                             std::string( "map_2d" ) );
    node_->get_parameter_or( name_space + ".map_2d_publish_frequency",
                             map_2d_publish_frequency_, 0.0 );

    if ( map_2d_z_min_ >= map_2d_z_max_ ) {
      RCLCPP_WARN( logger_,
                   "map_2d_z_min (%.2f) >= map_2d_z_max (%.2f) – resetting to [0.0, 1.5].",
                   map_2d_z_min_, map_2d_z_max_ );
      map_2d_z_min_ = 0.0;
      map_2d_z_max_ = 1.5;
    }

    RCLCPP_INFO( logger_,
                 "2D map generation ENABLED  |  slice z=[%.2f, %.2f)  |  topic='%s'  |  freq=%.1f Hz",
                 map_2d_z_min_, map_2d_z_max_, map_2d_topic_.c_str(), map_2d_publish_frequency_ );
  }

  // Sanity-check range pair
  if ( min_range_sq_ >= max_range_sq_ ) {
    min_range_sq_ = 0.0;
    max_range_sq_ = std::numeric_limits<double>::infinity();
  }

  // Distance service
  if ( publish_service_ ) {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "distance_ray_marker", rclcpp::QoS( 10 ) );
    service_callback_group_ =
        node_->create_callback_group( rclcpp::CallbackGroupType::Reentrant );
    distance_service_ =
        node_->create_service<hector_worldmodel_msgs::srv::GetDistanceToObstacle>(
            "get_distance_to_obstacle",
            std::bind( &SimplePointCloudOctomapUpdater::handleGetDistance, this,
                       std::placeholders::_1, std::placeholders::_2 ),
            rclcpp::ServicesQoS(), service_callback_group_ );
  }

  return true;
}

// =============================================================================
// initialize
// =============================================================================

bool SimplePointCloudOctomapUpdater::initialize( const rclcpp::Node::SharedPtr &node )
{
  node_ = node;
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>( node_->get_clock() );
  const auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface() );
  tf_buffer_->setCreateTimerInterface( create_timer_interface );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_ );

  // Enable / disable service
  enable_octomap_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "enable_octomap",
      [this]( const std_srvs::srv::SetBool::Request::SharedPtr req,
              const std_srvs::srv::SetBool::Response::SharedPtr res ) {
        enabled_ = req->data;
        if ( !req->data ) {
          tree_->lockWrite();
          tree_->clear();
          tree_->unlockWrite();
          tree_->triggerUpdateCallback();
          RCLCPP_INFO( logger_, "Octomap disabled and cleared" );
          res->message = "Octomap disabled and cleared";
        } else {
          RCLCPP_INFO( logger_, "Octomap enabled" );
          res->message = "Octomap enabled";
        }
        res->success = true;
      } );

  // OctoMap publisher
  octomap_pub_ = node_->create_publisher<octomap_msgs::msg::Octomap>( octomap_topic_, 1 );
  if ( publish_frequency_ > 0.0 ) {
    publish_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>( 1.0 / publish_frequency_ ),
        std::bind( &SimplePointCloudOctomapUpdater::publishOctomap, this ) );
  }

  // ── 2D map publisher & optional timer ────────────────────────────────────
  if ( generate_2d_map_ ) {
    // Latch-like behaviour: transient_local keeps the last message for late subscribers
    rclcpp::QoS map_qos( 1 );
    map_qos.transient_local();
    map_2d_pub_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>( map_2d_topic_, map_qos );

    if ( map_2d_publish_frequency_ > 0.0 ) {
      map_2d_timer_ = node_->create_wall_timer(
          std::chrono::duration<double>( 1.0 / map_2d_publish_frequency_ ),
          std::bind( &SimplePointCloudOctomapUpdater::publish2DMap, this ) );
      RCLCPP_INFO( logger_, "2D map timer started at %.1f Hz", map_2d_publish_frequency_ );
    }
  }

  return true;
}

// =============================================================================
// start / stop
// =============================================================================

void SimplePointCloudOctomapUpdater::start()
{
  if ( point_cloud_subscriber_ ) return;

  rclcpp::SubscriptionOptions options;
  options.callback_group =
      node_->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );

  auto qos_profile =
#if RCLCPP_VERSION_GTE( 28, 3, 0 )
      rclcpp::SensorDataQoS();
#else
      rmw_qos_profile_sensor_data;
#endif

  point_cloud_subscriber_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
          node_, point_cloud_topic_, qos_profile, options );

  if ( tf_listener_ && tf_buffer_ && !monitor_->getMapFrame().empty() ) {
    point_cloud_filter_ =
        std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            *point_cloud_subscriber_, *tf_buffer_, monitor_->getMapFrame(), 5, node_ );
    point_cloud_filter_->registerCallback(
        [this]( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud ) {
          cloudMsgCallback( cloud );
        } );
    RCLCPP_DEBUG( logger_, "Listening to '%s' using message filter with target frame '%s'",
                  point_cloud_topic_.c_str(),
                  point_cloud_filter_->getTargetFramesString().c_str() );
  } else {
    point_cloud_subscriber_->registerCallback(
        [this]( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud ) {
          cloudMsgCallback( cloud );
        } );
    RCLCPP_DEBUG( logger_, "Listening to '%s'", point_cloud_topic_.c_str() );
  }
}

void SimplePointCloudOctomapUpdater::stop()
{
  point_cloud_filter_.reset();
  point_cloud_subscriber_.reset();
  publish_timer_.reset();
  map_2d_timer_.reset();
  enable_octomap_service_.reset();
  distance_service_.reset();
}

// =============================================================================
// publishOctomap  (unchanged logic, kept for completeness)
// =============================================================================

void SimplePointCloudOctomapUpdater::publishOctomap()
{
  if ( octomap_pub_->get_subscription_count() == 0 ) return;

  octomap_msgs::msg::Octomap msg;
  msg.header.frame_id = monitor_->getMapFrame();
  msg.header.stamp = node_->now();

  tree_->lockRead();
  try {
    if ( octomap_msgs::binaryMapToMsg( *tree_, msg ) ) {
      octomap_pub_->publish( msg );
    }
  } catch ( ... ) {
    RCLCPP_ERROR( logger_, "Error serializing octomap for publishing" );
  }
  tree_->unlockRead();
}

// =============================================================================
// cloudMsgCallback
// =============================================================================

void SimplePointCloudOctomapUpdater::cloudMsgCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg )
{
  RCLCPP_DEBUG( logger_, "Received a new point cloud message" );

  if ( !enabled_ ) return;

  rclcpp::Time start = rclcpp::Clock( RCL_ROS_TIME ).now();

  if ( max_update_rate_ > 0 ) {
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
        tf2::fromMsg(
            tf_buffer_->lookupTransform( monitor_->getMapFrame(), cloud_msg->header.frame_id,
                                         cloud_msg->header.stamp,
                                         rclcpp::Duration::from_seconds( tf_timeout_ ) ),
            map_h_sensor );
      } catch ( tf2::TransformException &ex ) {
        RCLCPP_ERROR_STREAM( logger_, "Transform error of sensor data: " << ex.what()
                                                                         << "; quitting callback" );
        return;
      }
    } else {
      return;
    }
  }

  /* compute sensor origin in map frame */
  const tf2::Vector3 &sensor_origin_tf = map_h_sensor.getOrigin();
  octomap::point3d sensor_origin( sensor_origin_tf.getX(), sensor_origin_tf.getY(),
                                  sensor_origin_tf.getZ() );

  octomap::OcTreeKey dummy_key;
  if ( !tree_->coordToKeyChecked( sensor_origin, dummy_key ) ) {
    RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 5000,
        "Sensor origin (%.2f, %.2f, %.2f) is outside octree bounds, skipping update",
        sensor_origin.x(), sensor_origin.y(), sensor_origin.z() );
    return;
  }

  if ( !updateTransformCache( cloud_msg->header.frame_id, cloud_msg->header.stamp ) ) return;

  octomap::KeySet free_cells, occupied_cells, clip_cells;
  tree_->lockRead();

  try {
    for ( unsigned int row = 0; row < cloud_msg->height; row += point_subsample_ ) {
      unsigned int row_c = row * cloud_msg->width;
      sensor_msgs::PointCloud2ConstIterator<float> pt_iter( *cloud_msg, "x" );
      pt_iter += row_c;

      for ( unsigned int col = 0; col < cloud_msg->width;
            col += point_subsample_, pt_iter += point_subsample_ ) {
        if ( !std::isnan( pt_iter[0] ) && !std::isnan( pt_iter[1] ) &&
             !std::isnan( pt_iter[2] ) ) {

          const auto pt = tf2::Vector3( pt_iter[0], pt_iter[1], pt_iter[2] );
          const auto length2 = pt.length2();
          if ( length2 < min_range_sq_ ) continue;

          tf2::Vector3 point_tf = map_h_sensor * pt;

          if ( length2 > max_range_sq_ ) {
            tf2::Vector3 clipped_point_tf =
                map_h_sensor *
                ( tf2::Vector3( pt_iter[0], pt_iter[1], pt_iter[2] ).normalize() * max_range_ );
            clip_cells.insert( tree_->coordToKey( clipped_point_tf.getX(),
                                                  clipped_point_tf.getY(),
                                                  clipped_point_tf.getZ() ) );
          } else {
            occupied_cells.insert(
                tree_->coordToKey( point_tf.getX(), point_tf.getY(), point_tf.getZ() ) );
          }
        }
      }
    }

    for ( const octomap::OcTreeKey &occupied_cell : occupied_cells ) {
      if ( tree_->computeRayKeys( sensor_origin, tree_->keyToCoord( occupied_cell ), key_ray_ ) )
        free_cells.insert( key_ray_.begin(), key_ray_.end() );
    }
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

  for ( const octomap::OcTreeKey &occupied_cell : occupied_cells )
    free_cells.erase( occupied_cell );

  tree_->lockWrite();
  try {
    for ( const octomap::OcTreeKey &free_cell : free_cells )
      tree_->updateNode( free_cell, false );
    for ( const octomap::OcTreeKey &occupied_cell : occupied_cells )
      tree_->updateNode( occupied_cell, true );
  } catch ( ... ) {
    RCLCPP_ERROR( logger_, "Internal error while updating octree" );
  }
  tree_->unlockWrite();

  RCLCPP_DEBUG( logger_, "Processed point cloud in %lf ms",
                ( node_->now() - start ).seconds() * 1000.0 );

  tree_->triggerUpdateCallback();

  // ── 2D map: publish on every update when no dedicated timer is running ────
  if ( generate_2d_map_ && map_2d_publish_frequency_ <= 0.0 ) {
    publish2DMap();
  }
}

// =============================================================================
// publish2DMap  –  project OctoMap slice → nav_msgs::msg::OccupancyGrid
// =============================================================================

void SimplePointCloudOctomapUpdater::publish2DMap()
{
  // Skip if nobody is listening (avoids expensive tree traversal)
  if ( !map_2d_pub_ || map_2d_pub_->get_subscription_count() == 0 ) return;

  tree_->lockRead();

  const int max_depth = tree_->getTreeDepth();
  const unsigned multires_scale = 1u; // always full resolution (maxDepth == treeDepth)

  // ── Determine XY extent of the tree ──────────────────────────────────────
  double minX, minY, minZ, maxX, maxY, maxZ;
  tree_->getMetricMin( minX, minY, minZ );
  tree_->getMetricMax( maxX, maxY, maxZ );

  if ( minX >= maxX || minY >= maxY ) {
    tree_->unlockRead();
    RCLCPP_DEBUG( logger_, "publish2DMap: tree is empty, skipping." );
    return;
  }

  const octomap::OcTreeKey minKey =
      tree_->coordToKey( octomap::point3d( minX, minY, minZ ), max_depth );
  const octomap::OcTreeKey maxKey =
      tree_->coordToKey( octomap::point3d( maxX, maxY, maxZ ), max_depth );

  const int img_width  = static_cast<int>( ( maxKey[0] - minKey[0] ) / multires_scale + 1 );
  const int img_height = static_cast<int>( ( maxKey[1] - minKey[1] ) / multires_scale + 1 );

  if ( img_width <= 0 || img_height <= 0 ) {
    tree_->unlockRead();
    return;
  }

  // Grid cell state: -1 = unknown, 0 = free, 100 = occupied
  // We use a two-pass approach: first mark free, then mark occupied (occupied wins).
  std::vector<int8_t> grid( static_cast<size_t>( img_width * img_height ), -1 );

  // Helper: map OcTreeKey to 1-D grid index
  auto gridIdx = [&]( const octomap::OcTreeKey &key ) -> int {
    const int col = ( key[0] - minKey[0] ) / multires_scale;
    const int row = ( key[1] - minKey[1] ) / multires_scale;
    return img_width * row + col;
  };

  // Helper: fill all cells covered by a multi-res node
  auto fillCells = [&]( const octomap::OcTree::iterator &it, int8_t value ) {
    if ( it.getDepth() == static_cast<unsigned>( max_depth ) ) {
      const int idx = gridIdx( it.getKey() );
      if ( idx >= 0 && idx < static_cast<int>( grid.size() ) ) {
        // Occupied always overrides free
        if ( value == 100 || grid[idx] != 100 ) {
          grid[idx] = value;
        }
      }
    } else {
      const int int_size = 1 << ( max_depth - static_cast<int>( it.getDepth() ) );
      const octomap::OcTreeKey min_k = it.getIndexKey();
      for ( int dx = 0; dx < int_size; ++dx ) {
        const int col = ( min_k[0] + dx - minKey[0] ) / multires_scale;
        for ( int dy = 0; dy < int_size; ++dy ) {
          const int row = ( min_k[1] + dy - minKey[1] ) / multires_scale;
          const int idx = img_width * row + col;
          if ( idx >= 0 && idx < static_cast<int>( grid.size() ) ) {
            if ( value == 100 || grid[idx] != 100 ) {
              grid[idx] = value;
            }
          }
        }
      }
    }
  };

  // ── Single pass over the tree ─────────────────────────────────────────────
  for ( auto it = tree_->begin( max_depth ), end = tree_->end(); it != end; ++it ) {
    const double z = it.getZ();
    if ( z < map_2d_z_min_ || z >= map_2d_z_max_ ) continue;

    if ( tree_->isNodeOccupied( *it ) ) {
      fillCells( it, 100 );
    } else {
      fillCells( it, 0 );
    }
  }

  tree_->unlockRead();

  // ── Fill the OccupancyGrid message ────────────────────────────────────────
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.stamp    = node_->now();
  map_msg.header.frame_id = monitor_->getMapFrame();

  map_msg.info.resolution = static_cast<float>( tree_->getResolution() );
  map_msg.info.width      = static_cast<uint32_t>( img_width );
  map_msg.info.height     = static_cast<uint32_t>( img_height );

  // Origin: bottom-left corner of the grid in world coordinates
  map_msg.info.origin.position.x =
      tree_->keyToCoord( minKey[0] ) - tree_->getResolution() * 0.5;
  map_msg.info.origin.position.y =
      tree_->keyToCoord( minKey[1] ) - tree_->getResolution() * 0.5;
  map_msg.info.origin.position.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0;  // identity quaternion

  map_msg.data = grid;

  map_2d_pub_->publish( map_msg );

  RCLCPP_DEBUG( logger_, "Published 2D map (%d x %d, z=[%.2f, %.2f))",
                img_width, img_height, map_2d_z_min_, map_2d_z_max_ );
}

// =============================================================================
// handleGetDistance
// =============================================================================

void SimplePointCloudOctomapUpdater::handleGetDistance(
    const hector_worldmodel_msgs::srv::GetDistanceToObstacle::Request::SharedPtr req,
    const hector_worldmodel_msgs::srv::GetDistanceToObstacle::Response::SharedPtr res )
{
  tf2::Stamped<tf2::Transform> map_h_sensor;
  if ( monitor_->getMapFrame() == req->point.header.frame_id ) {
    map_h_sensor.setIdentity();
  } else {
    if ( tf_buffer_ ) {
      try {
        tf2::fromMsg(
            tf_buffer_->lookupTransform( monitor_->getMapFrame(), req->point.header.frame_id,
                                         req->point.header.stamp ),
            map_h_sensor );
      } catch ( tf2::TransformException &ex ) {
        RCLCPP_ERROR_STREAM( logger_, "Transform error of sensor data: " << ex.what()
                                                                         << "; quitting callback" );
        return;
      }
    } else {
      return;
    }
  }

  tf2::Vector3 direction_tf =
      map_h_sensor.getBasis() *
      tf2::Vector3( req->point.point.x, req->point.point.y, req->point.point.z );

  octomap::point3d sensor_origin( map_h_sensor.getOrigin().getX(),
                                  map_h_sensor.getOrigin().getY(),
                                  map_h_sensor.getOrigin().getZ() );
  octomap::point3d direction( direction_tf.getX(), direction_tf.getY(), direction_tf.getZ() );
  octomap::point3d end_ray;
  bool hit = tree_->castRay( sensor_origin, direction, end_ray );

  res->distance = hit ? ( sensor_origin - end_ray ).norm() : -1.0;
  res->end_point.header.frame_id = monitor_->getMapFrame();
  res->end_point.header.stamp    = node_->now();
  res->end_point.point.x         = end_ray.x();
  res->end_point.point.y         = end_ray.y();
  res->end_point.point.z         = end_ray.z();

  geometry_msgs::msg::Point start_point;
  start_point.x = map_h_sensor.getOrigin().getX();
  start_point.y = map_h_sensor.getOrigin().getY();
  start_point.z = map_h_sensor.getOrigin().getZ();
  publishMarker( start_point, res->end_point.point );
}

// =============================================================================
// publishMarker
// =============================================================================

void SimplePointCloudOctomapUpdater::publishMarker( const geometry_msgs::msg::Point &start,
                                                    const geometry_msgs::msg::Point &end ) const
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = monitor_->getMapFrame();
  m.header.stamp    = node_->now();
  m.ns              = "get_distance_to_obstacle";
  m.id              = 0;
  m.type            = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action          = visualization_msgs::msg::Marker::ADD;
  m.scale.x         = 0.02;
  m.color.r         = 1.0f;
  m.color.g         = 0.0f;
  m.color.b         = 0.0f;
  m.color.a         = 1.0f;
  m.points.push_back( start );
  m.points.push_back( end );
  marker_pub_->publish( m );
}

// =============================================================================
// Shape exclusion stubs (not used)
// =============================================================================

ShapeHandle SimplePointCloudOctomapUpdater::excludeShape( const shapes::ShapeConstPtr &shape )
{
  (void)shape;
  return ShapeHandle();
}

void SimplePointCloudOctomapUpdater::forgetShape( ShapeHandle handle ) { (void)handle; }

} // namespace occupancy_map_monitor