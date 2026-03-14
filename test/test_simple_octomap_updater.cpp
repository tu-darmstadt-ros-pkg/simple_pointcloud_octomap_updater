#include <gtest/gtest.h>

#include <hector_testing_utils/hector_testing_utils.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <hector_worldmodel_msgs/srv/get_distance_to_obstacle.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <simple_pointcloud_octomap_updater/simple_pointcloud_octomap_updater.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

// =============================================================================
// Shared helpers
// =============================================================================

namespace
{

sensor_msgs::msg::PointCloud2 make_point_cloud( const std::string &frame_id,
                                                const rclcpp::Time &stamp,
                                                const std::vector<geometry_msgs::msg::Point> &points )
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp    = stamp;
  msg.height          = 1;
  msg.width           = static_cast<uint32_t>( points.size() );

  sensor_msgs::PointCloud2Modifier modifier( msg );
  modifier.setPointCloud2FieldsByString( 1, "xyz" );
  modifier.resize( points.size() );

  sensor_msgs::PointCloud2Iterator<float> iter_x( msg, "x" );
  sensor_msgs::PointCloud2Iterator<float> iter_y( msg, "y" );
  sensor_msgs::PointCloud2Iterator<float> iter_z( msg, "z" );
  for ( const auto &pt : points ) {
    *iter_x = static_cast<float>( pt.x );
    *iter_y = static_cast<float>( pt.y );
    *iter_z = static_cast<float>( pt.z );
    ++iter_x; ++iter_y; ++iter_z;
  }
  return msg;
}

geometry_msgs::msg::Point make_pt( double x, double y, double z )
{
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

// =============================================================================
// Base test fixture  (identical to the original)
// =============================================================================

class SimpleOctomapUpdaterFixture : public hector_testing_utils::HectorTestFixture
{
protected:
  void SetUp() override
  {
    HectorTestFixture::SetUp();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>( tester_node_->get_clock() );
    monitor_ = std::make_unique<occupancy_map_monitor::OccupancyMapMonitor>(
        tester_node_, tf_buffer_, "map", 0.1 );
    updater_ = std::make_shared<occupancy_map_monitor::SimplePointCloudOctomapUpdater>();
    updater_->setMonitor( monitor_.get() );
    updater_->setTransformCacheCallback(
        []( const std::string &, const rclcpp::Time &,
            occupancy_map_monitor::ShapeTransformCache & ) { return true; } );
    updater_->initialize( tester_node_ );
  }

  void TearDown() override
  {
    if ( updater_ ) updater_->stop();
    updater_.reset();
    monitor_.reset();
    tf_buffer_.reset();
    HectorTestFixture::TearDown();
  }

  template<typename T>
  void declare_or_set_param( const std::string &name, const T &value )
  {
    if ( !tester_node_->has_parameter( name ) ) {
      tester_node_->declare_parameter<T>( name, value );
    } else {
      tester_node_->set_parameter( rclcpp::Parameter( name, value ) );
    }
  }

  void configure_params( const std::string &name_space, const std::string &topic,
                         bool publish_service, int point_subsample = 1, double min_range = 0.0,
                         double max_range = 5.0, double max_update_rate = 0.0 )
  {
    declare_or_set_param( name_space + ".point_cloud_topic", topic );
    declare_or_set_param( name_space + ".point_subsample", point_subsample );
    declare_or_set_param( name_space + ".max_update_rate", max_update_rate );
    declare_or_set_param( name_space + ".min_range", min_range );
    declare_or_set_param( name_space + ".max_range", max_range );
    declare_or_set_param( name_space + ".tf_timeout", 0.1 );
    declare_or_set_param( name_space + ".publish_distance_service", publish_service );
  }

  bool has_node( const octomap::point3d &point ) const
  {
    auto tree = monitor_->getOcTreePtr();
    tree->lockRead();
    auto *node = tree->search( point );
    tree->unlockRead();
    return node != nullptr;
  }

  bool is_occupied( const octomap::point3d &point ) const
  {
    auto tree = monitor_->getOcTreePtr();
    tree->lockRead();
    auto *node = tree->search( point );
    const bool occupied = node && tree->isNodeOccupied( node );
    tree->unlockRead();
    return occupied;
  }

  void spin_for( std::chrono::nanoseconds duration )
  {
    executor_->spin_until( []() { return false; }, duration );
  }

  std::shared_ptr<tf2_ros::Buffer>                                      tf_buffer_;
  std::unique_ptr<occupancy_map_monitor::OccupancyMapMonitor>           monitor_;
  std::shared_ptr<occupancy_map_monitor::SimplePointCloudOctomapUpdater> updater_;
};

// =============================================================================
// 2D-map specific fixture – extends the base with map param helpers
// =============================================================================

class Map2DFixture : public SimpleOctomapUpdaterFixture
{
protected:
  // Default test namespace
  static constexpr const char *kNs    = "simple_octomap";
  static constexpr const char *kTopic = "/test_cloud_2d";

  /// Set all required base params + the 2D map params.
  void configure_with_2d_map( double z_min = 0.0, double z_max = 1.5,
                               const std::string &map_topic = "map_2d",
                               double map_freq = 0.0 )
  {
    configure_params( kNs, kTopic, /*publish_service=*/false );
    declare_or_set_param<bool>( std::string( kNs ) + ".generate_2d_map", true );
    declare_or_set_param<double>( std::string( kNs ) + ".map_2d_z_min", z_min );
    declare_or_set_param<double>( std::string( kNs ) + ".map_2d_z_max", z_max );
    declare_or_set_param<std::string>( std::string( kNs ) + ".map_2d_topic", map_topic );
    declare_or_set_param<double>( std::string( kNs ) + ".map_2d_publish_frequency", map_freq );
  }

  /// Publish a cloud and wait until an OccupancyGrid arrives on `map_topic`.
  std::optional<nav_msgs::msg::OccupancyGrid>
  publish_cloud_and_wait_for_map(
      const std::vector<geometry_msgs::msg::Point> &pts,
      const std::string &map_topic = "map_2d",
      std::chrono::seconds timeout = 5s )
  {
    auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
        kTopic, rclcpp::SensorDataQoS() );
    auto map_sub =
        tester_node_->create_test_subscription<nav_msgs::msg::OccupancyGrid>( map_topic );

    EXPECT_TRUE( pub->wait_for_subscription( *executor_, timeout ) );

    pub->publish( make_point_cloud( "map", tester_node_->now(), pts ) );

    if ( !map_sub->wait_for_message( *executor_, timeout ) ) return std::nullopt;
    return map_sub->last_message();
  }

  /// Look up the OccupancyGrid cell value nearest to (wx, wy).
  /// Returns -2 if the coordinate is out of bounds.
  static int8_t cell_at( const nav_msgs::msg::OccupancyGrid &grid, double wx, double wy )
  {
    const double res = grid.info.resolution;
    const double ox  = grid.info.origin.position.x;
    const double oy  = grid.info.origin.position.y;

    const int col = static_cast<int>( ( wx - ox ) / res );
    const int row = static_cast<int>( ( wy - oy ) / res );

    if ( col < 0 || row < 0 ||
         col >= static_cast<int>( grid.info.width ) ||
         row >= static_cast<int>( grid.info.height ) )
      return -2; // out of bounds sentinel

    return grid.data[static_cast<size_t>( row * grid.info.width + col )];
  }
};

} // anonymous namespace

// =============================================================================
// Original tests (unchanged)
// =============================================================================

TEST_F( SimpleOctomapUpdaterFixture, SetParamsSuccedsWithAllRequiredParams )
{
  const std::string ns = "simple_octomap";
  declare_or_set_param( ns + ".point_cloud_topic", std::string( "/test_cloud" ) );
  declare_or_set_param( ns + ".max_update_rate", 0.0 );
  declare_or_set_param( ns + ".min_range", 0.0 );
  declare_or_set_param( ns + ".max_range", 5.0 );
  declare_or_set_param( ns + ".point_subsample", 1 );
  EXPECT_TRUE( updater_->setParams( ns ) );
}

TEST_F( SimpleOctomapUpdaterFixture, SetParamsFailsWithMissingRequiredParams )
{
  const std::string ns = "simple_octomap";
  declare_or_set_param( ns + ".max_update_rate", 0.0 );
  declare_or_set_param( ns + ".min_range", 0.0 );
  declare_or_set_param( ns + ".max_range", 5.0 );
  declare_or_set_param( ns + ".point_subsample", 1 );
  EXPECT_FALSE( updater_->setParams( ns ) );
}

TEST_F( SimpleOctomapUpdaterFixture, PointCloudMarksOccupiedCell )
{
  const std::string ns    = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( ns, topic, false );
  ASSERT_TRUE( updater_->setParams( ns ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  pub->publish( make_point_cloud( "map", tester_node_->now(), { make_pt( 1.0, 0.0, 0.0 ) } ) );

  EXPECT_TRUE( executor_->spin_until(
      [this]() { return is_occupied( octomap::point3d( 1.0f, 0.0f, 0.0f ) ); }, 5s ) );
}

TEST_F( SimpleOctomapUpdaterFixture, MinRangeSkipsPoints )
{
  const std::string ns    = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( ns, topic, false, 1, 0.5, 5.0, 0.0 );
  ASSERT_TRUE( updater_->setParams( ns ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  pub->publish( make_point_cloud( "map", tester_node_->now(), { make_pt( 0.1, 0.0, 0.0 ) } ) );
  spin_for( 300ms );

  EXPECT_FALSE( has_node( octomap::point3d( 0.1f, 0.0f, 0.0f ) ) );
}

TEST_F( SimpleOctomapUpdaterFixture, PointSubsampleSkipsEveryOtherPoint )
{
  const std::string ns    = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( ns, topic, false, 2, 0.0, 5.0, 0.0 );
  ASSERT_TRUE( updater_->setParams( ns ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  pub->publish( make_point_cloud( "map", tester_node_->now(),
                                  { make_pt( 1.0, 0.0, 0.0 ), make_pt( 2.0, 0.0, 0.0 ) } ) );

  EXPECT_TRUE( executor_->spin_until(
      [this]() { return is_occupied( octomap::point3d( 1.0f, 0.0f, 0.0f ) ); }, 5s ) );
  EXPECT_FALSE( has_node( octomap::point3d( 2.0f, 0.0f, 0.0f ) ) );
}

TEST_F( SimpleOctomapUpdaterFixture, DistanceServiceReturnsNoHitAndMarker )
{
  using Service = hector_worldmodel_msgs::srv::GetDistanceToObstacle;

  const std::string ns    = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( ns, topic, true );
  ASSERT_TRUE( updater_->setParams( ns ) );

  auto client     = tester_node_->create_test_client<Service>( "get_distance_to_obstacle" );
  auto marker_sub = tester_node_->create_test_subscription<visualization_msgs::msg::Marker>(
      "distance_ray_marker" );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  auto request                    = std::make_shared<Service::Request>();
  request->point.header.frame_id  = "map";
  request->point.header.stamp     = tester_node_->now();
  request->point.point.x          = 1.0;
  request->point.point.y          = 0.0;
  request->point.point.z          = 0.0;

  marker_sub->reset();
  auto response = hector_testing_utils::call_service<Service>( client->get(), request, *executor_ );
  ASSERT_NE( response, nullptr );
  EXPECT_LT( response->distance, 0.0 );
  EXPECT_EQ( response->end_point.header.frame_id, "map" );
  ASSERT_TRUE( marker_sub->has_new_message( 0 ) || marker_sub->wait_for_message( *executor_, 5s ) );
  auto marker = marker_sub->last_message();
  ASSERT_TRUE( marker.has_value() );
  EXPECT_EQ( marker->ns, "get_distance_to_obstacle" );
  ASSERT_EQ( marker->points.size(), 2u );
}

TEST_F( SimpleOctomapUpdaterFixture, DistanceServiceHitsOccupiedCell )
{
  using Service = hector_worldmodel_msgs::srv::GetDistanceToObstacle;

  const std::string ns    = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( ns, topic, true );
  ASSERT_TRUE( updater_->setParams( ns ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  auto client     = tester_node_->create_test_client<Service>( "get_distance_to_obstacle" );
  auto marker_sub = tester_node_->create_test_subscription<visualization_msgs::msg::Marker>(
      "distance_ray_marker", rclcpp::QoS( 10 ) );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );
  ASSERT_TRUE( marker_sub->is_connected() );

  pub->publish( make_point_cloud( "map", tester_node_->now(), { make_pt( 1.0, 0.0, 0.0 ) } ) );
  ASSERT_TRUE( executor_->spin_until(
      [this]() { return is_occupied( octomap::point3d( 1.0f, 0.0f, 0.0f ) ); }, 5s ) );

  auto request                   = std::make_shared<Service::Request>();
  request->point.header.frame_id = "map";
  request->point.header.stamp    = tester_node_->now();
  request->point.point.x         = 1.0;
  request->point.point.y         = 0.0;
  request->point.point.z         = 0.0;

  marker_sub->reset();
  auto response = hector_testing_utils::call_service<Service>( client->get(), request, *executor_ );
  ASSERT_NE( response, nullptr );
  EXPECT_GT( response->distance, 0.0 );
  EXPECT_NEAR( response->distance, 1.0, 0.2 );
  EXPECT_EQ( response->end_point.header.frame_id, "map" );
  EXPECT_NEAR( response->end_point.point.x, 1.0, 0.2 );

  ASSERT_TRUE( marker_sub->has_new_message( 0 ) || marker_sub->wait_for_message( *executor_, 5s ) );
  auto marker = marker_sub->last_message();
  ASSERT_TRUE( marker.has_value() );
  EXPECT_EQ( marker->ns, "get_distance_to_obstacle" );
  ASSERT_EQ( marker->points.size(), 2u );
}

// =============================================================================
// 2D map – parameter tests
// =============================================================================

// When generate_2d_map is absent, setParams must still succeed and the flag
// must remain false (feature is opt-in).
TEST_F( Map2DFixture, Generate2DMapDefaultsToFalse )
{
  configure_params( kNs, kTopic, false );
  // Note: do NOT declare generate_2d_map
  ASSERT_TRUE( updater_->setParams( kNs ) );
  // No publisher created → the topic must not exist
  // (we verify indirectly: publishing a cloud must NOT produce a map message)
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      kTopic, rclcpp::SensorDataQoS() );
  auto map_sub =
      tester_node_->create_test_subscription<nav_msgs::msg::OccupancyGrid>( "map_2d" );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  pub->publish( make_point_cloud( "map", tester_node_->now(), { make_pt( 1.0, 0.0, 0.5 ) } ) );
  spin_for( 300ms );

  EXPECT_FALSE( map_sub->has_new_message( 0 ) )
      << "OccupancyGrid must NOT be published when generate_2d_map is false";
}

// All five 2D-map parameters must load without error.
TEST_F( Map2DFixture, Generate2DMapParamsLoadCorrectly )
{
  configure_with_2d_map( /*z_min=*/0.1, /*z_max=*/1.2, "my_map", /*freq=*/5.0 );
  EXPECT_TRUE( updater_->setParams( kNs ) );
}

// If z_min >= z_max, setParams must still succeed (it resets to safe defaults
// with a warning rather than failing).
TEST_F( Map2DFixture, Generate2DMapInvalidZRangeClamped )
{
  configure_with_2d_map( /*z_min=*/2.0, /*z_max=*/0.5 );
  EXPECT_TRUE( updater_->setParams( kNs ) )
      << "setParams must succeed even with inverted z range";
}

// =============================================================================
// 2D map – publication tests
// =============================================================================

// A point cloud with a single occupied voxel inside the slice must produce a
// non-empty OccupancyGrid message on the correct topic.
TEST_F( Map2DFixture, Generate2DMapPublishesGridOnCloudUpdate )
{
  configure_with_2d_map( 0.0, 1.5 );
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  auto result = publish_cloud_and_wait_for_map( { make_pt( 1.0, 1.0, 0.5 ) } );

  ASSERT_TRUE( result.has_value() ) << "Expected an OccupancyGrid to be published";
  EXPECT_EQ( result->header.frame_id, "map" );
  EXPECT_GT( result->info.width,  0u );
  EXPECT_GT( result->info.height, 0u );
  EXPECT_FALSE( result->data.empty() );
}

// Voxels whose Z falls inside [z_min, z_max) must appear as occupied (100) in
// the grid; voxels outside that band must not affect the slice.
TEST_F( Map2DFixture, Generate2DMapOccupiedCellIs100 )
{
  configure_with_2d_map( 0.0, 1.5 );
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  // Point at z = 0.5 – inside the slice
  auto result = publish_cloud_and_wait_for_map( { make_pt( 1.0, 1.0, 0.5 ) } );
  ASSERT_TRUE( result.has_value() );

  const int8_t v = cell_at( *result, 1.0, 1.0 );
  EXPECT_EQ( v, static_cast<int8_t>( 100 ) )
      << "Voxel inside slice should be 100 (occupied)";
}

// A voxel above z_max must not mark the corresponding cell as occupied.
TEST_F( Map2DFixture, Generate2DMapPointAboveSliceIgnored )
{
  configure_with_2d_map( 0.0, 1.0 ); // slice [0, 1)
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  // Point at z = 2.0 – outside (above) the slice
  auto result = publish_cloud_and_wait_for_map( { make_pt( 1.0, 1.0, 2.0 ) } );
  ASSERT_TRUE( result.has_value() );

  const int8_t v = cell_at( *result, 1.0, 1.0 );
  // The cell should NOT be 100; it could be 0 (free from ray-trace) or -1 (unknown)
  EXPECT_NE( v, static_cast<int8_t>( 100 ) )
      << "Voxel above z_max must not appear occupied in the 2D slice";
}

// A voxel below z_min must not mark the corresponding cell as occupied.
TEST_F( Map2DFixture, Generate2DMapPointBelowSliceIgnored )
{
  configure_with_2d_map( 0.5, 1.5 ); // slice [0.5, 1.5)
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  // Point at z = 0.1 – outside (below) the slice
  auto result = publish_cloud_and_wait_for_map( { make_pt( 1.0, 1.0, 0.1 ) } );
  ASSERT_TRUE( result.has_value() );

  const int8_t v = cell_at( *result, 1.0, 1.0 );
  EXPECT_NE( v, static_cast<int8_t>( 100 ) )
      << "Voxel below z_min must not appear occupied in the 2D slice";
}

// Two points at different Z heights but the same XY: the lower one is inside
// the slice, the higher one is not.  The cell must be occupied.
TEST_F( Map2DFixture, Generate2DMapOnlyInSliceVoxelCounts )
{
  configure_with_2d_map( 0.0, 1.0 );
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  auto result = publish_cloud_and_wait_for_map( {
    make_pt( 1.0, 1.0, 0.5 ),  // in slice
    make_pt( 1.0, 1.0, 2.0 ),  // above slice
  } );
  ASSERT_TRUE( result.has_value() );

  EXPECT_EQ( cell_at( *result, 1.0, 1.0 ), static_cast<int8_t>( 100 ) );
}

// The OccupancyGrid's resolution must match the OctoMap resolution (0.1 m in
// the fixture's monitor).
TEST_F( Map2DFixture, Generate2DMapResolutionMatchesOctomap )
{
  configure_with_2d_map();
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  auto result = publish_cloud_and_wait_for_map( { make_pt( 1.0, 0.0, 0.5 ) } );
  ASSERT_TRUE( result.has_value() );

  // OccupancyMapMonitor is initialised with resolution 0.1 in SetUp()
  EXPECT_NEAR( static_cast<double>( result->info.resolution ), 0.1, 1e-4 );
}

// The grid must be large enough to contain the published point.
TEST_F( Map2DFixture, Generate2DMapGridCoversPublishedPoint )
{
  configure_with_2d_map();
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  auto result = publish_cloud_and_wait_for_map( { make_pt( 2.0, 3.0, 0.5 ) } );
  ASSERT_TRUE( result.has_value() );

  EXPECT_NE( cell_at( *result, 2.0, 3.0 ), static_cast<int8_t>( -2 ) )
      << "Published point (2, 3) must be within the grid bounds";
}

// Two distinct XY positions must each appear as independent cells.
TEST_F( Map2DFixture, Generate2DMapTwoPointsProduceTwoCells )
{
  configure_with_2d_map();
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  auto result = publish_cloud_and_wait_for_map( {
    make_pt( 1.0, 0.0, 0.5 ),
    make_pt( 3.0, 0.0, 0.5 ),
  } );
  ASSERT_TRUE( result.has_value() );

  EXPECT_EQ( cell_at( *result, 1.0, 0.0 ), static_cast<int8_t>( 100 ) );
  EXPECT_EQ( cell_at( *result, 3.0, 0.0 ), static_cast<int8_t>( 100 ) );
}

// Cells that the OctoMap has marked free (from ray-trace but no obstacle)
// must appear as 0, not 100 or -1.
TEST_F( Map2DFixture, Generate2DMapFreeCellIsZero )
{
  configure_with_2d_map( 0.0, 1.5 );
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  // Place an obstacle far away; the origin-to-obstacle ray passes through (0.5, 0, 0.5)
  // which will be marked free by the ray-casting step.
  auto result = publish_cloud_and_wait_for_map( { make_pt( 2.0, 0.0, 0.5 ) } );
  ASSERT_TRUE( result.has_value() );

  // (0.5, 0.0) lies on the ray from origin to (2.0, 0.0) → should be free
  const int8_t free_cell = cell_at( *result, 0.5, 0.0 );
  EXPECT_EQ( free_cell, static_cast<int8_t>( 0 ) )
      << "Cells along the ray before the obstacle must be 0 (free)";
}

// With a dedicated publish timer (map_2d_publish_frequency > 0) the map must
// arrive even if we do NOT publish a new point cloud after start().
TEST_F( Map2DFixture, Generate2DMapTimerPublishesWithoutNewCloud )
{
  configure_with_2d_map( 0.0, 1.5, "map_2d", /*freq=*/10.0 );
  ASSERT_TRUE( updater_->setParams( kNs ) );

  // First: insert a voxel via a cloud so the tree is not empty
  updater_->start();
  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      kTopic, rclcpp::SensorDataQoS() );
  auto map_sub =
      tester_node_->create_test_subscription<nav_msgs::msg::OccupancyGrid>( "map_2d" );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  pub->publish( make_point_cloud( "map", tester_node_->now(), { make_pt( 1.0, 0.0, 0.5 ) } ) );
  // Wait for the tree to be populated
  ASSERT_TRUE( executor_->spin_until(
      [this]() { return is_occupied( octomap::point3d( 1.0f, 0.0f, 0.5f ) ); }, 5s ) );

  // Reset the subscription so we only see messages produced by the timer
  map_sub->reset();

  // The timer fires at 10 Hz so within 500 ms we expect at least one message
  EXPECT_TRUE( map_sub->wait_for_message( *executor_, 500ms ) )
      << "2D map timer should publish autonomously at the configured frequency";
}

// When generate_2d_map is false the 2D topic must remain silent even after
// a cloud arrives that would produce a map if the feature were enabled.
TEST_F( Map2DFixture, Generate2DMapSilentWhenDisabled )
{
  configure_params( kNs, kTopic, false );
  declare_or_set_param<bool>( std::string( kNs ) + ".generate_2d_map", false );
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      kTopic, rclcpp::SensorDataQoS() );
  auto map_sub =
      tester_node_->create_test_subscription<nav_msgs::msg::OccupancyGrid>( "map_2d" );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  pub->publish( make_point_cloud( "map", tester_node_->now(), { make_pt( 1.0, 0.0, 0.5 ) } ) );
  spin_for( 300ms );

  EXPECT_FALSE( map_sub->has_new_message( 0 ) )
      << "No OccupancyGrid should be published when generate_2d_map=false";
}

// The map frame_id must match the monitor's map frame ("map").
TEST_F( Map2DFixture, Generate2DMapFrameIdMatchesMonitor )
{
  configure_with_2d_map();
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  auto result = publish_cloud_and_wait_for_map( { make_pt( 1.0, 0.0, 0.5 ) } );
  ASSERT_TRUE( result.has_value() );
  EXPECT_EQ( result->header.frame_id, "map" );
}

// A custom map_2d_topic must be respected.
TEST_F( Map2DFixture, Generate2DMapCustomTopicName )
{
  configure_with_2d_map( 0.0, 1.5, "robot_floor_map" );
  ASSERT_TRUE( updater_->setParams( kNs ) );
  updater_->start();

  // Subscribe to the custom topic name
  auto result = publish_cloud_and_wait_for_map( { make_pt( 1.0, 0.0, 0.5 ) }, "robot_floor_map" );
  EXPECT_TRUE( result.has_value() )
      << "OccupancyGrid must be published on the custom topic 'robot_floor_map'";
}

// =============================================================================

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  auto result = RUN_ALL_TESTS();
  // Use _exit to avoid rmw_zenoh shutdown crash (context destroyed before service handles)
  _exit( result );
}