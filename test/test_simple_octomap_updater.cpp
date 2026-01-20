#include <gtest/gtest.h>

#include <hector_testing_utils/hector_testing_utils.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <hector_worldmodel_msgs/srv/get_distance_to_obstacle.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <simple_pointcloud_octomap_updater/simple_pointcloud_octomap_updater.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace
{
sensor_msgs::msg::PointCloud2 make_point_cloud( const std::string &frame_id,
                                                const rclcpp::Time &stamp,
                                                const std::vector<geometry_msgs::msg::Point> &points )
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<uint32_t>( points.size() );

  sensor_msgs::PointCloud2Modifier modifier( msg );
  modifier.setPointCloud2FieldsByString( 1, "xyz" );
  modifier.resize( points.size() );

  sensor_msgs::PointCloud2Iterator<float> iter_x( msg, "x" );
  sensor_msgs::PointCloud2Iterator<float> iter_y( msg, "y" );
  sensor_msgs::PointCloud2Iterator<float> iter_z( msg, "z" );
  for ( const auto &pt : points ) {
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return msg;
}

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
    if ( updater_ ) {
      updater_->stop();
    }
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

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<occupancy_map_monitor::OccupancyMapMonitor> monitor_;
  std::shared_ptr<occupancy_map_monitor::SimplePointCloudOctomapUpdater> updater_;
};
} // namespace

TEST_F( SimpleOctomapUpdaterFixture, SetParamsSuccedsWithAllRequiredParams )
{
  const std::string name_space = "simple_octomap";
  declare_or_set_param( name_space + ".point_cloud_topic", "/test_cloud" );
  declare_or_set_param( name_space + ".max_update_rate", 0.0 );
  declare_or_set_param( name_space + ".min_range", 0.0 );
  declare_or_set_param( name_space + ".max_range", 5.0 );
  declare_or_set_param( name_space + ".point_subsample", 1 );
  EXPECT_TRUE( updater_->setParams( name_space ) );
}

TEST_F( SimpleOctomapUpdaterFixture, SetParamsFailsWithMissingRequiredParams )
{
  const std::string name_space = "simple_octomap";
  declare_or_set_param( name_space + ".max_update_rate", 0.0 );
  declare_or_set_param( name_space + ".min_range", 0.0 );
  declare_or_set_param( name_space + ".max_range", 5.0 );
  declare_or_set_param( name_space + ".point_subsample", 1 );
  EXPECT_FALSE( updater_->setParams( name_space ) );
}

TEST_F( SimpleOctomapUpdaterFixture, PointCloudMarksOccupiedCell )
{
  const std::string name_space = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( name_space, topic, false );
  ASSERT_TRUE( updater_->setParams( name_space ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  geometry_msgs::msg::Point point;
  point.x = 1.0;
  point.y = 0.0;
  point.z = 0.0;

  pub->publish( make_point_cloud( "map", tester_node_->now(), { point } ) );

  EXPECT_TRUE( executor_->spin_until(
      [this]() { return is_occupied( octomap::point3d( 1.0F, 0.0F, 0.0F ) ); }, 5s ) );
}

TEST_F( SimpleOctomapUpdaterFixture, MinRangeSkipsPoints )
{
  const std::string name_space = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( name_space, topic, false, 1, 0.5, 5.0, 0.0 );
  ASSERT_TRUE( updater_->setParams( name_space ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  geometry_msgs::msg::Point point;
  point.x = 0.1;
  point.y = 0.0;
  point.z = 0.0;

  pub->publish( make_point_cloud( "map", tester_node_->now(), { point } ) );
  spin_for( 300ms );

  EXPECT_FALSE( has_node( octomap::point3d( 0.1F, 0.0F, 0.0F ) ) );
}

TEST_F( SimpleOctomapUpdaterFixture, PointSubsampleSkipsEveryOtherPoint )
{
  const std::string name_space = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( name_space, topic, false, 2, 0.0, 5.0, 0.0 );
  ASSERT_TRUE( updater_->setParams( name_space ) );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  ASSERT_TRUE( pub->wait_for_subscription( *executor_, 5s ) );

  geometry_msgs::msg::Point first_point;
  first_point.x = 1.0;
  first_point.y = 0.0;
  first_point.z = 0.0;

  geometry_msgs::msg::Point second_point;
  second_point.x = 2.0;
  second_point.y = 0.0;
  second_point.z = 0.0;

  pub->publish( make_point_cloud( "map", tester_node_->now(), { first_point, second_point } ) );

  EXPECT_TRUE( executor_->spin_until(
      [this]() { return is_occupied( octomap::point3d( 1.0F, 0.0F, 0.0F ) ); }, 5s ) );
  EXPECT_FALSE( has_node( octomap::point3d( 2.0F, 0.0F, 0.0F ) ) );
}

TEST_F( SimpleOctomapUpdaterFixture, DistanceServiceReturnsNoHitAndMarker )
{
  using Service = hector_worldmodel_msgs::srv::GetDistanceToObstacle;

  const std::string name_space = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( name_space, topic, true );
  ASSERT_TRUE( updater_->setParams( name_space ) );

  auto client = tester_node_->create_test_client<Service>( "get_distance_to_obstacle" );
  auto marker_sub = tester_node_->create_test_subscription<visualization_msgs::msg::Marker>(
      "distance_ray_marker" );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );

  auto request = std::make_shared<Service::Request>();
  request->point.header.frame_id = "map";
  request->point.header.stamp = tester_node_->now();
  request->point.point.x = 1.0;
  request->point.point.y = 0.0;
  request->point.point.z = 0.0;

  marker_sub->reset();

  auto response = hector_testing_utils::call_service<Service>( client->get(), request, *executor_ );
  ASSERT_NE( response, nullptr );
  EXPECT_LT( response->distance, 0.0 );
  EXPECT_EQ( response->end_point.header.frame_id, "map" );
  // msg either arrives during service or spin after
  ASSERT_TRUE( marker_sub->has_new_message( 0 ) || marker_sub->wait_for_message( *executor_, 5s ) );
  auto marker = marker_sub->last_message();
  ASSERT_TRUE( marker.has_value() );
  EXPECT_EQ( marker->ns, "get_distance_to_obstacle" );
  ASSERT_EQ( marker->points.size(), 2u );
}

TEST_F( SimpleOctomapUpdaterFixture, DistanceServiceHitsOccupiedCell )
{
  using Service = hector_worldmodel_msgs::srv::GetDistanceToObstacle;

  const std::string name_space = "simple_octomap";
  const std::string topic = "/test_cloud";
  configure_params( name_space, topic, true );
  ASSERT_TRUE( updater_->setParams( name_space ) );
  updater_->initialize( tester_node_ );
  updater_->start();

  auto pub = tester_node_->create_test_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS() );
  auto client = tester_node_->create_test_client<Service>( "get_distance_to_obstacle" );
  auto marker_sub = tester_node_->create_test_subscription<visualization_msgs::msg::Marker>(
      "distance_ray_marker", rclcpp::QoS( 10 ) );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 5s ) );
  ASSERT_TRUE( marker_sub->is_connected() );
  geometry_msgs::msg::Point point;
  point.x = 1.0;
  point.y = 0.0;
  point.z = 0.0;

  pub->publish( make_point_cloud( "map", tester_node_->now(), { point } ) );
  ASSERT_TRUE( executor_->spin_until(
      [this]() { return is_occupied( octomap::point3d( 1.0F, 0.0F, 0.0F ) ); }, 5s ) );

  auto request = std::make_shared<Service::Request>();
  request->point.header.frame_id = "map";
  request->point.header.stamp = tester_node_->now();
  request->point.point.x = 1.0;
  request->point.point.y = 0.0;
  request->point.point.z = 0.0;

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
