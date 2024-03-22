#pragma once

#include "particle_filter.hpp"
#include <fstream>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <time.h>
#include <sstream>

#include <dta/dta.hpp>
#include <sector_handler/sector_reader.hpp>
#include <adx_data/sector.hpp>
#include <sector_handler/sector_tracker.hpp>

class ParticleFilterNode
{
  private:
    ParticleFilter particleFilter;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr click_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trj_pub;

    std::ofstream* file;

    // flag for sync
    bool map_initialized = false;
    bool take_pose = false;
    bool odom_initialized = false;
    // bool lidar_initialized = false;
    bool first_MCL = true;
    bool is_update = false;
    bool first_visualize = true;
    bool first_odom = true;

    SectorTracker mSectorTracker;

    // clicked_poseCb
    std::vector<float> initial_pose;

    // odomCb
    float odom_x;
    float odom_y;
    float odom_v;
    float oldDts = 0.0f;
    float oldYaw = 0.0f;
    float oldX = 0.0f;
    float oldY = 0.0f;
    float dts = 0.0f;
    float w = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float velocity;
    double last_stamp;

    // lidarCb
    float angle_min;
    float angle_max;
    std::vector<float> scan;
    std::vector<float> rays_angles;

    // visualize
    nav_msgs::msg::Odometry position;
    geometry_msgs::msg::PoseArray particles;
    nav_msgs::msg::Path path;
    std::vector<Particle_t> particle_pose;
    std::vector<float> prev_pose;
    std::vector<float> poses;
    std::vector<float> variance;
    std::vector<Eigen::Matrix<float, 10, 4>> opt_latencies;

    std::string conf_path;

    // tf2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    void update(rclcpp::Time timestamp);
    void MCL();
    void publish_tf(rclcpp::Time timestamp);
    void visualize(rclcpp::Time timestamp);

  public:
#if TKROS_VERSION == 1
    ros::Publisher pose_pub;
    ros::Publisher particles_pub;
    ros::Publisher trj_pub;
#endif
    rclcpp::Node::SharedPtr node;

    ParticleFilterNode();
    void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void clicked_poseCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void getOptimalConfig(int sector, int budget, int& nparticles, int& nthreads, int test);
    void lidarCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

tf2::Quaternion createQuaternionFromYaw(float yaw);
geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(float yaw);
