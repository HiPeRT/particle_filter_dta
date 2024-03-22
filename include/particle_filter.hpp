#pragma once
#include "data_structures.hpp"
#include "ray_marching.hpp"
#include <chrono>
#include <ctime>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

class ParticleFilter
{
  private:
    // map
    int MAX_RANGE_METERS;
    bool map_initialized = false;
    cv::Mat srcMap;
    cv::Mat distMap;
    Eigen::MatrixXi array_255;
    Eigen::MatrixXi permissible_region;
    float* distanceMap;
    // init
    std::vector<double> weights;
    std::default_random_engine generator;
    std::vector<float> current_pose;

    // motion model
    bool firstMM;
    float std_w_mul;
    float std_yaw;
    float std_x_mul;
    float std_y_mul;
    float std_v_mul;

    // sensor model table
    float z_max;
    float z_rand;
    float z_hit;
    float z_short;
    float sigma_hit;
    int table_width;
    std::vector<float> sensor_model_table;

    // sensor model
    Cloud_t cloud;
    float max_range;
    int maxRayIteration;
    float maxRange;
    float angleMin;
    float angleMax;
    float angleIncrement;
    float angleDownsample;
    int nAngle;

    // debug
    int count = 0;
    std::ofstream myfile;
    RayMarching rayMarching;
    clock_t tStart, tEnd;
    double elapsed;

    // utils
    int n_valid_particles_;
    bool useFPGA;
    bool useGPU;
    static bool wayToSort(Particle_t i, Particle_t j);
    //std::vector<float> expected_pose;

  public:
    Map_t map;
    int MAX_PARTICLES;
    std::vector<Particle_t> particles;
    std::vector<float> expected_pose();
    std::vector<float> expected_variance(std::vector<float> expected_pose);

    int n_threads;
    void init(std::string conf_path);
    void getOmap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void initialize_particles_pose(std::vector<float> p);
    void precompute_sensor_model();
    std::vector<Particle_t> resample(unsigned int n_particles);
    void motion_model(float velocity, float dts, float yaw_rate);
    void sensor_model(std::vector<float> obs, std::vector<float> rays_angle);

    void normalize();
};
