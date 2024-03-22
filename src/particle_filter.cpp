#include "particle_filter.hpp"
#include "utils.hpp"
#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

static constexpr float FLOAT_PI = static_cast<float>(M_PI);

/**
 * Initializes all the parameters that are necessary for the next
 * functions
 */
void ParticleFilter::init(std::string conf_path)
{
    firstMM = true;

    YAML::Node config = YAML::LoadFile(conf_path);
    if (config.IsNull()) {
        std::cout << "error" << std::endl;
        return;
    }

    useFPGA = config["useFPGA"].as<bool>();
    useGPU = config["useGPU"].as<bool>();
    MAX_PARTICLES = config["MAX_PARTICLES"].as<int>();
    MAX_RANGE_METERS = config["MAX_RANGE_METERS"].as<int>();
    std_w_mul = config["std_w_mul"].as<float>();
    std_yaw = config["std_yaw"].as<float>();
    std_x_mul = config["std_x_mul"].as<float>();
    std_y_mul = config["std_y_mul"].as<float>();
    std_v_mul = config["std_v_mul"].as<float>();
    z_short = config["z_short"].as<float>();
    z_max = config["z_max"].as<float>();
    z_rand = config["z_rand"].as<float>();
    z_hit = config["z_hit"].as<float>();
    sigma_hit = config["sigma_hit"].as<float>();

    cloud.maxRayIteration = config["maxRayIteration"].as<int>();
    cloud.maxRange = config["maxRange"].as<float>();
    cloud.angleMin = config["angleMin"].as<float>();
    cloud.angleMax = config["angleMax"].as<float>();
    cloud.angleIncrement = config["angleIncrement"].as<float>();
    cloud.angleDownsample = config["angleDownsample"].as<float>();
    cloud.nAngle =
      int((std::fabs(cloud.angleMax) + std::fabs(cloud.angleMin)) / cloud.angleIncrement);

    n_threads = config["num_threads"].as<int>();

    rayMarching.init(useFPGA, useGPU, n_threads);
}

/**
 *  Fetch the occupancy grid map from the map_server instance, and
 *  stores a matrix which indicates the permissible region of the map
 **/
void ParticleFilter::getOmap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{

    map.map_height = map_msg->info.height;
    map.map_width = map_msg->info.width;
    map.map_originX = map_msg->info.origin.position.x;
    map.map_originY = map_msg->info.origin.position.y;
    map.map_resolution = map_msg->info.resolution;
    map.MAX_RANGE_PX = MAX_RANGE_METERS / map.map_resolution;
    map.opp_originX = (map.map_width * map.map_resolution) + map.map_originX;
    map.opp_originY = -map.map_originY;

    table_width = map.MAX_RANGE_PX + 1;

    array_255 = Eigen::MatrixXi(map.map_height, map.map_width);
    permissible_region = Eigen::MatrixXi(map.map_height, map.map_width);
    for (int i = 0; i < map.map_width * map.map_height; i++) {
        int r = i / map.map_width;
        int c = i % map.map_width;
        array_255(r, c) = map_msg->data[i];
    }

    cv::Mat img_test = cv::Mat::ones(cv::Size(map.map_width, map.map_height), CV_8UC1) * 0;
    for (int i = 0; i < map.map_height; i++) {
        for (int j = 0; j < map.map_width; j++) {
            if (array_255(i, j) == 0) {
                img_test.at<uchar>(i, j) = 255;
            } else {
                img_test.at<uchar>(i, j) = 0;
            }
        }
    }

    cvtColor(img_test, img_test, cv::COLOR_GRAY2RGB);
    img_test.copyTo(srcMap);
    if (srcMap.empty()) {
        std::cout << "Could not open or find the image!" << std::endl;
        return;
    }
    cv::flip(srcMap, srcMap, -1);
    cv::flip(srcMap, srcMap, 0);
    int filling =
      cv::floodFill(srcMap, cv::Point(0, 0), cv::Scalar(0, 0, 0), (cv::Rect*)0, cv::Scalar(), 200);
    // Create a kernel that we will use to sharpen our image
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);

    cv::Mat imgLaplacian;
    // filter2D(srcMap, imgLaplacian, CV_32F, kernel);
    cv::Mat sharp = srcMap;
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    srcMap.convertTo(sharp, CV_32F);
    cv::Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    // Create bweightinary image from source image
    cv::Mat bw;
    // cvtColor(imgResult, bw, cv::COLOR_BGR2GRAY);
    cvtColor(srcMap, bw, cv::COLOR_BGR2GRAY);
    threshold(bw, bw, 100, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // Perform the distance transform algorithm
    distanceTransform(bw, distMap, cv::DIST_L2, 3);
    cv::normalize(distMap, distMap, 0, 1, cv::NORM_MINMAX);

    cv::Point p;
    float distanceMapTmp[map.map_width * map.map_height];
    for (int i = 0; i < distMap.rows; i++) {
        for (int j = 0; j < distMap.cols; j++) {
            p.x = j;
            p.y = i;
            distanceMapTmp[i * distMap.cols + j] = distMap.at<float>(p);
        }
    }

    distanceMap = distanceMapTmp;
}

/**
 * Initialize particles in the general region of the provided pose.
 **/
void ParticleFilter::initialize_particles_pose(std::vector<float> p)
{

    weights.resize(MAX_PARTICLES);
    particles.resize(MAX_PARTICLES);
    std::default_random_engine generator3;
    std::normal_distribution<float> dist_x(0.0f, 0.02f);
    std::normal_distribution<float> dist_y(0.0f, 0.02f);
    std::normal_distribution<float> dist_yaw(0.0f, 0.02f);

    std::cout << "x = " << p[0] << " y= " << p[1] << "yaw = " << p[2] << std::endl;
    firstMM = true;

    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].x = p[0] + dist_x(generator3);
        particles[i].y = p[1] + dist_y(generator3);
        particles[i].yaw = p[2];
        particles[i].weight = 1.0f;
        weights[i] = particles[i].weight;
        for (int j = 0; j < N_RAYS_DS; j++) {
            particles[i].rays[j] = 0;
        }
    }
}

/**
 * Generate and store a table which represents the sensor model. For each discrete computed
 * range value, this provides the probability of measuring any (discrete) range.
 **/
void ParticleFilter::precompute_sensor_model()
{

    sensor_model_table.resize(table_width * table_width);

    for (int d = 0; d < table_width; d++) {
        float norm = 0.0f;
        for (int r = 0; r < table_width; r++) {
            float prob = 0.0f;
            float z = float(r - d);
            prob += z_hit * std::exp(-(z * z) / (2.0f * sigma_hit * sigma_hit)) /
                    (sigma_hit * sqrtf(2.0f * M_PI));
            if (r < d) {
                prob += 2.0f * z_short * (d - r) / float(d);
            }
            if (int(r) == int(map.MAX_RANGE_PX)) {
                prob += z_max;
            }
            if (r < int(map.MAX_RANGE_PX)) {
                prob += z_rand * 1.0f / float(map.MAX_RANGE_PX);
            }
            norm += prob;
            sensor_model_table[r * table_width + d] = prob;
        }
        for (int j = 0; j < table_width; j++) {
            sensor_model_table[j * table_width + d] /= norm;
        }
    }
}

/**
 * Resample the distribution of the particles at each istant
 *  with a unfiorm distribution
 */
std::vector<Particle_t> ParticleFilter::resample(unsigned int n_particles)
{
    std::vector<Particle_t> new_particles;
    float U, r;
    float c = particles[0].weight;
    int i = 1;

    std::uniform_real_distribution<double> distribution(0, 1.0f / (float)n_particles);
    r = distribution(generator);
    for (int m = 0; m < n_particles; m++) {
        U = r + float(m - 1) * 1.0f / (float)n_particles;
        while (U > c) {
            i = (i + 1) % n_particles;
            c += particles[i].weight;
        }
        new_particles.push_back(particles[i]);
    }
    return new_particles;
}

/**
 * The motion model applies the odometry to the particle distribution. Since there the odometry
 * data is inaccurate, the motion model mixes in gaussian noise to spread out the distribution.
 * Vectorized motion model. Computing the motion model over all particles is thousands of times
 * faster than doing it for each particle individually due to vectorization and reduction in
 * function call overhead
 **/

void ParticleFilter::motion_model(float velocity, float dts, float yaw_rate)
{
    std::vector<Particle_t> valid_particles;
    Particle_t tmp_particle;
    float x, y, yaw;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        // Add measurements to each particle
        if (std::fabs(yaw_rate) < 0.001f) { // car going straight
            x = particles[i].x + velocity * dts * std::cos(particles[i].yaw);
            y = particles[i].y + velocity * dts * std::sin(particles[i].yaw);
            yaw = particles[i].yaw;
        } else { // yaw rate is not equal to zero, car turning
            x = particles[i].x +
                (velocity) / yaw_rate *
                  (std::sin(particles[i].yaw + yaw_rate * dts) - std::sin(particles[i].yaw));
            y = particles[i].y +
                (velocity) / yaw_rate *
                  (std::cos(particles[i].yaw) - std::cos(particles[i].yaw + yaw_rate * dts));
            yaw = particles[i].yaw + yaw_rate * dts;

            // remap to [-M_PI , M_PI]
            yaw -= static_cast<int>((yaw + FLOAT_PI) / (2.0f * FLOAT_PI)) * (2.0f * FLOAT_PI);
        }

        float std_x = std_x_mul + std::fabs(velocity * dts * std_v_mul);
        float std_y = std_y_mul;
        // RANDOM GAUSSIAN NOISE
        float x_noise = 0.0f;
        float y_noise = 0.0f;
        float yaw_noise = 0.0f;

        std::normal_distribution<double> dist_x(x_noise, std_x);
        std::normal_distribution<double> dist_y(y_noise, std_y);
        std::normal_distribution<double> dist_yaw(yaw_noise,
                                                  std_yaw + std::fabs(yaw_rate * dts * std_w_mul));
        x_noise = dist_x(generator);
        y_noise = dist_y(generator);
        double tsin = sin(yaw);
        double tcos = cos(yaw);
        double rot_x = x_noise * tcos - y_noise * tsin;
        double rot_y = x_noise * tsin + y_noise * tcos;

        tmp_particle.x = x + rot_x;
        tmp_particle.y = y + rot_y;

        int c = (int)((map.opp_originX - tmp_particle.x) / map.map_resolution);
        int r = (int)((map.opp_originY + tmp_particle.y) / map.map_resolution);
        // float distance = distanceMap[r * map.map_width + c];

        if (distanceMap[r * map.map_width + c] >= map.map_resolution) {
            tmp_particle.yaw = yaw + dist_yaw(generator);

            valid_particles.push_back(tmp_particle);
        }
    }
    n_valid_particles_ = valid_particles.size();
    particles = valid_particles;

    if ((!firstMM) && n_valid_particles_ < MAX_PARTICLES) {
    //    std::vector<Particle_t> new_particles = resample(MAX_PARTICLES - n_valid_particles_);
    //    particles.insert(particles.end(), new_particles.begin(), new_particles.end());
    }
    firstMM = false;
}

/**
 * This functiion updates the weights of each particle
 * calculated by the compute_weights method of the ray_marching class
 **/
void ParticleFilter::sensor_model(std::vector<float> obs, std::vector<float> rays_angle)
{
    weights.resize(MAX_PARTICLES);
    Particle_t* particles_d = &particles[0];
    // float* obs_d = &obs[0];
    // float* sensor_model_table_d = &sensor_model_table[0];

    if (useFPGA) {
        rayMarching.calculateRaysFPGA(
          particles_d, distanceMap, &map, &cloud, n_valid_particles_, &rays_angle[0]);
    } else if (useGPU) {
#ifdef TKCUDA_ENABLED
        rayMarching.calculateRaysGPU(
          particles_d, distanceMap, &map, &cloud, n_valid_particles_, &rays_angle[0]);
#endif
    } else {
        rayMarching.num_threads = n_threads;
        rayMarching.calculateRays(
          particles_d, distanceMap, &map, &cloud, n_valid_particles_, &rays_angle[0]);
    }

    rayMarching.calculateWeights(
      particles_d, &obs[0], &sensor_model_table[0], &map, &cloud, table_width, n_valid_particles_);
    // Copy the different weights of particles_d in particles and weights
    for (int i = 0; i < n_valid_particles_; i++) {
        particles[i].weight = particles_d[i].weight;
        weights[i] = particles_d[i].weight;
    }
}

/**
 * Normalize to make weights sum equal to 1
 **/
void ParticleFilter::normalize()
{
    double weightSum = 0;
    for (int i = 0; i < n_valid_particles_; i++) {
        weightSum += particles[i].weight;
    }

    for (int i = 0; i < n_valid_particles_; i++) {
        weights[i] /= weightSum;
        particles[i].weight /= weightSum;
    }
}

/**
 * Returns the most probable position of the car at each instant
 */
std::vector<float> ParticleFilter::expected_pose()
{

    // std::vector<float> expected_pose;
    current_pose.resize(3);
    Eigen::MatrixXf pose = Eigen::MatrixXf(n_valid_particles_, 3);
    Eigen::VectorXf w = Eigen::VectorXf(n_valid_particles_);
    for (int i = 0; i < n_valid_particles_; i++) {
        pose(i, 0) = particles[i].x;
        pose(i, 1) = particles[i].y;
        pose(i, 2) = particles[i].yaw;
        w[i] = particles[i].weight;
    }
    Eigen::VectorXf expected = pose.transpose() * w;
    current_pose[0] = expected(0);
    current_pose[1] = expected(1);
    current_pose[2] = expected(2);

    return current_pose;
}

/**
 * @brief Variance estimate
 *
 * Estimate variance using the particle weights
 *
 * @return the variance vector
 */
std::vector<float> ParticleFilter::expected_variance(std::vector<float> expected_pose)
{
    std::vector<float> variance = { 0, 0, 0 };
    for (int i = 0; i < MAX_PARTICLES; i++) {
        variance[0] += particles[i].weight * std::pow(particles[i].x - expected_pose[0], 2);
        variance[1] += particles[i].weight * std::pow(particles[i].y - expected_pose[1], 2);
        variance[2] += particles[i].weight * std::pow(particles[i].yaw - expected_pose[2], 2);
    }
    return variance;
}
