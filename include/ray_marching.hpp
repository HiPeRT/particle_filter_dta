#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <yaml-cpp/yaml.h>
#ifdef TKCUDA_ENABLED
#include "cudaRayMarching.h"
#endif
#include "data_structures.hpp"
#include "uio_map.hpp"

#define WIDTH 485
#define HEIGHT 379
#define N_RAYS 60
#define N_PARTICLES 2000

enum uio_devices : unsigned int
{
    RM = 4,
    X_BUF = 5,
    Y_BUF = 6,
    YAW_BUF = 7,
    RAYS_BUF = 8,
    MAP_BUF = 9,
    ANGLES_BUF = 10
};

class RayMarching
{

  private:
    std::vector<double> weights;
    int count = 0;
    float x[N_PARTICLES];
    float y[N_PARTICLES];
    float yaw[N_PARTICLES];
    float rays[N_PARTICLES * N_RAYS];
    float rays_total[N_PARTICLES * N_RAYS];
    float rays_ds[N_PARTICLES * N_RAYS_DS];
    float distanceMap_d[WIDTH * HEIGHT];
    float orig_x;
    float orig_y;
    float map_resolution;
    int map_height;
    int map_width;
    float angleMin;
    float angleIncrement;
    float maxRange;
    int particles_d;
    int fd;
    bool loadMapFPGA;

    // FPGA
    UioMap rm;
    UioMap x_buf;
    UioMap y_buf;
    UioMap yaw_buf;
    UioMap rays_buf;
    UioMap map_buf;
    UioMap angles_buf;

    // GPU
#ifdef TKCUDA_ENABLED
    cRayMarching cudaRm;
#endif
    bool first;

  public:
    int num_threads;
    void init(bool FPGA, bool GPU, int n_threads);
    void compute_weights(Particle_t* particles,
                         float* obs,
                         float* distMap,
                         float* sensor_model_table,
                         Map_t* map,
                         Cloud_t* cloud,
                         int table_width,
                         int n_particles);
    void calculateRays(Particle_t* particles,
                       float* distMap,
                       Map_t* map,
                       Cloud_t* cloud,
                       int n_particles,
                          float* rays_angle);
    void calculateRaysFPGA(Particle_t* particles,
                           float* distMap,
                           Map_t* map,
                           Cloud_t* cloud,
                           int n_particles,
                          float* rays_angle);
#ifdef TKCUDA_ENABLED
    void calculateRaysGPU(Particle_t* particles,
                          float* distMap,
                          Map_t* map,
                          Cloud_t* cloud,
                          int n_particles,
                          float* rays_angle);
#endif
    void calculateWeights(Particle_t* particles,
                          float* obs,
                          float* sensor_model_table,
                          Map_t* map,
                          Cloud_t* cloud,
                          int table_width,
                          int n_particles);

    void calculateWeightsOMP(Particle_t* particles,
                             float* obs,
                             float* sensor_model_table,
                             Map_t map,
                             Cloud_t cloud,
                             int table_width,
                             int n_particles);
};
