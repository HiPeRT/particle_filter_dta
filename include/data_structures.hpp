#pragma once

#define N_RAYS_DS (60)

struct Particle_t
{
    int id;
    float x, y;
    float yaw;
    double weight;
    float rays[N_RAYS_DS];
};

struct Map_t
{
    int MAX_RANGE_PX;
    float opp_originX;
    float opp_originY;
    float map_resolution;
    float map_originX;
    float map_originY;
    int map_height;
    int map_width;
};

struct Cloud_t
{
    int maxRayIteration;
    float maxRange;
    float angleMin;
    float angleMax;
    float angleIncrement;
    float angleDownsample;
    int nAngle;
};