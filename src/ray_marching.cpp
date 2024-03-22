#include "ray_marching.hpp"
#include "xrm.hpp"
#include <cstring>
#include <ctime>
#include <omp.h>

/**
 * Initializes all the parameters that are necessary for ray_marching class
 */
void RayMarching::init(bool FPGA, bool GPU, int n_threads)
{
    if (FPGA && !GPU) {

        while(uio_initialize(&rm, uio_devices::RM)) {
            std::cout << "Init Error RM" << std::endl;
            uio_release(&rm);
        }

        while (uio_initialize(&x_buf, uio_devices::X_BUF)) {
            std::cout << "Init Error X" << std::endl;
            uio_release(&x_buf);
        }

        while (uio_initialize(&y_buf, uio_devices::Y_BUF)) {
            std::cout << "Init Error Y" << std::endl;
            uio_release(&y_buf);
        }

        while (uio_initialize(&yaw_buf, uio_devices::YAW_BUF)) {
            std::cout << "Init Error YAW" << std::endl;
            uio_release(&yaw_buf);
        }

        while (uio_initialize(&rays_buf, uio_devices::RAYS_BUF)) {
            std::cout << "Init Error RAYS" << std::endl;
            uio_release(&rays_buf);
        }

        while (uio_initialize(&map_buf, uio_devices::MAP_BUF)) {
            std::cout << "Init Error MAP" << std::endl;
            uio_release(&map_buf);
        }

        while (uio_initialize(&angles_buf, uio_devices::ANGLES_BUF)) {
            std::cout << "Init Error ANGLES" << std::endl;
            uio_release(&angles_buf);
        }


        loadMapFPGA = false;
    } else if (GPU && !FPGA) {
        first = true;
    }
    num_threads = n_threads;
}

void RayMarching::calculateRays(Particle_t* particles,
                                float* distMap,
                                Map_t *map,
                                Cloud_t *cloud,
                                int n_particles,
                                float* rays_angle)
{
    #pragma omp parallel for num_threads(num_threads)
    for (int i = 0; i < n_particles; i++) {
        for (int j = 0; j < N_RAYS_DS; ++j) {
            float angle = (particles[i].yaw + cloud->angleMin) + rays_angle[j];
            double rayPoseX = particles[i].x;
            double rayPoseY = particles[i].y;
            float t = 0.0f;
            float out = cloud->maxRange;
            while (t < cloud->maxRayIteration) {
                int c = (int)((map->opp_originX - rayPoseX) / map->map_resolution);
                int r = (int)((map->opp_originY + rayPoseY) / map->map_resolution);

                if (c < 0 || c >= map->map_width || r < 0 || r > map->map_height) {
                    out = cloud->maxRange;
                    break;
                }

                double distance = distMap[r * map->map_width + c];
                rayPoseX += distance * std::cos(angle);
                rayPoseY += distance * std::sin(angle);

                if (distance <= map->map_resolution) {
                    float xd = rayPoseX - particles[i].x;
                    float yd = rayPoseY - particles[i].y;
                    out = sqrtf(xd * xd + yd * yd);
                    break;
                }

                t += fmaxf(distance * 0.999f, 1.0);
            }

            particles[i].rays[j] = out;
        }
    }
}

void RayMarching::calculateRaysFPGA(Particle_t* particles,
                                    float* distMap,
                                    Map_t *map,
                                    Cloud_t *cloud,
                                    int n_particles,
                                    float* rays_angle)
{

    orig_x = map->opp_originX;
    orig_y = map->opp_originY;
    map_resolution = map->map_resolution;
    map_height = map->map_height;
    map_width = map->map_width;
    particles_d = n_particles;

    if (!loadMapFPGA) {
        for (int i = 0; i < map_height * map_width; i++) {
            distanceMap_d[i] = distMap[i];
        }
    }

    for (int i = 0; i < n_particles; ++i) {
        ((float*)x_buf.addr)[i] = particles[i].x;
    }

    for (int i = 0; i < n_particles; ++i) {
        ((float*)y_buf.addr)[i] = particles[i].y;
    }

    for(int i=0; i<N_RAYS_DS; ++i) {
        ((float*)angles_buf.addr)[i] = rays_angle[i];
    }

    for (int i = 0; i < n_particles; ++i) {
        ((float*)yaw_buf.addr)[i] = particles[i].yaw;
    }

    if (!loadMapFPGA) {
        for (int i = 0; i < WIDTH * HEIGHT; ++i) {
            ((float*)map_buf.addr)[i] = distanceMap_d[i];
        }
    }

    if (!loadMapFPGA) {
        XRmk_Set_x(&rm, (uint32_t)x_buf.phy);
        XRmk_Set_y(&rm, (uint32_t)y_buf.phy);
        XRmk_Set_yaw(&rm, (uint32_t)yaw_buf.phy);
        XRmk_Set_rays(&rm, (uint32_t)rays_buf.phy);
        XRmk_Set_rays_angle(&rm, (uint32_t)angles_buf.phy);
        XRmk_Set_distMap(&rm, (uint32_t)map_buf.phy);
        XRmk_Set_orig_x(&rm, (*(uint32_t*)&orig_x));
        XRmk_Set_orig_y(&rm, (*(uint32_t*)&orig_y));
        XRmk_Set_map_resolution(&rm, (*(uint32_t*)&map_resolution));
        XRmk_Set_map_height(&rm, (uint32_t)map_height);
        XRmk_Set_map_width(&rm, (uint32_t)map_width);
        XRmk_Set_n_particles(&rm, (uint32_t)particles_d);
        XRmk_Set_rm_mode(&rm, 0);

        XRmk_Start(&rm);
        while (!XRmk_IsDone(&rm))
            ;

        loadMapFPGA = true;
        XRmk_Set_rm_mode(&rm, 1);
    }

    XRmk_Set_n_particles(&rm, (uint32_t)particles_d);

    XRmk_Start(&rm);
    while (!XRmk_IsDone(&rm))
        ;

    for (int i = 0; i < n_particles; ++i) {
        for (int j = 0; j < N_RAYS_DS; ++j) {
            particles[i].rays[j] = ((float*)rays_buf.addr)[i * N_RAYS_DS + j];
        }
    }
}
#ifdef TKCUDA_ENABLED
void RayMarching::calculateRaysGPU(Particle_t* particles,
                                   float* distMap,
                                   Map_t* map,
                                   Cloud_t* cloud,
                                   int n_particles,
                                   float* rays_angle)
{
    if (first) {
        cudaRm.init(particles, distMap, cloud, map, n_particles, rays_angle);
        first = false;
    }
    cudaRm.calculateRays(particles, n_particles, rays_ds);
    for (int i = 0; i < n_particles; i++) {
        for (int j = 0; j < N_RAYS_DS; ++j) {
            particles[i].rays[j] = rays_ds[i * N_RAYS_DS + j];
        }
    }
}
#endif

void RayMarching::calculateWeights(Particle_t* particles,
                                   float* obs,
                                   float* sensor_model_table,
                                   Map_t* map,
                                   Cloud_t* cloud,
                                   int table_width,
                                   int n_particles)
{

    for (int i = 0; i < n_particles; i++) {
        double weight_temp = 1.0f;
        for (int j = 0; j < N_RAYS_DS; j++) { // TODO: substitute N_RAYS_DS with real number of rays

            float realRayPX = obs[j] / map->map_resolution;
            realRayPX = std::min<float>(std::max<float>(realRayPX, 0.0), (map->MAX_RANGE_PX - 1.0));

            float virtualRayPX = particles[i].rays[j] / map->map_resolution;

            virtualRayPX =
              std::min<float>(std::max<float>(virtualRayPX, 0.0), (map->MAX_RANGE_PX - 1.0));

            weight_temp *=
              (double)sensor_model_table[int(virtualRayPX) * table_width + int(realRayPX)];
            // weight_temp *=
            // (double)sensor_model_table[int(realRayPX)*table_width+int(virtualRayPX)];
        }

        particles[i].weight = weight_temp;
    }
}
