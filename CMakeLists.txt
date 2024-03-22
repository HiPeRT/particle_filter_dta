cmake_minimum_required(VERSION 3.5.0)
project(particle_filter VERSION 1.0
        DESCRIPTION "particle_filter"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    if(NOT DEFINED CMAKE_CUDA_STANDARD)
        set(CMAKE_CUDA_STANDARD ${CMAKE_CXX_STANDARD})
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    endif()
    enable_language(CUDA)
else()
    message(STATUS "No CUDA support")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(adx_data REQUIRED)
find_package(adx_msgs REQUIRED)
find_package(roscomp REQUIRED)

find_package(fcl 0.7.0 EXACT REQUIRED) # TODO: make this quiet
find_package(sector_handler REQUIRED)  # TODO: make this quiet
find_package(dta REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message(STATUS "OpenMP FOUND.")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
    include
    pf_node/include
    uio_map/include
)

add_library(
    uio_map
    uio_map/src/uio_map.cpp
)

add_library(
    raymarching
    src/ray_marching.cpp
    src/xrm.cpp
)

target_link_libraries(
    raymarching
    uio_map
    OpenMP::OpenMP_CXX
)

if (CMAKE_CUDA_COMPILER)
    add_library(
        gpuRM STATIC
        src/cudaRayMarching.cu
    )
    target_include_directories(gpuRM PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}>
        $<INSTALL_INTERFACE:include>
    )

    target_compile_options(gpuRM PUBLIC -DTKCUDA_ENABLED)
    set_target_properties(gpuRM
        PROPERTIES
            CUDA_SEPARABLE_COMPILATION ON
            CUDA_RESOLVE_DEVICE_SYMBOLS ON
    )

    target_link_libraries(raymarching
        gpuRM
    )
endif()

add_executable(
    pf_node
    pf_node/src/pf_node.cpp
    src/particle_filter.cpp
)

ament_target_dependencies(
    pf_node
    rclcpp
    nav_msgs
    std_msgs
    sector_handler
    sensor_msgs
    laser_geometry
    tf2
    tf2_ros
    dta
)

target_link_libraries(
	pf_node
	uio_map
	raymarching
	yaml-cpp
    adx_data::sector
    adx_data::pose
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
)

install(TARGETS
    pf_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   launch
   conf
   DESTINATION share/${PROJECT_NAME}
)

ament_package()
