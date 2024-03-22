// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.1 (64-bit)
// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
//#include <cstddef>
#include "xrm.hpp"

/************************** Function Implementation *************************/


void XRmk_Start(UioMap *rm) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_AP_CTRL,
    (XRmk_ReadReg(rm->addr, XRM_CONTROL_ADDR_AP_CTRL) & 0x80) | 0x01);
}

u32 XRmk_IsDone(UioMap *rm) {
  assert(rm != nullptr);

  return (XRmk_ReadReg(rm->addr, XRM_CONTROL_ADDR_AP_CTRL) >> 1) & 0x1;
}



void XRmk_Set_distMap(UioMap *rm, u64 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_DISTMAP_DATA, (u32)(Data));
  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_DISTMAP_DATA + 4, (u32)(Data >> 32));
}

void XRmk_Set_x(UioMap *rm, u64 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_X_DATA, (u32)(Data));
  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_X_DATA + 4, (u32)(Data >> 32));
}


void XRmk_Set_y(UioMap *rm, u64 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_Y_DATA, (u32)(Data));
  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_Y_DATA + 4, (u32)(Data >> 32));
}



void XRmk_Set_yaw(UioMap *rm, u64 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_YAW_DATA, (u32)(Data));
  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_YAW_DATA + 4, (u32)(Data >> 32));
}



void XRmk_Set_rays(UioMap *rm, u64 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_RAYS_DATA, (u32)(Data));
  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_RAYS_DATA + 4, (u32)(Data >> 32));
}

void XRmk_Set_rays_angle(UioMap *rm, u64 Data) {
    assert(rm != nullptr);

    XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_RAYS_ANGLE_DATA, (u32)(Data));
    XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_RAYS_ANGLE_DATA + 4, (u32)(Data >> 32));
}


void XRmk_Set_rm_mode(UioMap *rm, u32 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_RM_MODE_DATA, Data);
}



void XRmk_Set_map_height(UioMap *rm, u32 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_MAP_HEIGHT_DATA, Data);
}



void XRmk_Set_map_width(UioMap *rm, u32 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_MAP_WIDTH_DATA, Data);
}



void XRmk_Set_n_particles(UioMap *rm, u32 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_N_PARTICLES_DATA, Data);
}



void XRmk_Set_orig_x(UioMap *rm, u32 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_ORIG_X_DATA, Data);
}



void XRmk_Set_orig_y(UioMap *rm, u32 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_ORIG_Y_DATA, Data);
}



void XRmk_Set_map_resolution(UioMap *rm, u32 Data) {
  assert(rm != nullptr);

  XRmk_WriteReg(rm->addr, XRM_CONTROL_ADDR_MAP_RESOLUTION_DATA, Data);
}





