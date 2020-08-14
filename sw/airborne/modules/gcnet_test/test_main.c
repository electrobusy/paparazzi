/*
 * Copyright (C) Rohan Chotalal
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/gcnet_test/test_main.c"
 * @author Rohan Chotalal
 * This module includes the implementation and interaction of the G&CNet with the
 * lower-level control loops.  
 */

// standard C libraries
#include <stdio.h>

// Header for this file
#include "modules/gcnet_test/test_main.h"

// guidance libraries (already in paparazzi) 
#include "firmwares/rotorcraft/guidance/guidance_v.h" // access functions for vertical guidance
#include "firmwares/rotorcraft/guidance/guidance_h.h" // access functions for horizontal guidance

// get access to library associated to filters 
#include "filters/low_pass_filter.h"

// Settings: 
#ifndef NN_FILTER_CUTOFF
#define NN_FILTER_CUTOFF 1.5f
#endif

// ----
// declare variables - position and velocity
// -- optitrack:
struct FloatVect3 pos_OT, vel_OT;
// -- IMU:
struct FloatVect3 pos_NED, vel_NED;
// -- NN controller assumes NWU coordinate frame: 
struct FloatVect3 pos_NWU, vel_NWU;
// -- Extra variables: 
struct FloatVect3 hoverPos;
struct FloatEulers hoverEuler;

// declare variables - attitude
// -- [CHECK]
struct FloatEulers att_euler_OT2NED = {0.0, 0.0, 0.0};
struct FloatEulers att_euler_NED2NWU = {0.0, 0.0, 0.0};
// -- quaternions
struct FloatQuat att_quat = {1, 0, 0, 0}; 

// declare variables - transformation matrices
struct FloatRMat R_OT_2_NED, R_NED_2_NWU;

// declare filtering variables
Butterworth2LowPass accel_ned_filt;

/*
Function: Hovering quadrotor (in this case, using information from optitrack) -- initialize
*/
static void hovering_quad_init(void) // (float hover_time)
{
    // select modes for the horizontal and vertical guidance modules:
    guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
    guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);

    // get current position
    hoverPos.x = stateGetPositionNed_f()->x;
    hoverPos.y = stateGetPositionNed_f()->y;
    hoverPos.z = stateGetPositionNed_f()->z;
    // set psi to 0
    hoverEuler.psi = 0;

    // check if enters in this module
    printf("Hover is initialized! \n");
    printf("We are inside the module: let us start hovering! \n");

    // define filtering parameters and initialize filter
    float ts = 1.f / PERIODIC_FREQUENCY;
    float tau = 1.f / (2.f * M_PI * NN_FILTER_CUTOFF);
    init_butterworth_2_low_pass(&accel_ned_filt, tau, ts, 0.0f);
}

/*
Function: Hovering quadrotor (in this case, using information from optitrack) -- constantly running
*/
static void hovering_quad_run(void)
{
    // [CHECK THIS TOO]
    att_euler_OT2NED.phi = stateGetNedToBodyEulers_f()->phi;
    att_euler_OT2NED.theta = stateGetNedToBodyEulers_f()->theta;
    att_euler_OT2NED.psi = stateGetNedToBodyEulers_f()->psi;

    // [CHECK THIS]
    att_euler_NED2NWU.phi = -stateGetNedToBodyEulers_f()->phi;
    att_euler_NED2NWU.theta = stateGetNedToBodyEulers_f()->theta;
    att_euler_NED2NWU.psi = -stateGetNedToBodyEulers_f()->psi;

    // -- calculate rotational matrices (in euler angles)
    float_rmat_of_eulers_321(&R_OT_2_NED, &att_euler_OT2NED); 
    float_rmat_of_eulers_321(&R_NED_2_NWU, &att_euler_NED2NWU);

    // -- get coordinates in NED (North-East-Down) frame (-- cyberzoo's)
    // position:    
    pos_OT.x = stateGetPositionNed_f()->x;
    pos_OT.y = stateGetPositionNed_f()->y;
    pos_OT.z = stateGetPositionNed_f()->z;
    // velocity: 
    vel_OT.x = stateGetSpeedNed_f()->x;
    vel_OT.y = stateGetSpeedNed_f()->y;
    vel_OT.z = stateGetSpeedNed_f()->z;
    
    // -- transform coordinates from NED to NWU (North-West-Up) frame: 
    float_rmat_transp_vmult(&pos_NED, &R_OT_2_NED, &pos_OT);
    float_rmat_transp_vmult(&vel_NED, &R_OT_2_NED, &vel_OT);

    float_rmat_transp_vmult(&pos_NWU, &R_NED_2_NWU, &pos_NED);
    float_rmat_transp_vmult(&vel_NWU, &R_NED_2_NWU, &vel_NED);
   
    // -- get quad's acceleration in the Z axis: 
    struct NedCoor_f *accel = stateGetAccelNed_f();

    // -- filter it, since it is usually noisy (wether it is from optitrack or IMU)
    update_butterworth_2_low_pass(&accel_ned_filt, accel->z);

    // -- remove gravity 
    float filtered_az = (accel_ned_filt.o[0] - GRAVITY_ACC)/cosf(stateGetNedToBodyEulers_f()->theta);
   
    guidance_h_set_guided_pos(0.0, 0.0); // position (x,y) = (0,0)
    guidance_v_set_guided_z(-5); // position z = 5 m
    guidance_h_set_guided_heading(0.0); // psi = 0

    // printf('az = %f\n', filtered_az);

    // printf("Event started \n");
}

// Module functions
// Init
void gcnet_init() { hovering_quad_init(); }

// Run
void gcnet_run() { hovering_quad_run(); }