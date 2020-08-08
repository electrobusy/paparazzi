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
 * @file "modules/gcnet/gcnet_main.h"
 * @author Rohan Chotalal
 * This module includes the implementation and interaction of the G&CNet with the
 * lower-level control loops.  
 */

#ifndef GCNET_MAIN_H
#define GCNET_MAIN_H

#include <time.h>
#include <stdbool.h> // get "true" and "false" identifiers

// paparazzi-based library
#include "state.h"
#include "math/pprz_algebra_float.h"

// user-made libraries
#include "nn_operations.h"

// declare math variables
#ifndef PI
#define PI 3.14159
#endif

// declare environment and drone parameters
#ifndef GRAVITY_ACC
#define GRAVITY_ACC 9.81
#endif

#ifndef BEBOP_MASS 
#define BEBOP_MASS 0.38905 
#endif

#ifndef BEBOP_DRAG_X
#define BEBOP_DRAG_X -0.5
#endif

#ifndef BEBOP_DRAG_Y
#define BEBOP_DRAG_Y -0.5
#endif

// For the datasets used for the G&CNet, we considered beta_z = 0
#ifndef BEBOP_DRAG_Z
#define BEBOP_DRAG_Z 0
#endif

// declare variables - time: 
// -- to obtain processing time:
struct timeval t0; 
struct timeval t1;
// -- processing time: 
float nn_process_time; 
// [ADD MORE VARIABLES]

// declare variables - position and velocity
// -- optitrack:
struct FloatVect3 pos_OT, vel_OT;
// -- IMU:
struct FloatVect3 pos_NED, vel_NED;
// -- NN controller assumes NWU coordinate frame: 
struct FloatVect3 pos_NWU, vel_NWU;

// -- network's frame 
struct FloatVect2 delta_pos_NWU, delta_pos_net; // position
struct FloatVect2 vel_net; // velocity
float psi_ref_net;

// declare variables - attitude
// -- [CHECK]
struct FloatEulers att_euler_OT2NED = {0.0, 0.0, 0.0};
struct FloatEulers att_euler_NED2NWU = {0.0, 0.0, 0.0};
// -- quaternions
struct FloatQuat att_quat = {1, 0, 0, 0}; 

// declare variables - transformation matrices
struct FloatRMat R_OT_2_NED, R_NED_2_NWU;

// neural network state and control/action: 
float state_nn[NUM_STATE];
float control_nn[NUM_CONTROLS];

// define tolerances: 
float tol = 0.2;


#endif