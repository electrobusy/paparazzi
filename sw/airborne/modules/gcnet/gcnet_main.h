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
 * lower-level control loops.  Based in the "ctrl_module_outerloop_demo" files.
 */

#ifndef GCNET_MAIN_H
#define GCNET_MAIN_H

// paparazzi library with standard definitions
#include "std.h" 

// paparazzi-based libraries
#include "math/pprz_algebra_float.h"

#include "state.h"


// Set the both "guidance_h" and "guidance_v" modules using the "module" mode
// NOTE: Therefore, it automatically uses the guidance_*_module functions implemented here 
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// user-made libraries with:
// -- functions for nn operations
#include "modules/gcnet/nn_operations.h"
// -- variables with nn parameters (weights, biases and other information about the nets)
// #include "modules/gcnet/nn_parameters_non_zero.h" // -- already in "nn_operations.h"

// control inputs (from receiver and own commands): 
struct ctrl_struct {
	// RC Inputs
  struct Int32Eulers rc_sp;

	// Output commanded attitude
  struct FloatRates omega_sp;

	// thrust pct
	float thrust_pct; 
};

/* --- 
Variables declared here because of logs -- go to "file_logger.c" in modules/loggers/file_logger.c
That is why I added all variables as "extern", so that file can access these variables from "gcnet_main.c" file
--- */
// neural network state and control/action: 
extern float state_nn[NUM_STATES];
extern float control_nn[NUM_CONTROLS];

// declare variables - time: 
// -- to obtain processing time:
// extern struct timeval t0; 
// extern struct timeval t1;
// -- processing time: 
extern float nn_process_time; 

// declare variables - position and velocity
// -- optitrack:
extern struct FloatVect3 pos_OT, vel_OT;
// -- NED frame: 
extern struct FloatVect3 pos_NED, vel_NED;
// -- NWU frame: 
extern struct FloatVect3 pos_NWU, vel_NWU;
extern struct FloatVect2 delta_pos_NWU;
// -- network's frame (intermediate frame) 
extern struct FloatVect2 delta_pos_net; // position
extern struct FloatVect2 vel_net; // velocity
extern float psi_net;

// -- rotational matrices  
extern struct FloatRMat R_OT_2_NED, R_NED_2_NWU;
extern struct FloatEulers eulers_OT_2_NED;
extern struct FloatEulers eulers_NED_2_NWU; // NED reference frame rolls PI radians to become NWU

// declare variables - vehicle's attitude
// -- Eulers
extern struct FloatEulers att_euler_NED, att_euler_NWU;
// -- quaternions
extern struct FloatQuat att_quat; 

// desired position and yaw angle [-- MAKE SURE THAT THIS CAN BE SET FROM THE FLIGHT PLAN]
extern float desired_X;
extern float desired_Y; 
extern float desired_Z; 
extern float desired_psi;

// control inputs (from RC or NN): 
extern struct ctrl_struct ctrl;
extern float thrust_pct_before;

// tolerances
extern float tol_x;
extern float tol_y;
extern float tol_z;

// Variable used to debug PID thrust control: 
struct debug_PID
{
	float acc_ned_z;
	float acc_ned_filt_x;
	float acc_ned_filt_y;
	float acc_ned_filt_z;
	float desired_az;
	float filtered_az;
	float error_az;
	float integrator_error;
	float derivative_error;
};

extern struct debug_PID debug_az_PID;

// define tolerances (later when you reach final position)
// extern float tol;

// GUIDANCE LOOPS: 
// Implement own Horizontal loops: 
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

// Implement own Vertical loops - even though we don't use them 
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif