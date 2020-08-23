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
 * @file "modules/gcnet/gcnet_main.c"
 * @author Rohan Chotalal
 * This module includes the implementation and interaction of the G&CNet with the
 * lower-level control loops. Based in the "ctrl_module_outerloop_demo" files.
 */

// Header for this file
#include "modules/gcnet/gcnet_main.h"

// paparazzi-based libraries
#include "math/pprz_algebra_float.h"

#include "state.h"
#include "autopilot.h"

#include "subsystems/radio_control.h"
#include "subsystems/electrical.h"

#include "filters/low_pass_filter.h" // filters

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

// standard libraries 
#include <time.h>
#include <stdbool.h> // get "true" and "false" identifiers

// user-made libraries with:
// -- functions for nn operations
#include "modules/gcnet/nn_operations.h"
// -- variables with nn parameters (weights, biases and other information about the nets)
// #include "modules/gcnet/nn_parameters.h" // -- already in "nn_operations.h"

// declare math parameter
#ifndef PI
#define PI 3.1415
#endif

// declare environment and drone parameters
#ifndef GRAVITY_ACC
#define GRAVITY_ACC 9.81
#endif

#ifndef BEBOP_MASS 
#define BEBOP_MASS 0.38905 
#endif

// neural network state and control/action: 
float state_nn[NUM_STATES];
float control_nn[NUM_CONTROLS];

/* ---
Declare functions from this file
--- */
float timedifference_msec(struct timeval t_t0, struct timeval t_t1); 
void gcnet_control(bool in_flight);	

/* --- 
Declare important variables: 
--- */
// declare variables - time: 
// -- to obtain processing time:
struct timeval t0; 
struct timeval t1;
// -- processing time: 
float nn_process_time; 

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
// -- angles used for conversion
struct FloatEulers att_euler_OT2NED = {0.0, 0.0, 0.0};
struct FloatEulers att_euler_NED2NWU = {PI, 0.0, 0.0};
// -- quaternions
struct FloatQuat att_quat = {1, 0, 0, 0}; 

// declare variables - transformation matrices
struct FloatRMat R_OT_2_NED, R_NED_2_NWU;

// define tolerances: 
float tol = 0.3;

// desired position and yaw angle [-- make sure that this can be set up from the flight plan]
float desired_X = 5;
float desired_Y = 0; 
float desired_Z = 0; 
float desired_psi = 0;

// Neural Network State and Controls
float state_nn[NUM_STATES];
float control_nn[NUM_CONTROLS];

// -- quaternion control setpoints -t this is used because the INDI rate interface PR is not done
struct FloatQuat quat_ctrl_sp;
struct FloatEulers euler_ctrl_sp;
struct FloatRates omega_sp;

// control inputs (from receiver and own commands): 
struct ctrl_struct {
	// RC Inputs
  struct Int32Eulers rc_sp;

	// Output commanded attitude
  struct Int32Eulers cmd;

	// thrust pct
	int32_t thrust_pct; 
} ctrl;

/*
Function: Compute the time difference
*/
float timedifference_msec(struct timeval t_t0, struct timeval t_t1)
{
	return (t_t1.tv_sec - t_t0.tv_sec) * 1000.0f + (t_t1.tv_usec - t_t0.tv_usec) / 1000.0f;
}

/*
Function: Guidance and Control Network function
*/ 
void gcnet_control(UNUSED bool in_flight)
{
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

	// Compute the difference in position in the NWU frame: 
	delta_pos_NWU.x = pos_NWU.x - desired_X;
	delta_pos_NWU.y = pos_NWU.y - desired_Y;

	// Compute the value of psi_ref in the network's reference frame
	psi_ref_net = att_euler_NED2NWU.psi - desired_psi;

	// -- get coordinates in the network's reference frame (where the x-axis 
	// points to the waypoint and the y axis is perpendicular to this):
	// NOTE: 2-D coordinate transformation
	delta_pos_net.x = cosf(psi_ref_net)*delta_pos_NWU.x + sinf(psi_ref_net)*delta_pos_NWU.y;
	delta_pos_net.y = -sinf(psi_ref_net)*delta_pos_NWU.x + cosf(psi_ref_net)*delta_pos_NWU.y;

	// -- get horizontal velocity components aligned with the network's reference frame
	// NOTE: 2-D coordinate transformation
	vel_net.x = cosf(psi_ref_net)*vel_NWU.x + sinf(psi_ref_net)*vel_NWU.y;
	vel_net.y = -sinf(psi_ref_net)*vel_NWU.x + cosf(psi_ref_net)*vel_NWU.y;

	// -- transform Euler to quaternions - function is already coded on "pprz_algebra_float.c" file
	float_quat_of_eulers(&att_quat, &att_euler_NED2NWU);

	// -- assign states to the state vector that is fed to the network: 
	state_nn[0] = delta_pos_net.x;
	state_nn[1] = delta_pos_net.y; 
	state_nn[2] = pos_NWU.z - desired_Z;
	state_nn[3] = vel_net.x;
	state_nn[4] = vel_net.y;
	state_nn[5] = vel_NWU.z; 
	state_nn[6] = att_quat.qi;
	state_nn[7] = att_quat.qx;
	state_nn[8] = att_quat.qy; 
	state_nn[9] = att_quat.qz; 

	// NOTE: variable "control_nn" was declared in gcnet_main.h - encodes the network's output

	// -- feed state to the network 
	gettimeofday(&t0, 0);
	nn_control(state_nn, control_nn);
	gettimeofday(&t1, 0);
	nn_process_time = timedifference_msec(t0, t1);

	// -- Since thrust is within the interval [-m*g, m*g], we have to sum m*g
	// control_nn[0] = control_nn[0] + BEBOP_MASS*GRAVITY_ACC; // to stay between 0 and 2*m*g 
}

/**
 * Implement own horizontal loop functions 
 * NOTE: In this case, the thrust output of the network is also computed here 
 */

// Start horizontal controller
void guidance_h_module_init(void)
{ 

}

// Enter in the guidance_h module
void guidance_h_module_enter(void)
{
	// start quaternion control set-points to zero: 
	quat_ctrl_sp.qi = 1;
	quat_ctrl_sp.qx = 0;
	quat_ctrl_sp.qy = 0;
	quat_ctrl_sp.qz = 0; 

	// get the euler set-points and initialize them too: 
	float_eulers_of_quat(&euler_ctrl_sp, &quat_ctrl_sp);

	// Convert RC to setpoint
  stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

// Read the RC values 
void guidance_h_module_read_rc(void)
{
	stabilization_attitude_read_rc_setpoint_eulers(&ctrl.rc_sp, autopilot.in_flight, false, false);
}

// Run the horizontal controller
void guidance_h_module_run(bool in_flight)
{
	// in case of low batery, keep the drone in the NAV mode and hover
  /* if (electrical.bat_low) 
	{
    autopilot_static_set_mode(AP_MODE_NAV);
  } 
  else 
	{*/ 
		// 1 -- start nn controller
    gcnet_control(in_flight);
		
		// 2 -- propagate inner loop control: body thrust 
		// -- simulated thrust percentage: 
		ctrl.thrust_pct = control_nn[0]/(BEBOP_MASS*GRAVITY_ACC);
		// -- real hovering thrust percentage is not at 0.5, therefore we need an adjustment 
		if (ctrl.thrust_pct >= 0) // above m*g (if we consider interval [0, 2*m*g])
		{
			ctrl.thrust_pct = GUIDANCE_V_NOMINAL_HOVER_THROTTLE + (1 - GUIDANCE_V_NOMINAL_HOVER_THROTTLE)/(0.5 - 0)*ctrl.thrust_pct;
		}
		else // below m*g (if we consider interval [0, 2*m*g])
		{
			ctrl.thrust_pct = (GUIDANCE_V_NOMINAL_HOVER_THROTTLE - 0)/(0.5 - 0)*ctrl.thrust_pct;
		}		
		
		int32_t thrust_int = ctrl.thrust_pct * MAX_PPRZ; // see how to transform this! 

		// bound thrust:
  	Bound(thrust_int, 0, MAX_PPRZ);

		stabilization_cmd[COMMAND_THRUST] = thrust_int;

		// 2 -- inner loop controls: rates --> for the rates the noise is much less! 
		/* 
		stabilization_cmd[COMMAND_ROLL] = RATE_BFP_OF_REAL(control_nn[1]); 
		stabilization_cmd[COMMAND_PITCH] = RATE_BFP_OF_REAL(control_nn[2]); 
		stabilization_cmd[COMMAND_YAW] = RATE_BFP_OF_REAL(control_nn[3]); 
		*/
		// 3 -- Let's use attitude INDI controller instead of integrating the rates 
		// to obtain desired attitude (because the INDI rate PR is not done yet):
		// 3.1 -- Get desired rates: 
		omega_sp.p = control_nn[1];
		omega_sp.q = control_nn[2];
		omega_sp.r = control_nn[3];

		// 3.2 -- Get desired attitude by integrating the rates:  
		float_quat_integrate(&quat_ctrl_sp, &omega_sp, 1/PERIODIC_FREQUENCY);
		
		// 3.3 -- Convert desired attitude in quaternions to Euler: 
		float_eulers_of_quat(&euler_ctrl_sp, &quat_ctrl_sp);

		// 3.4 -- Conversion to int32_t: 
		ctrl.cmd.phi = ANGLE_BFP_OF_REAL(euler_ctrl_sp.phi);
		ctrl.cmd.theta = ANGLE_BFP_OF_REAL(euler_ctrl_sp.theta);
		ctrl.cmd.psi = ANGLE_BFP_OF_REAL(euler_ctrl_sp.psi);

		// 4 -- Set attitude setpoint and run the stabilization command: 
		stabilization_attitude_set_rpy_setpoint_i(&(ctrl.cmd));
  	stabilization_attitude_run(in_flight);
		
		// -- if drone within the final region, then return True and switch to another controller
		/* 
		if ((fabs(state_nn[0]) < tol) && (fabs(state_nn[1]) < tol) && (fabs(state_nn[2]) < 0.1))
			autopilot_static_set_mode(AP_MODE_HOVER_Z_HOLD);
		*/ 
  // }
}

/**
 * Implement own vertical loop functions 
 * NOTE: In this case we do not do that, but we leave these functions here
 * to allow vertical guidance using the neural network. 
 */

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  // your vertical controller goes here
}