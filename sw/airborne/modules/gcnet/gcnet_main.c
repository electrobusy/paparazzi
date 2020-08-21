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

// get filters 
#include "filters/low_pass_filter.h"

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
// -- [CHECK]
struct FloatEulers att_euler_OT2NED = {0.0, 0.0, 0.0};
struct FloatEulers att_euler_NED2NWU = {0.0, 0.0, 0.0};
// -- quaternions
struct FloatQuat att_quat = {1, 0, 0, 0}; 

// declare variables - transformation matrices
struct FloatRMat R_OT_2_NED, R_NED_2_NWU;

// define tolerances: 
float tol = 0.2;

// desired position and yaw angle: 
float desired_X = 5;
float desired_Y = 0; 
float desired_Z = 0; 
float desired_psi = 0;

// Neural Network State and Controls
float state_nn[NUM_STATES];
float control_nn[NUM_CONTROLS];

/*
Function: Compute the time difference
*/
float timedifference_msec(struct timeval t_t0, struct timeval t_t1)
{
	return (t_t1.tv_sec - t_t0.tv_sec) * 1000.0f + (t_t1.tv_usec - t_t0.tv_usec) / 1000.0f;
}

// gcnet_control function: 
bool gcnet_control(bool in_flight)
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
	control_nn[0] = control_nn[0] + BEBOP_MASS*GRAVITY_ACC; // to stay between 0 and 2*m*g 
}

/**
 * Implement own horizontal loop functions 
 * NOTE: In this case, the thrust output of the network is used 
 */

// Start horizontal controller
void guidance_h_module_init(void)
{ 

}

// Enter in the guidance_h module
void guidance_h_module_enter(void)
{
	stabilization_indi_set_rpy_setpoint_i()
}

// Read the RC values 
void guidance_h_module_read_rc(void)
{

}

// Run the horizontal controller
void guidance_h_module_run(bool in_flight)
{
	// in case of low batery, keep the drone in the NAV mode and hover
  if (electrical.bat_low) 
	{
    autopilot_static_set_mode(AP_MODE_NAV);
  } 
  else 
	{
		// [TODO -- add an if statement that lets us choose between hover and nn control mode]
    gcnet_control(in_flight);

		// -- inner loop controls: thrust 
		stabilization_cmd[COMMAND_THRUST] = control_nn[0];
		
		// Shuo's approach -- use the guidance_v module: -- CODE THIS

		// -- inner loop controls: rates // for the rates the noise is much less! 
		stabilization_cmd[COMMAND_ROLL] = RATE_BFP_OF_REAL(control_nn[1]); 
		stabilization_cmd[COMMAND_PITCH] = RATE_BFP_OF_REAL(control_nn[2]); 
		stabilization_cmd[COMMAND_YAW] = RATE_BFP_OF_REAL(control_nn[3]); 
		
		// -- if drone within the final region, then return True and switch to another controller
		if ((fabs(state_nn[0]) < tol) && (fabs(state_nn[1]) < tol) && (fabs(state_nn[2]) < 0.1))
			autopilot_static_set_mode(AP_MODE_NAV);
  }
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