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
 * lower-level control loops. 
 */

// Header for this file
#include "modules/gcnet/gcnet_main_v2.h"

// paparazzi-based libraries
#include "subsystems/radio_control.h"
#include "subsystems/electrical.h"
#include "subsystems/datalink/telemetry.h"

// inner loop stabilization functions
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_indi.h"

// guidance functions
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

// IMU 
#include "subsystems/imu.h" 

// Other libraries 
#include "state.h"
#include "autopilot.h"

// standard libraries 
#include <time.h>
#include <stdbool.h> // get "true" and "false" identifiers

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

/*
Thrust PID control
*/
#include "filters/low_pass_filter.h"

float thrust_effectiveness = 0.05f; // transfer function from G to thrust percentage
float error_integrator = 0.f;

#ifndef NN_FILTER_CUTOFF
#define NN_FILTER_CUTOFF 1.5f
#endif

#ifndef THRUST_P_GAIN
#define THRUST_P_GAIN -1.7
#endif

#ifndef THRUST_D_GAIN
#define THRUST_D_GAIN -0.3
#endif

#ifndef THRUST_I_GAIN
#define THRUST_I_GAIN -0.1
#endif

float thrust_p_gain = THRUST_P_GAIN;
float thrust_d_gain = THRUST_D_GAIN;
float thrust_i_gain = THRUST_I_GAIN;

Butterworth2LowPass accel_ned_filt;

static float nominal_throttle = 0.f;

/* --- 
Declare important variables (for the network) 
--- */
// neural network state and control/action: 
float state_nn[NUM_STATES];
float control_nn[NUM_CONTROLS];

// declare variables - time: 
// -- to obtain processing time:
struct timeval t0; 
struct timeval t1;
// -- processing time: 
float nn_process_time; 

// declare variables - position and velocity
// -- optitrack:
struct FloatVect3 pos_enu, vel_enu;

// -- network's frame 
struct FloatVect2 delta_pos_enu;
float psi_net;

// declare variables - attitude
// -- quaternions
struct FloatEulers att_euler_enu;
struct FloatQuat att_quat; 

// desired position and yaw angle [-- MAKE SURE THAT THIS CAN BE SET FROM THE FLIGHT PLAN]
float desired_X = 0;
float desired_Y = 5; 
float desired_Z = 1; 
float desired_psi = 0;

// control inputs (from RC and NN): 
struct ctrl_struct ctrl;

/* ---
Declare functions used in this file
--- */
float timedifference_msec(struct timeval t_t0, struct timeval t_t1); 
void gcnet_control(bool in_flight);	
void gcnet_guidance(bool in_flight);
void acceleration_z_controller(float desired_az);

/**
 * Send GCNet telemetry information to ground station
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void send_gcnet_main(struct transport_tx *trans, struct link_device *dev) 
{
	struct FloatEulers att_euler_enu_deg = {att_euler_enu.phi*180/PI, att_euler_enu.theta*180/PI, att_euler_enu.psi*180/PI};	
	
	struct FloatRates *rates = stateGetBodyRates_f();
	rates->q = -rates->q;
	rates->r = -rates->r;

	float az_nn = (control_nn[0] + BEBOP_MASS*GRAVITY_ACC)/BEBOP_MASS;
	float az_imu = -ACCEL_FLOAT_OF_BFP(imu.accel.z);

	pprz_msg_send_GCNET_MAIN(
		trans, dev, AC_ID, 
		&(stateGetPositionEnu_f()->x), &(stateGetPositionEnu_f()->y), &(stateGetPositionEnu_f()->z),
    &(stateGetSpeedEnu_f()->x), &(stateGetSpeedEnu_f()->y), &(stateGetSpeedEnu_f()->z),
		&(att_euler_enu_deg.phi), &(att_euler_enu_deg.theta), &(att_euler_enu_deg.psi), 
		&(rates->p), &(rates->q), &(rates->r),
		&(control_nn[1]), &(control_nn[2]), &(control_nn[3]),
		&(az_nn), &(az_imu) ,
		&(state.ned_origin_f.hmsl)); 
    // &autopilot.mode, &record);
}

/*
Function: Compute the time difference [miliseconds]
*/
float timedifference_msec(struct timeval t_t0, struct timeval t_t1)
{
	return (t_t1.tv_sec - t_t0.tv_sec) * 1000.0f + (t_t1.tv_usec - t_t0.tv_usec) / 1000.0f;
}

/*
Function: Guidance and Control Network function (network receives the drone's state 
and predicts the controls (taking into account the internal model used for training)
*/ 
void gcnet_control(UNUSED bool in_flight)
{
	// -- get position/velocity in ENU (East-North-Up) frame (-- cyberzoo's)
	// position -- current    
	pos_enu.x = stateGetPositionEnu_f()->x;
	pos_enu.y = stateGetPositionEnu_f()->y;
	pos_enu.z = stateGetPositionEnu_f()->z;
	// velocity -- current
	vel_enu.x = stateGetSpeedEnu_f()->x;
	vel_enu.y = stateGetSpeedEnu_f()->y;
	vel_enu.z = stateGetSpeedEnu_f()->z;
	// -- get angles 
	// angles (in the enu frame) -- current orientation
	att_euler_enu.phi = stateGetNedToBodyEulers_f()->phi; // N is same for both NED and ENU frames (so theta is the same)
	att_euler_enu.theta = -stateGetNedToBodyEulers_f()->theta; // minus sign - because we are getting the angles from the NED frame -- needs conversion
	att_euler_enu.psi = -stateGetNedToBodyEulers_f()->psi; // minus sign - because we are getting the angles from the NED frame -- needs conversion

	// Compute the difference in position in the NWU frame: 
	delta_pos_enu.x = pos_enu.x - desired_X;
	delta_pos_enu.y = pos_enu.y - desired_Y;

	// Compute the value of psi_ref in the network's reference frame
	psi_net = att_euler_enu.psi - desired_psi;

	att_euler_enu.psi = psi_net;

	// -- transform Euler to quaternions - function is already coded on "pprz_algebra_float.c" file
	float_quat_of_eulers(&att_quat, &att_euler_enu);

	// -- assign states to the state vector that is fed to the network: 
	state_nn[0] = delta_pos_enu.x;
	state_nn[1] = delta_pos_enu.y; 
	state_nn[2] = pos_enu.z - desired_Z;
	state_nn[3] = vel_enu.x;
	state_nn[4] = vel_enu.y;
	state_nn[5] = vel_enu.z; 
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

/*
Function: Wrapper of the previous function and foward network's calculations to the drone
*/
void gcnet_guidance(bool in_flight)
{
	// 1 -- start nn controller
	gcnet_control(in_flight);

	// 2 -- propagate inner loop controls: rates --> for the rates the noise is much less! 
	// 2.1 -- Get desired rates: 
	ctrl.omega_sp.p = control_nn[1];
	ctrl.omega_sp.q = -control_nn[2];
	ctrl.omega_sp.r = -control_nn[3];
	// 2.2 -- Run INDI function: 
	stabilization_indi_rate_run(ctrl.omega_sp, in_flight);
	
	// 3 -- propagate inner loop controls: body thrust --> for the thrust, az measurement is noisy!
  acceleration_z_controller(-control_nn[0]);
}

/*
Function: Internal thrust PID control (to correct the thrust of the neural network model with 
the thrust sent to the machine)
*/
void acceleration_z_controller(float desired_az)
{
  // to accumulate the integral of the error: 
	static float integrator_error = 0.f;

	// previous error (to calculate error derivative)
	static float previous_error = 0.f;

  // get acceleration (NED) 
	struct NedCoor_f *accel = stateGetAccelNed_f();
	
	// filter acceleration using a butterworth filter (because it is noisy): 
	update_butterworth_2_low_pass(&accel_ned_filt, accel->z);

	// transform from NED to Body: 
	float filtered_az = (accel_ned_filt.o[0] - GRAVITY_ACC)/cosf(stateGetNedToBodyEulers_f()->theta)/cosf(stateGetNedToBodyEulers_f()->phi);

	// get the acceleration error:
	float error_az =  desired_az - filtered_az;

	// cumulative integration error: 
	integrator_error += error_az / 100.0;

  // PID control
	float thrust_sp = (error_az*thrust_p_gain + integrator_error*thrust_i_gain + thrust_d_gain*(error_az - previous_error)/100.0)*thrust_effectiveness + nominal_throttle;
	
	// set the desired vertical thrust in the "vertical guidance module":
	guidance_v_set_guided_th(thrust_sp);

	// printf("%f\n", thrust_sp);

	// set the value of the previous error: 
	previous_error = error_az;
}

/*
Function: Init function for the GCNet module
*/
void gcnet_init(void)
{
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GCNET_MAIN, send_gcnet_main);
  
  nominal_throttle = guidance_v_nominal_throttle;

  float tau = 1.f / (2.f * M_PI * NN_FILTER_CUTOFF);
  float sample_time = 1.f / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_ned_filt, tau, sample_time, 0.0);

  guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
  guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
}

/*
Function: Run function for the GCNet module
*/
void gcnet_ctrl(void)
{
  gcnet_guidance(true);
}