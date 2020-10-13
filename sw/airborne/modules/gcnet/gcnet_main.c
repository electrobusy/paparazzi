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
#include "subsystems/radio_control.h"
#include "subsystems/electrical.h"
#include "subsystems/datalink/telemetry.h"

#include "filters/low_pass_filter.h" // filters

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "firmwares/rotorcraft/stabilization/stabilization_rate_indi.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"

#include "subsystems/navigation/waypoints.h"

#include "autopilot.h"
#include "subsystems/imu.h" // -- IMU

#include "generated/flight_plan.h" // -- check this folder

#include "modules/gcnet/nn_parameters_non_zero.h" // -- parameters for the non-zero end-velocity

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

// For the vertical (z) PI controller:
#define KP_ALT 3 // 0.12
#define KD_ALT 2 // 0.08
#define KI_ALT 2 // 0.04

#define HOVERTHRUST GUIDANCE_V_NOMINAL_HOVER_THROTTLE // 0.56

// For the horizontal (x-y) P controller:
#define KP_POS_X 1
#define KP_POS_Y 1

#define KP_VEL_X 0.8 // 0.2
#define KP_VEL_Y 0.8 // 0.2

#define CTRL_MAX_SPEED 10.0 // [m/s]
#define CTRL_MAX_ROLL 50.0*PI/180.0 // [rad]
#define CTRL_MAX_PITCH 50.0*PI/180.0 // [rad]

// Other default values
// Closed-loop thrust control, else linear transform
#define NN_ACTIVE_CONTROL true

#ifndef NN_FILTER_CUTOFF
#define NN_FILTER_CUTOFF 1.5f
#endif

#ifndef THRUST_P_GAIN
#define THRUST_P_GAIN -0.75
#endif

#ifndef THRUST_D_GAIN
#define THRUST_D_GAIN -1.0
#endif

#ifndef THRUST_I_GAIN
#define THRUST_I_GAIN -1.0
#endif

float thrust_p_gain = THRUST_P_GAIN;
float thrust_d_gain = THRUST_D_GAIN;
float thrust_i_gain = THRUST_I_GAIN;

Butterworth2LowPass accel_ned_filt_x;
Butterworth2LowPass accel_ned_filt_y;
Butterworth2LowPass accel_ned_filt_z;

struct FloatVect3 accel_ned_filt;
struct FloatVect3 acc_body_filt;
struct FloatRMat R_NED_2_BODY;

float thrust_effectiveness = 0.05f; // transfer function from G to thrust percentage
float error_integrator = 0.f;
float nominal_throttle = 0.52; // GUIDANCE_V_NOMINAL_HOVER_THROTTLE; // 0.52

/* ---
For hover controller:
--- */ 
float start_time;

/* ---
Declare functions from this file
--- */
float timedifference_msec(struct timeval t_t0, struct timeval t_t1); 
void z_PID_ctrl(float z_cmd);
void xy_PID_ctrl(float x_cmd, float y_cmd);
void gcnet_init(void);
void gcnet_control(bool in_flight);	
void gcnet_guidance(bool in_flight);
void acceleration_z_controller(float desired_az);

/* --- 
Declare important variables: 
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
struct FloatVect3 pos_OT, vel_OT;
// -- NED frame: 
struct FloatVect3 pos_NED, vel_NED;
// -- NWU frame: 
struct FloatVect3 pos_NWU, vel_NWU;
struct FloatVect2 delta_pos_NWU;
// -- network's frame (intermediate frame) 
struct FloatVect2 delta_pos_net; // position
struct FloatVect2 vel_net; // velocity
float psi_net;
struct FloatEulers att_euler_net; // attitude

// -- rotational matrices  
struct FloatRMat R_OT_2_NED, R_NED_2_NWU;
struct FloatEulers eulers_OT_2_NED = {0.0, 0.0, 0.0};
struct FloatEulers eulers_NED_2_NWU = {PI, 0.0, 0.0}; // NED reference frame rolls PI radians to become NWU

// declare variables - vehicle's attitude
// -- Eulers
struct FloatEulers att_euler_NED, att_euler_NWU;
// -- quaternions
struct FloatQuat att_quat; 

// desired position and yaw angle [-- MAKE SURE THAT THIS CAN BE SET FROM THE FLIGHT PLAN]
// --> vectors (later we will obtain the points from the flightplan -- still needs improvement)
 
float desired_X_vec[4] = {-2.5, 2.5, 2.5, -2.5};
float desired_Y_vec[4] = {2.5, 2.5, -2.5, -2.5};
float desired_Z_vec[4] = {1, 2, 2, 1};
float desired_psi_vec[4] = {PI/2, 0, -PI/2, -180*PI/180};

/* 
// -- simulation 
float desired_X_vec[4] = {0, 6, 7, 1};
float desired_Y_vec[4] = {5, 6, 1, 0};
float desired_Z_vec[4] = {1, 2, 1, 1};
float desired_psi_vec[4] = {PI/2, 0, -PI/2, -180*PI/180};
*/ 

// times at which the drone enters in the waypoints
float t_wp_entry[4] = {0, 0, 0, 0};

// --> integer to choose next point
int idx_wp;

// --> current desired values
float desired_X;
float desired_Y; 
float desired_Z; 
float desired_psi;

// control inputs (from RC or NN): 
struct ctrl_struct ctrl;
float thrust_pct_before;

// variable for PID thrust control debugging: 
struct debug_PID debug_az_PID;

// variable for PID horizontal and vertical control:
struct debug_PID_xyz debug_PID_hv;

// define tolerances (later when you reach final position)
bool mask = true;
bool mask_last_waypoint = false;
float tol_x = 0.2;
float tol_y = 0.2;
float tol_z = 0.1;

// Mask to activate zero-end network:
bool zero_end_net;

bool first_run = true;

/* ----------- FUNTIONS ----------- */

// position and velocity (for logs):
// -- OT:  
struct FloatVect3 pos_OT_v2;
struct FloatVect3 vel_OT_v2;
// -- NED:
struct FloatVect3 pos_NED_v2;
struct FloatVect3 vel_NED_v2;
// -- NWU:
struct FloatVect3 pos_NWU_v2;
struct FloatVect3 vel_NWU_v2;

// Sending stuff to ground station
// states and control inputs + thrust
static void send_gcnet_main(struct transport_tx *trans, struct link_device *dev) 
{
	// Initialize transformation matrices (which are fixed):
	float_rmat_of_eulers_321(&R_OT_2_NED,&eulers_OT_2_NED);
	float_rmat_of_eulers_321(&R_NED_2_NWU,&eulers_NED_2_NWU);

	pos_OT_v2.x = stateGetPositionNed_f()->x;
	pos_OT_v2.y = stateGetPositionNed_f()->y;
	pos_OT_v2.z = stateGetPositionNed_f()->z;

	vel_OT_v2.x = stateGetSpeedNed_f()->x;
	vel_OT_v2.y = stateGetSpeedNed_f()->y;
	vel_OT_v2.z = stateGetSpeedNed_f()->z;

	// -- transform from OT to the NED frame (the frames are the same) 
	float_rmat_transp_vmult(&pos_NED_v2, &R_OT_2_NED, &pos_OT_v2);
	float_rmat_transp_vmult(&vel_NED_v2, &R_OT_2_NED, &vel_OT_v2);

	// -- transform to the NWU (North-West-Up = x-y-z) frame 
	float_rmat_transp_vmult(&pos_NWU_v2, &R_NED_2_NWU, &pos_NED_v2);
	float_rmat_transp_vmult(&vel_NWU_v2, &R_NED_2_NWU, &vel_NED_v2);

	// -- get angles 
	// angles (in the NED frame) -- current orientation
	att_euler_NED.phi = stateGetNedToBodyEulers_f()->phi; 
	att_euler_NED.theta = stateGetNedToBodyEulers_f()->theta; 
	att_euler_NED.psi = stateGetNedToBodyEulers_f()->psi; 

	// transform to NWU frame
	att_euler_NWU.phi = att_euler_NED.phi; // N is same for both NED and NWU frames (so theta is the same)
	att_euler_NWU.theta = -att_euler_NED.theta; // minus sign - because we are getting the angles from the NED frame -- needs conversion
	att_euler_NWU.psi = -att_euler_NED.psi; // minus sign - because we are getting the angles from the NED frame -- needs conversion

	struct FloatEulers att_euler_NWU_deg = {att_euler_NWU.phi*180/PI, att_euler_NWU.theta*180/PI, att_euler_NWU.psi*180/PI};	
	
	struct FloatRates *rates = stateGetBodyRates_f();
	rates->q = -rates->q;
	rates->r = -rates->r;

	struct NedCoor_f *accel = stateGetAccelNed_f();

	float az_nn_model = (control_nn[0] + BEBOP_MASS*GRAVITY_ACC)/BEBOP_MASS;
	float az_nn_drone = ctrl.thrust_pct;
	float az_imu = -ACCEL_FLOAT_OF_BFP(imu.accel.z);

	pprz_msg_send_GCNET_MAIN(
		trans, dev, AC_ID, 
		&(pos_OT_v2.x), &(pos_OT_v2.y), &(pos_OT_v2.z),
    &(vel_NWU_v2.x), &(vel_NWU_v2.y), &(vel_NWU_v2.z),
		&(att_euler_NWU_deg.phi), &(att_euler_NWU_deg.theta), &(att_euler_NWU_deg.psi), 
		&(rates->p), &(rates->q), &(rates->r),
		&(control_nn[1]), &(control_nn[2]), &(control_nn[3]),
		&(accel->z), &(ctrl.thrust_pct), &(az_imu),
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
Function: Guidance and Control Network Init Function
*/
void gcnet_init(void)
{
	// Initialize transformation matrices (which are fixed):
	float_rmat_of_eulers_321(&R_OT_2_NED,&eulers_OT_2_NED);
  float_rmat_of_eulers_321(&R_NED_2_NWU,&eulers_NED_2_NWU);

	// If we use the PID control to adjust the net to drone throttle 
	if(NN_ACTIVE_CONTROL)
	{
		float sample_time = 1.f / PERIODIC_FREQUENCY;
		float tau = 1.f / (2.f * M_PI * NN_FILTER_CUTOFF);
		init_butterworth_2_low_pass(&accel_ned_filt_x, tau, sample_time, 0.0);
		init_butterworth_2_low_pass(&accel_ned_filt_y, tau, sample_time, 0.0);
		init_butterworth_2_low_pass(&accel_ned_filt_z, tau, sample_time, 0.0);
	}

	// Entering the neighbourhood area: 
	t_wp_entry[0] = 0;
	t_wp_entry[1] = 0;
	t_wp_entry[2] = 0;
	t_wp_entry[3] = 0;

	// idx_wp initialization:
	idx_wp = 0;

	// Initialize desired position and yaw angle: 
	desired_X = -2.5;
	desired_Y = 2.5;
	desired_Z = 1;
	desired_psi = 0;

	zero_end_net = true;
}	

/*
Function: Guidance and Control Network function
*/ 
void gcnet_control(UNUSED bool in_flight)
{
	// -- get position/velocity in OT, which is the same as NED (North-East-Down = x-y-z) frame 
	// position -- current    
	pos_OT.x = stateGetPositionNed_f()->x;
	pos_OT.y = stateGetPositionNed_f()->y;
	pos_OT.z = stateGetPositionNed_f()->z;
	// velocity -- current
	vel_OT.x = stateGetSpeedNed_f()->x;
	vel_OT.y = stateGetSpeedNed_f()->y;
	vel_OT.z = stateGetSpeedNed_f()->z;

	// -- transform from OT to the NED frame (the frames are the same) 
	float_rmat_transp_vmult(&pos_NED, &R_OT_2_NED, &pos_OT);
	float_rmat_transp_vmult(&vel_NED, &R_OT_2_NED, &vel_OT);

	// -- transform to the NWU (North-West-Up = x-y-z) frame 
	float_rmat_transp_vmult(&pos_NWU, &R_NED_2_NWU, &pos_NED);
	float_rmat_transp_vmult(&vel_NWU, &R_NED_2_NWU, &vel_NED);

	// -- get angles 
	// angles (in the NED frame) -- current orientation
	att_euler_NED.phi = stateGetNedToBodyEulers_f()->phi; 
	att_euler_NED.theta = stateGetNedToBodyEulers_f()->theta; 
	att_euler_NED.psi = stateGetNedToBodyEulers_f()->psi; 

	// transform to NWU frame
	att_euler_NWU.phi = att_euler_NED.phi; // N is same for both NED and NWU frames (so theta is the same)
	att_euler_NWU.theta = -att_euler_NED.theta; // minus sign - because we are getting the angles from the NED frame -- needs conversion
	att_euler_NWU.psi = -att_euler_NED.psi; // minus sign - because we are getting the angles from the NED frame -- needs conversion

	// Compute the difference in position in the NWU frame: 
	delta_pos_NWU.x = pos_NWU.x - desired_X;
	delta_pos_NWU.y = pos_NWU.y - desired_Y;

	// -- get coordinates in the network's reference frame (where the x-axis 
	// points to the waypoint and the y axis is perpendicular to this):
	// NOTE: 2-D coordinate transformation
	delta_pos_net.x = cosf(desired_psi)*delta_pos_NWU.x + sinf(desired_psi)*delta_pos_NWU.y;
	delta_pos_net.y = -sinf(desired_psi)*delta_pos_NWU.x + cosf(desired_psi)*delta_pos_NWU.y;

	// -- get horizontal velocity components aligned with the network's reference frame
	// NOTE: 2-D coordinate transformation
	vel_net.x = cosf(desired_psi)*vel_NWU.x + sinf(desired_psi)*vel_NWU.y;
	vel_net.y = -sinf(desired_psi)*vel_NWU.x + cosf(desired_psi)*vel_NWU.y;

	// Compute the value of psi_ref in the network's reference frame
	psi_net = att_euler_NWU.psi - desired_psi;

	// psi in [-pi,pi]. If we surpass the boundaries, we have to sum or subtract 2*pi
	if(psi_net < -PI)
	{
		psi_net = psi_net + 2*PI;
	}
	else if(psi_net > PI)
	{
		psi_net = psi_net - 2*PI;
	}
	
	// -- attitude in Euler sent to the network
	att_euler_net.phi = att_euler_NWU.phi;
	att_euler_net.theta = att_euler_NWU.theta;
	att_euler_net.psi = psi_net;

	// -- transform Euler to quaternions - function is already coded on "pprz_algebra_float.c" file
	float_quat_of_eulers(&att_quat, &att_euler_net);

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

	if(zero_end_net) // zero end-velocity network
	{
		nn_control(state_nn, control_nn);
	} 
	else // non-zero end-velocity network
	{
		float state_norm[NUM_STATES];

    // 1 - pre-processing input
    preprocess_input(state_nn, state_norm, in_norm_mean_nz, in_norm_std_nz);

    // 2 - neural network prediction
    nn_predict(state_norm, control_nn, weights_in_nz, bias_in_nz, weights_hid_nz, bias_hid_nz, weights_out_nz, bias_out_nz);
    
    // 3 - post-processing output
    postprocess_output(control_nn, out_scale_min_nz, out_scale_max_nz, out_norm_mean_nz, out_norm_std_nz);
	} 
	
	gettimeofday(&t1, 0);
	nn_process_time = timedifference_msec(t0, t1); 	
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
	
	// 3 -- propagate inner loop controls: body thrust --> for the thrust, IMU is noisy
	if (NN_ACTIVE_CONTROL)
	{
		// -- Since thrust is within the interval [-m*g, m*g], we have to sum m*g
		// control_nn[0] = (control_nn[0] + BEBOP_MASS*GRAVITY_ACC)/BEBOP_MASS; // to stay between 0 and 2*m*g 

		// -- Use PID controller
		acceleration_z_controller(-(control_nn[0] + BEBOP_MASS*GRAVITY_ACC)/BEBOP_MASS);
	}
	else
	{
		// -- throttle in pct from nn:
		thrust_pct_before = control_nn[0]/(BEBOP_MASS*GRAVITY_ACC)/2;

		/* ANOTHER WAY OF CONVERSION -- TWO LINES
		// -- real hovering thrust percentage is not at 0.5, therefore we need an adjustment 
		if (thrust_pct_before >= 0) // above m*g (if we consider interval [0, 2*m*g])
		{
			Bound(thrust_pct_before, 0, 0.5);
			ctrl.thrust_pct = GUIDANCE_V_NOMINAL_HOVER_THROTTLE + (1 - GUIDANCE_V_NOMINAL_HOVER_THROTTLE)/(0.5 - 0)*thrust_pct_before;
		}
		else // below m*g (if we consider interval [0, 2*m*g])
		{
			Bound(thrust_pct_before, -0.5, 0);
			ctrl.thrust_pct = (GUIDANCE_V_NOMINAL_HOVER_THROTTLE - 0)/(0.5 - 0)*(-thrust_pct_before);
		}	
		*/ 

		// -- convert to throttle, in pct, sent to the drone
		ctrl.thrust_pct = thrust_pct_before*2*(nominal_throttle/0.5);

		// -- convert to integer
		int32_t thrust_int = ctrl.thrust_pct * MAX_PPRZ; // see how to transform this! 
	
		// -- bound it:
		Bound(thrust_int, 0, MAX_PPRZ);

		// -- send command
		stabilization_cmd[COMMAND_THRUST] = thrust_int;  

	}

	// printf("%f \t %f \t %f \t %f \t %f \t %d \t %f \t %f \t %f \t %f \t %f (x,y,z,psi,nn_time,idx_wp, imu_x, imu_y, imu_z, nominal_throttle, nominal throttle - pprz) \n", fabs(state_nn[0]), fabs(state_nn[1]), fabs(state_nn[2]), att_euler_NWU.psi*180/PI, nn_process_time, idx_wp, ACCEL_FLOAT_OF_BFP(imu.accel.x), ACCEL_FLOAT_OF_BFP(imu.accel.y), ACCEL_FLOAT_OF_BFP(imu.accel.z), nominal_throttle, GUIDANCE_V_NOMINAL_HOVER_THROTTLE);
	
	// if drone within the waypoint's neighbourhood:
	if ((fabs(state_nn[0]) < tol_x) && (fabs(state_nn[1]) < tol_y) && (fabs(state_nn[2]) < tol_z))
	{ 
		/*
		if(mask_last_waypoint)
		{
			t_wp_entry[idx_wp] = get_sys_time_float();
			mask_last_waypoint = false;
			idx_wp = 4;
		}
		*/
		
		// if last waypoint is activated (considering that we have 4 waypoints) 
		/* 
		if(idx_wp == 3)
		{  
			zero_end_net = true; 
			mask_last_waypoint = true;
			*/ 
			// idx_wp = 0; 
			/* 
			if(mask) 
			{
				guidance_h_hover_enter();
				// guidance_v_init(); -- the _v_z_enter() function already initializes with the required GUIDED mode
				guidance_v_z_enter();
				mask = false;
				printf("Hover here now!\n");
			}
			// guidance_v_guided_mode = GUIDANCE_V_GUIDED_MODE_ZHOLD;
			guidance_h_guided_run(in_flight);
			guidance_v_guided_run(in_flight); */
			 
			printf("Hello Jelle, I am hovering badly with the net!\n"); 
			if(mask)
			{
				t_wp_entry[0] = get_sys_time_float();
				mask = false;
			}	
		/*	 
		}
		else // else change waypoint 
		{
			// this is because of the last waypoint (so we can log the time)
			if (idx_wp < 4)
			{
				t_wp_entry[idx_wp] = get_sys_time_float();
				idx_wp = idx_wp + 1;

				printf("Change Waypoint!\n");

				desired_X = desired_X_vec[idx_wp];
				desired_Y = desired_Y_vec[idx_wp]; 
				desired_Z = desired_Z_vec[idx_wp]; 
				desired_psi = desired_psi_vec[idx_wp]; 
			}
		}
		*/
	}
}

/*
Function: PID throttle feedback control
*/
void acceleration_z_controller(float desired_az)
{
  // to accumulate the integral of the error: 
	static float integrator_error = 0.f;

	// previous error (to calculate error derivative)
	static float previous_error = 0.f;

  // get acceleration (from Opti-Track)
	struct NedCoor_f *accel = stateGetAccelNed_f();

	/*
	float imu_x = ACCEL_FLOAT_OF_BFP(imu.accel.x);
	float imu_y = ACCEL_FLOAT_OF_BFP(imu.accel.y);
	float imu_z = ACCEL_FLOAT_OF_BFP(imu.accel.z);
	*/
	
	// filter acceleration using a butterworth filter (because it is noisy): 
	update_butterworth_2_low_pass(&accel_ned_filt_x, accel->x);
	update_butterworth_2_low_pass(&accel_ned_filt_y, accel->y);
	update_butterworth_2_low_pass(&accel_ned_filt_z, accel->z);

	accel_ned_filt.x = accel_ned_filt_x.o[0];
	accel_ned_filt.y = accel_ned_filt_y.o[0];
	accel_ned_filt.z = accel_ned_filt_z.o[0] - GRAVITY_ACC;

	// transform from NED to Body: 
	float_rmat_of_eulers_321(&R_NED_2_BODY, &att_euler_NED);

	float_rmat_vmult(&acc_body_filt, &R_NED_2_BODY, &accel_ned_filt);

	float filtered_az = acc_body_filt.z;

	// --- DEBUG ---
	debug_az_PID.acc_ned_z = accel->z;
	debug_az_PID.acc_ned_filt_x = accel_ned_filt.x;
	debug_az_PID.acc_ned_filt_y = accel_ned_filt.y;
	debug_az_PID.acc_ned_filt_z = accel_ned_filt.z;
	debug_az_PID.desired_az = desired_az;
	debug_az_PID.filtered_az = filtered_az;
	// -------------

	// transform from NED to Body - using simplication of psi ~ 0: 
	// float filtered_az = (accel_ned_filt.o[0] - GRAVITY_ACC)/cosf(stateGetNedToBodyEulers_f()->theta)/cosf(stateGetNedToBodyEulers_f()->phi);

	// get the acceleration error:
	float error_az =  desired_az - filtered_az;

	// --- DEBUG ---
	debug_az_PID.error_az = error_az;
	// -------------

	// cumulative integration error: 
	integrator_error += error_az / PERIODIC_FREQUENCY;
	ctrl.thrust_pct = (error_az*thrust_p_gain + integrator_error*thrust_i_gain + thrust_d_gain*(error_az - previous_error)/PERIODIC_FREQUENCY)*thrust_effectiveness + nominal_throttle;

	// --- DEBUG ---
	debug_az_PID.integrator_error = integrator_error;
	debug_az_PID.derivative_error = (error_az - previous_error)/PERIODIC_FREQUENCY;
	// -------------

	// set the desired vertical thrust in the "vertical guidance module":
	// guidance_v_set_guided_th(thrust_sp); // this only works on GUIDANCE_V_MODE_GUIDED. But we are on GUIDANCE_V_MODE_MODULE 

	int32_t thrust_int = ctrl.thrust_pct * MAX_PPRZ; // see how to transform this! 
	
	// bound thrust:
	Bound(thrust_int, 0, MAX_PPRZ);

	stabilization_cmd[COMMAND_THRUST] = thrust_int;  

	// set the value of the previous error: 
	previous_error = error_az;

	/* 
	printf("==============================================\n");		
	printf("%f \t %f \t %f (az, az_filt, filt_az) \n", accel->z, accel_ned_filt.o[0], filtered_az);		
	printf("%f \t %f (e_az, ie_az) \n", error_az, integrator_error);
	printf("%f (thrust_pct - drone) \n", ctrl.thrust_pct);		
	printf("==============================================\n");		
	*/ 
}

/*
Function: z (vertical) PI controller 
NOTE: z should be negative
*/
void z_PID_ctrl(float pos_cmd)
{

	static float pos_error_int = 0;

	// get current height (in NED)
  float z_m = stateGetPositionNed_f()->z;
	// get current velocity (in NED)
  float vz_m = stateGetSpeedNed_f()->z;
  
	// P gain
  float pos_error = pos_cmd - z_m;
  
	// I gain
	pos_error_int += pos_error / PERIODIC_FREQUENCY;

	Bound(pos_error_int, -0.1, 0.1)
  
	// thrust command
  float thrust_cmd = - KP_ALT * pos_error - KI_ALT * pos_error_int - KD_ALT * (0 - vz_m) + HOVERTHRUST/(cosf(stateGetNedToBodyEulers_f()->theta)*cosf(stateGetNedToBodyEulers_f()->phi));

  if(thrust_cmd > 0.8)
	{
    thrust_cmd = 0.8;
  }
	
  stabilization_cmd[COMMAND_THRUST] = thrust_cmd * MAX_PPRZ; // 9125.0;

	debug_PID_hv.z_cmd = pos_cmd;
	debug_PID_hv.z_error_int = pos_error_int;
	debug_PID_hv.z_error = pos_error;
	debug_PID_hv.thrust_cmd = thrust_cmd;
} 

/*
Function: x-y (horizontal) P control
*/ 
void xy_PID_ctrl(float x_cmd, float y_cmd)
{
	// -- POSITION LOOP 
	// get current position
	float x_curr = stateGetPositionNed_f()->x;
	float y_curr = stateGetPositionNed_f()->y;

	// my friends, this computation is done in NED 
	float error_x = x_cmd - x_curr;
	float error_y = y_cmd - y_curr;

	// cosine and sine of psi
	float psi = stateGetNedToBodyEulers_f()->psi; 
	float cpsi = cosf(psi);
	float spsi = sinf(psi);

	// velocity command
	float vx_cmd = KP_POS_X*error_x;
	float vy_cmd = KP_POS_Y*error_y;

	// transform position errors to velocity frame (rotation in psi) -- error is the desired speed - position_error * 1 (where 1 is the gain)
	float vx_cmd_vel_frame = cpsi*vx_cmd + spsi*vy_cmd; 
	float vy_cmd_vel_frame = -spsi*vx_cmd + cpsi*vy_cmd;

	// bound the error:
	Bound(vx_cmd_vel_frame,-CTRL_MAX_SPEED,CTRL_MAX_SPEED);
	Bound(vy_cmd_vel_frame,-CTRL_MAX_SPEED,CTRL_MAX_SPEED);

	// -- VELOCITY LOOP
	// get current velocity
	float vx_curr = stateGetSpeedNed_f()->x;
	float vy_curr = stateGetSpeedNed_f()->y; 

	// transform current velocity to the velocity frame (same rotation in psi):
	float vx_curr_vel_frame = cpsi*vx_curr + spsi*vy_curr; 
	float vy_curr_vel_frame = -spsi*vx_curr + cpsi*vy_curr;

	// get the attitude commands (position controller) -> NOTE: X affects theta and Y affects roll
	float phi_cmd = KP_VEL_Y*(vy_cmd_vel_frame - vy_curr_vel_frame);
	float theta_cmd = KP_VEL_X*(vx_cmd_vel_frame - vx_curr_vel_frame);

	// bound the attitude commands:
	Bound(phi_cmd,-CTRL_MAX_ROLL,CTRL_MAX_ROLL);
	Bound(theta_cmd,-CTRL_MAX_PITCH,CTRL_MAX_PITCH);
	
	struct FloatEulers cmd;
	struct FloatQuat cmd_quat;

	cmd.phi = 0.0;
	cmd.theta = 0.0;
	cmd.psi = psi; // we want psi to be zero too

	float_quat_of_eulers(&cmd_quat, &cmd);

	struct Int32Quat cmd_quat_int;

	QUAT_BFP_OF_REAL(cmd_quat_int,cmd_quat);

	/* 
	stabilization_attitude_set_rpy_setpoint_i(&cmd);
  stabilization_attitude_run(true);
	*/ 

	stabilization_indi_attitude_run(cmd_quat_int, true);


	debug_PID_hv.x_cmd = x_cmd;
	debug_PID_hv.y_cmd = y_cmd;
	debug_PID_hv.error_x = error_x;
	debug_PID_hv.error_y = error_y;
	debug_PID_hv.error_x_vel_frame = vx_cmd_vel_frame;
	debug_PID_hv.error_y_vel_frame = vx_cmd_vel_frame;
	debug_PID_hv.vx_curr_vel_frame = vx_curr_vel_frame;
	debug_PID_hv.vy_curr_vel_frame = vy_curr_vel_frame;
	debug_PID_hv.phi_cmd = cmd.phi;
	debug_PID_hv.theta_cmd = cmd.theta;
	debug_PID_hv.psi_cmd = cmd.psi;
}


/**
 * Implement own horizontal loop functions 
 * NOTE: In this case, the thrust output of the network is also computed here 
 */

// Start horizontal controller
void guidance_h_module_init(void)
{ 
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GCNET_MAIN, send_gcnet_main);
}

// Enter in the guidance_h module
void guidance_h_module_enter(void)
{
	autopilot_set_motors_on(true);

	start_time = get_sys_time_float();

	// initialize certain variables to execute the gcnet 
	gcnet_init();
	guidance_v_init(); // this lets use the GUIDANCE_V_NOMINAL_HOVER_THROTTLE variable
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
  if (electrical.bat_low) 
	{
    autopilot_static_set_mode(AP_MODE_NAV);
  } 
  else 
	{
		// float des_x = 2.0;
		// float des_y = 2.0;
		// float des_z = 1.0;

		/* 
		// -- transform from OT to the NED frame (the frames are the same) 
		float_rmat_transp_vmult(&pos_NED, &R_OT_2_NED, &pos_OT);

		// -- transform to the NWU (North-West-Up = x-y-z) frame 
		float_rmat_transp_vmult(&pos_NWU, &R_NED_2_NWU, &pos_NED);
		*/ 

		// 1 - PID that leads drone to desired initial point: 
		// xy_PID_ctrl(des_x,des_y);
		// z_PID_ctrl(-des_z);

		// 2 - After reaching the desired point, estimate norminal thrust for 5-10 seconds: 

		// 3 - Activate the GCNET mode:

		/*
		// get the nominal of the drone to estimate the nominal thrust:  
  	if (first_run) {
    	start_time = get_sys_time_float();
    	nominal_throttle = (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    	first_run = false;
  	}*/

  	// Let the vehicle hover 
  	// if (get_sys_time_float() - start_time < 5.0f) 
		// {
		// hovering_ctrl(-1.0);
    	// return;
  	// }
		
		/*use neural network to guide drone */
		gcnet_guidance(in_flight);

		/* 
		if(get_sys_time_float() - start_time < 2)
		{
			printf("Time = %f \n", get_sys_time_float() - start_time);
			printf("I am hovering\n");
		}
		else
		{
			printf("Network time, baby\n");
		*/
		
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