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

// For hover controller:
#define KP_ALT 2
#define KD_ALT 0.3
#define KI_ALT 0.1

#define KP_Z 3.0
#define KP_VZ 3.0
#define KP_VZDOT 0.1

#define HOVERTHRUST 0.56

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
#define THRUST_D_GAIN -1
#endif

#ifndef THRUST_I_GAIN
#define THRUST_I_GAIN -1
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
float nominal_throttle = 0.52; //GUIDANCE_V_NOMINAL_HOVER_THROTTLE;

/* ---
For hover controller:
--- */ 
float z_i = 0;
float start_time;

/* ---
Declare functions from this file
--- */
float timedifference_msec(struct timeval t_t0, struct timeval t_t1); 
void hovering_ctrl(float z_cmd);
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

float desired_X_vec[4] = {-2.5, 1.4, 1.4, -2.5};
float desired_Y_vec[4] = {2.7, 2.7, -1.8, -1.8};
float desired_Z_vec[4] = {1, 2, 1, 1};
float desired_psi_vec[4] = {PI/2, 0, -PI/2, -170*PI/180};

/*
// -- simulation 
float desired_X_vec[4] = {0, 6, 7, 1};
float desired_Y_vec[4] = {5, 6, 1, 0};
float desired_Z_vec[4] = {1, 2, 1, 1};
float desired_psi_vec[4] = {PI/2, 0, -PI/2, -178*PI/180};
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

// Mask to activate zero-end network:
bool zero_end_net = true;

// control inputs (from RC or NN): 
struct ctrl_struct ctrl;
float thrust_pct_before;

// variable for PID thrust control debugging: 
struct debug_PID debug_az_PID;

// define tolerances (later when you reach final position)
bool mask = true;
bool mask_last_waypoint = false;
float tol_x = 0.5;
float tol_y = 0.5;
float tol_z = 0.3;

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

	float az_nn_model = (control_nn[0] + BEBOP_MASS*GRAVITY_ACC)/BEBOP_MASS;
	float az_nn_drone = ctrl.thrust_pct*1.35*GRAVITY_ACC;
	float az_imu = -ACCEL_FLOAT_OF_BFP(imu.accel.z);

	pprz_msg_send_GCNET_MAIN(
		trans, dev, AC_ID, 
		&(pos_NWU_v2.x), &(pos_NWU_v2.y), &(pos_NWU_v2.z),
    &(vel_NWU_v2.x), &(vel_NWU_v2.y), &(vel_NWU_v2.z),
		&(att_euler_NWU_deg.phi), &(att_euler_NWU_deg.theta), &(att_euler_NWU_deg.psi), 
		&(rates->p), &(rates->q), &(rates->r),
		&(control_nn[1]), &(control_nn[2]), &(control_nn[3]),
		&(az_nn_model), &(ctrl.thrust_pct), &(az_imu),
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
Function: Hovering controller (altitude controller)
NOTE: Commanded position must be negative! 
*/
void hovering_ctrl(float z_cmd)
{
	// get current height (in NED)
  float z_measured = stateGetPositionNed_f()->z;
	// get current velocity (in NED)
  float vz_measured = stateGetSpeedNed_f()->z;
  
	// position error
  float zv_command = (KP_ALT *(z_cmd - z_measured));
  
	// bound the velocity command
  if(zv_command < -4)
	{
    zv_command = -4;
  }
  
  if(zv_command > 2.5)
	{
    zv_command = 2.5;
  }

	// velocity error 
	z_i += (zv_command - vz_measured)/PERIODIC_FREQUENCY;
  
	// thrust command
  float thrust_cmd = -KD_ALT*(zv_command - vz_measured) - z_i*KI_ALT + HOVERTHRUST/(cosf(stateGetNedToBodyEulers_f()->theta)/cosf(stateGetNedToBodyEulers_f()->phi));

  if(thrust_cmd > 0.8)
	{
    thrust_cmd = 0.8;
  }
  stabilization_cmd[COMMAND_THRUST] = thrust_cmd*9125.;

	struct FloatEulers cmd;
	struct FloatQuat cmd_quat;

	cmd.phi = ANGLE_BFP_OF_REAL(0.0);
	cmd.theta = ANGLE_BFP_OF_REAL(0.0);
	cmd.psi = ANGLE_BFP_OF_REAL(0.0);

	float_quat_of_eulers(&cmd_quat, &cmd);

	struct Int32Quat cmd_quat_int;

	QUAT_BFP_OF_REAL(cmd_quat_int,cmd_quat);

	stabilization_indi_attitude_run(cmd_quat_int, true);
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

	// idx_wp initialization:
	idx_wp = 0;

	// Initialize desired position and yaw angle: 
	desired_X = -2.5; // desired_X_vec[0];
	desired_Y = 2.5; // desired_Y_vec[0];
	desired_Z = 1; // desired_Z_vec[0];
	desired_psi = PI/2; // desired_psi_vec[0];
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
		ctrl.thrust_pct = thrust_pct_before*2*(0.65/0.5);

		// -- convert to integer
		int32_t thrust_int = ctrl.thrust_pct * MAX_PPRZ; // see how to transform this! 
	
		// -- bound it:
		Bound(thrust_int, 0, MAX_PPRZ);

		// -- send command
		stabilization_cmd[COMMAND_THRUST] = thrust_int;  

	}

	printf("%f \t %f \t %f \t %f \t %f \t %d \t %d \t %d \t %d (x,y,z,psi,nn_time,idx_wp, imu_x, imu_y, imu_z) \n", fabs(state_nn[0]), fabs(state_nn[1]), fabs(state_nn[2]), att_euler_NWU.psi*180/PI, nn_process_time, idx_wp, imu.accel.x, imu.accel.y, imu.accel.z);
	
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
		
		// if last waypoint is activated (considering that we have 4 waypoints) 
		if(idx_wp == 3)
		{  
			zero_end_net = true; 
			mask_last_waypoint = true;
			// idx_wp = 0; */
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
		/*}
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

	// transform from NED to Body: 
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
	// guidance_v_init();
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
		/* 
		// get the nominal of the drone to estimate the nominal thrust:  
  	if (first_run) {
    	start_time = get_sys_time_float();
    	nominal_throttle = (float)stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
    	first_run = false;
  	}
		*/ 
  	// Let the vehicle hover 
  	// if (get_sys_time_float() - start_time < 5.0f) 
		// {
		// hovering_ctrl(-1.0);
    	// return;
  	// }
		/*use neural network to guide drone */
		gcnet_guidance(in_flight);
		/* 
		printf("==============================================\n");		
		printf("%f \t %f \t %f (wp_x,wp_y,wp_z) \n",); 		
		printf("==============================================\n");		
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