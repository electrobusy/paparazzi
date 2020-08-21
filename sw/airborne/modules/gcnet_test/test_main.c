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

#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE
// #define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_NAV

#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE
//#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_NAV

// Function definition: 
void hovering_quad_run(bool in_flight);

/*
Function: Hovering quadrotor (in this case, using information from optitrack) -- initialize
*/
void hovering_quad_init(void) // (float hover_time)
{
    // check if enters in this module
    printf("Hover is initialized! \n");
    printf("We are inside the module: let us start hovering! \n");
}

/**
 * hovering_quad_run
 * @param[in] in_flight - Whether we are in flight or not
 */
void hovering_quad_run(bool in_flight)
{
    if (in_flight)
    {
        guidance_h_set_guided_pos(0.0, 0.0); // position (x,y) = (0,0)
        guidance_h_set_guided_heading(0.0); // psi = 0
    }
}

/** -----
 * HORIZONTAL GUIDANCE MODE 
 ----- */

/**
 * Initialization of horizontal guidance module.
 */
void guidance_h_module_init(void)
{
    hovering_quad_init();
}

/**
 * Horizontal guidance mode entered
 */
void guidance_h_module_enter(void) 
{
    printf("Entered module!\n");
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void) {}

/**
 * Main guidance loop
 * @param[in] in_flight - Whether we are in flight or not
 */
void guidance_h_module_run(bool in_flight)
{
    hovering_quad_run(in_flight);
}

/** -----
 * VERTICAL GUIDANCE MODE 
 ----- */

 /**
 * Initialization of vertical guidance module.
 */
 void guidance_v_module_init(void)
{
    hovering_quad_init();
}

/**
 * Vertical guidance mode entered
 */
void guidance_v_module_enter(void) 
{
    printf("Entered module!\n");
}

/**
 * Function: guidance_v_module_run
 * Description: Run vertical guidance module
 * @param[in] in_flight - Whether we are in flight or not
 */
void guidance_v_module_run(bool in_flight)
{
    if (in_flight)
    {
        guidance_v_set_guided_z(-5); // position z = 5 m
    }
}

// ---------- 
// FOR LATER
/* 
void acceleration_z_controller(float desired_az)
{
    // define integration error: 
	static float integrator_error = 0.f;
	static float previous_error = 0.f;

    // get acceleration: 
	struct NedCoor_f *accel = stateGetAccelNed_f();
	
    // filter acceleration using a butterworth filter (since this is too noisy): 
    update_butterworth_2_low_pass(&accel_ned_filt, accel->z);
	
    // remove gravity from the filtered acceleration (since IMU): 
    float filtered_az = (accel_ned_filt.o[0])/cosf(stateGetNedToBodyEulers_f()->theta)/cosf(stateGetNedToBodyEulers_f()->phi);
	
    // get the acceleration error:
    float error_az =  desired_az - filtered_az;

    // cumulative integration error: 
	error_integrator += error_az / 100.0;

	float thrust_sp = (error_az*thrust_p_gain + error_integrator*thrust_i_gain + thrust_d_gain*(error_az-previous_error)/100.0)*thrust_effectiveness + nominal_throttle;
	
    // set the desired vertical thrust in the "vertical guidance module":
    guidance_v_set_guided_th(thrust_sp);

    // set the value of the previous error: 
	previous_error = error_az;
}
*/