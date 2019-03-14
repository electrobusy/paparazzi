/*
 * Copyright (C) Shuo Li
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
 * @file "modules/guidance_loop_controller/guidance_loop_controller.c"
 * @author Shuo Li
 * This module includes all controllers that calculate low level command to attitude loop or even lower level loop.
 */

#include "modules/guidance_loop_controller/guidance_loop_controller.h"
#include "modules/guidance_h_module/guidance_h_module.h"
#include "modules/nn/nn.h"
#include "modules/nn/nn_params.h"
#include "stdio.h"
#include "state.h"
#include<sys/time.h>
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
//#include "modules/nn/nn_params.h"


enum ControllerInUse controllerInUse;
struct Pos hoverPos;
struct Euler hoverEuler;
bool flagNN;
struct NN_CMD nn_cmd;
struct NN_STATE nn_state;
struct ADAPT_HOVER_COEFFCIENT hover_coefficient;
float scale_factor;

struct FloatRMat R_OPTITRACK_2_NED, R_NED_2_NWU;
struct FloatEulers eulersOT2NED = {0.0,0.0,-0.0/180.0*3.14};
struct FloatEulers eulersNED2BWU = {3.14,0.0,0.0};
struct FloatVect3 pos_OT,vel_OT,pos_NED,vel_NED,pos_NWU,vel_NWU;
float nn_time;
float temp_time;
float psi_c;
struct timeval t0;
struct timeval t1;
struct timeval NN_start;
float nn_x_sp,nn_z_sp;


bool hover_with_optitrack(float hoverTime)
{
    if(controllerInUse!= CONTROLLER_HOVER_WITH_OPTITRACK)
    {
            controllerInUse= CONTROLLER_HOVER_WITH_OPTITRACK;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            hoverPos.x = stateGetPositionNed_f()->x;
            hoverPos.y= stateGetPositionNed_f()->y;
            hoverPos.z = stateGetPositionNed_f()->z;
            hoverEuler.psi= 0;
            printf("hover is initialized\n");
            printf("time 2 is %f\n",getTime(2));
            flagNN = false;
    }

    // -------------for log -----------------------

    float_rmat_of_eulers_321(&R_OPTITRACK_2_NED,&eulersOT2NED);
    float_rmat_of_eulers_321(&R_NED_2_NWU,&eulersNED2BWU);
    pos_OT.x = stateGetPositionNed_f()->x;
    pos_OT.y = stateGetPositionNed_f()->y;
    pos_OT.z = stateGetPositionNed_f()->z;
    vel_OT.x = stateGetSpeedNed_f()->x;
    vel_OT.y = stateGetSpeedNed_f()->y;
    vel_OT.z = stateGetSpeedNed_f()->z;
    
   float_rmat_transp_vmult(&pos_NED, &R_OPTITRACK_2_NED, &pos_OT);
   float_rmat_transp_vmult(&vel_NED, &R_OPTITRACK_2_NED, &vel_OT);

   float_rmat_transp_vmult(&pos_NWU, &R_NED_2_NWU, &pos_NED);
   float_rmat_transp_vmult(&vel_NWU, &R_NED_2_NWU, &vel_NED);
   // ----------------------------------------------
   
   guidance_h_set_guided_pos(0.0, 0.0); 
   //guidance_v_set_guided_z(-1.5);
   set_z_ref(-1.5);
   guidance_h_set_guided_heading(0.0);

    if(getTime(2)>hoverTime)
    {
        return true;
    }
    else
    {
        return false;
    }

}

float timedifference_msec(struct timeval t0, struct timeval t1)
{
	return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}

void nn_controller(float desired_x,float desired_z)
{
    if(controllerInUse!= CONTROLLER_NN_CONTROLLER)
    {
            controllerInUse = CONTROLLER_NN_CONTROLLER;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            flagNN = true;
            printf("[nn controle] nn controller is activated]");

	    float_rmat_of_eulers_321(&R_OPTITRACK_2_NED,&eulersOT2NED);
	    float_rmat_of_eulers_321(&R_NED_2_NWU,&eulersNED2BWU);

	    hoverPos.x = stateGetPositionNed_f()->x;
	    hoverPos.y = stateGetPositionNed_f()->y;
	    hoverPos.z = stateGetPositionNed_f()->z;

	    guidance_h_set_guided_heading(0);
	    guidance_v_set_guided_z(desired_z);
	    gettimeofday(&NN_start, 0);
    }

    nn_x_sp = desired_x;
    nn_z_sp = desired_z;

   guidance_h_set_guided_pos(desired_x,0.0); 
   guidance_h_set_guided_heading(0.0);
    //set_z_ref(desired_z);
    int time_int;
    gettimeofday(&t1, 0);
    temp_time = timedifference_msec(NN_start,t1);
    time_int = temp_time/1000;
    int temp = time_int/10%10;
	    //guidance_v_set_guided_z(-0.5-0.5*(temp));

    // transform coordinate from Optitrack frame to NED frame of cyberzoo and then to North-west-up frame

    pos_OT.x = stateGetPositionNed_f()->x;
    pos_OT.y = stateGetPositionNed_f()->y;
    pos_OT.z = stateGetPositionNed_f()->z;
    vel_OT.x = stateGetSpeedNed_f()->x;
    vel_OT.y = stateGetSpeedNed_f()->y;
    vel_OT.z = stateGetSpeedNed_f()->z;
    
    float_rmat_transp_vmult(&pos_NED, &R_OPTITRACK_2_NED, &pos_OT);
    float_rmat_transp_vmult(&vel_NED, &R_OPTITRACK_2_NED, &vel_OT);

    float_rmat_transp_vmult(&pos_NWU, &R_NED_2_NWU, &pos_NED);
    float_rmat_transp_vmult(&vel_NWU, &R_NED_2_NWU, &vel_NED);

   // prepare current states to feed NN
    float state[NUM_STATE_VARS] = {pos_NWU.x-desired_x, vel_NWU.x, pos_NWU.z+desired_z, vel_NWU.z, -stateGetNedToBodyEulers_f()->theta};
    float control[NUM_CONTROL_VARS];
    gettimeofday(&t0, 0);
    //nn_stable(state, control);
    nn(state, control);
//   nested_control(state, control) ;
    gettimeofday(&t1, 0);
    nn_time = timedifference_msec(t0,t1);

    //nn_cmd.thrust_ref = control[0] ;
    //nn_cmd.rate_ref = -control[1] ;
	float F_min = 1.76;
	float F_max = 2.35;
	
	nn_cmd.FL = F_min+(F_max-F_min)*control[0];
	nn_cmd.FR = F_min+(F_max-F_min)*control[1];
}

bool go_to_point(float desired_x,float desired_y,float desired_z,float desired_heading)
{
    if(controllerInUse != CONTROLLER_GO_TO_POINT)
    {
            controllerInUse = CONTROLLER_GO_TO_POINT ;
            clearClock(2);
            guidance_h_mode_changed(GUIDANCE_H_MODE_GUIDED);
            guidance_v_mode_changed(GUIDANCE_V_MODE_GUIDED);
            flagNN = false;
    }
    guidance_h_set_guided_pos(desired_x, desired_y);
    guidance_v_set_guided_z(desired_z);
    guidance_h_set_guided_heading(desired_heading);

    float error_x =fabs(stateGetPositionNed_f()->x - desired_x);
    float error_y =fabs(stateGetPositionNed_f()->y - desired_y);
    float error_z =fabs(stateGetPositionNed_f()->z - desired_z);
    //set_z_ref(desired_z);
    if(error_x+error_y+error_z < 0.2)
    {
        return true;
    }
    else
    {
        return false;
    }

}

