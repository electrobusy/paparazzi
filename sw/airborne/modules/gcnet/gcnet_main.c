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

#include "gcnet_main.h"

/*
Function: Compute the time difference
*/
float timedifference_msec(struct timeval t_t0, struct timeval t_t1)
{
    return (t_t1.tv_sec - t_t0.tv_sec) * 1000.0f + (t_t1.tv_usec - t_t0.tv_usec) / 1000.0f;
}

bool gcnet_control()
{
    // 
    
    
    // -- calculate rotational matrices (in euler angles)
    float_rmat_of_eulers_321(&R_OT_2_NED, &att_euler_OT2NED); 
    float_rmat_of_eulers_321(&R_NED_2_NWU, &att_euler_NED2BWU);

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

    // -- get coordinates in the network's reference frame (where the x-axis 
    // points to the waypoint and the y axis is perpendicular to this):
    // NOTE: 2-D coordinate transformation
    // see again the value of psi_ref
    delta_pos_net.x = cosf(psi_ref)*delta_pos_NWU.x + sinf(psi_ref)*delta_pos_NWU.y;
    delta_pos_net.y = -sinf(psi_ref)*delta_pos_NWU.x + cosf(psi_ref)*delta_pos_NWU.y;

    // -- get horizontal velocity components aligned with the network's reference frame
    vel_net.x = cosf(psi_ref)*vel_NWU.x + sinf(psi_ref)*vel_NWU.y;
    vel_net.y = -sinf(psi_ref)*vel_NWU.x + cosf(psi_ref)*vel_NWU.y;

    // -- transform Euler to quaternions - function is already coded on "pprz_algebra_float.c" file
    float_quat_of_eulers(&att_quat, &att_euler_NED2BWU)

    // -- assign states to the state vector that is fed to the network: 
    state_nn[0] = delta_pos_net.x;
    state_nn[1] = delta_pos_net.y; 
    state_nn[2] = z - desired_pos.z;
    state_nn[3] = vel_net.x;
    state_nn[4] = vel_net.y;
    state_nn[5] = vel_NWU.z; 
    state_nn[6] = att_quat.qi;
    state_nn[7] = att_quat.qx;
    state_nn[8] = att_quat.qy; 
    state_nn[9] = att_quat.qz; 

    // NOTE: variable "control_nn" was declared in gcnet_main.h  

    // -- feed state to the network 
    gettimeofday(&t0, 0);
    nn_control(state_nn, control_nn);
    gettimeofday(&t1, 0);
    nn_process_time = timedifference_msec(t0, t1);

    // -- Since thrust is within the interval [-m*g, m*g], we have to sum m*g
    control_nn[0] = control_nn[0] + BEBOP_MASS*GRAVITY_ACC; // to stay between 0 and 2*m*g 

    // -- inner loop controls 
    // [ADD THIS!]

}