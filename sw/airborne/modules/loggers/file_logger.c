/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "std.h"

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

// Add neural network library 
#include "modules/gcnet/gcnet_main.h"


/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in file_logger_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void file_logger_write_header(FILE *file) {
  fprintf(file, "time,");
  // > Logs from body - In NED frame:
  fprintf(file, "pos_x_NED,pos_y_NED,pos_z_NED,");
  fprintf(file, "vel_x_NED,vel_y_NED,vel_z_NED,");
  fprintf(file, "acc_x_NED,acc_y_NED,acc_z_NED,");
  fprintf(file, "att_phi_NED,att_theta_NED,att_psi_NED,");
  fprintf(file, "att_qi_NED,att_qx_NED,att_qy_NED,att_qz_NED,");
  fprintf(file, "rate_p_NED,rate_q_NED,rate_r_NED,");
  // > Neural Network file - Logs: [MY FILE]
  // -- post-processing time: 
  fprintf(file, "NN_pp_time,");
  // -- body state variables:
  fprintf(file, "pos_x_ENU,pos_y_ENU,pos_z_ENU,");
  fprintf(file, "vel_x_ENU,vel_y_ENU,vel_z_ENU,");
  fprintf(file, "att_phi_ENU,att_theta_ENU,att_psi_ENU,");
  fprintf(file, "att_qi_ENU,att_qx_ENU,att_qy_ENU,att_qz_ENU,");
  fprintf(file, "rate_p_ENU,rate_q_ENU,rate_r_ENU,");
  // -- desired position and yaw angle: 
  fprintf(file, "des_pos_x_ENU,des_pos_y_ENU,des_pos_z_ENU,des_pos_psi_ENU,");
  // -- delta position - ENU and Network (intermediate) frames
  fprintf(file, "d_pos_x_ENU,d_pos_y_ENU,d_pos_x_net,d_pos_y_net,");
  // -- velocity and yaw angle in Network frame
  fprintf(file, "v_x_net,v_y_net,yaw_net,");
  // -- Neural Network - inputs (states):
  fprintf(file, "x_nn,y_nn,z_nn,v_x_nn,v_y_nn,v_z_nn,qi_nn,qx_nn,qy_nn,qz_nn,");
  // -- Neural Network - outputs (controls):
  fprintf(file, "T_nn,");
  fprintf(file, "p_nn,q_nn,r_nn,");
  // -- Thrust percentage: 
  fprintf(file, "T_pct,"); // before conversion
  fprintf(file, "T_pct_conv,"); // after conversion
  // > Attitude commands - from rate integration:
  fprintf(file, "qi_cmd,qx_cmd,qy_cmd,qz_cmd,");
  fprintf(file, "phi_cmd,theta_cmd,psi_cmd,");
  // > INDI controller: 
  // fill this in! 
  // > Lower level controls - Rotors: 
  // -- Motor Mixing commands:
  fprintf(file, "motor_cmd_1,motor_cmd_2,motor_cmd_3,motor_cmd_4,");
  // -- Motor Commands - Trim:
  fprintf(file, "motor_cmd_trim_1,motor_cmd_trim_2,motor_cmd_trim_3,motor_cmd_trim_4,");
  // -- Motor Commands - Thrust:
  fprintf(file, "motor_cmd_thrust_1,motor_cmd_thrust_2,motor_cmd_thrust_3,motor_cmd_thrust_4,");
  // -- Motor Commands - Pitch:
  fprintf(file, "motor_cmd_pitch_1,motor_cmd_pitch_2,motor_cmd_pitch_3,motor_cmd_pitch_4,");
  // -- Motor Commands - Roll:
  fprintf(file, "motor_cmd_roll_1,motor_cmd_roll_2,motor_cmd_roll_3,motor_cmd_roll_4,");
  // > IMU accelerations: 
  fprintf(file, "IMU_ax,IMU_ax,IMU_az,");
  // [ADD MORE - IN THE END]
#ifdef COMMAND_THRUST
  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
#else
  fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of file_logger_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void file_logger_write_row(FILE *file) {
  struct EnuCoor_f *pos = stateGetPositionEnu_f(); 
  struct EnuCoor_f *vel = stateGetSpeedEnu_f(); 
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", att->phi, -att->theta, -att->psi); 
  fprintf(file, "%f,%f,%f,", rates->p, -rates->q, -rates->r); 
  fprintf(file, "%f,%f,%f,", control_nn[1], control_nn[2], control_nn[3]);
  fprintf(file, "%f,", control_nn[0]);
#ifdef COMMAND_THRUST
  fprintf(file, "%d,%d,%d,%d\n",
      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif
}


/** Start the file logger and open a new file */
void file_logger_start(void)
{
  // Create output folder if necessary
  if (access(STRINGIFY(FILE_LOGGER_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(FILE_LOGGER_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[file_logger] Could not create log file directory %s.\n", STRINGIFY(FILE_LOGGER_PATH));
      return;
    }
  }

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), date_time);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), date_time, counter);
    counter++;
  }

  file_logger = fopen(filename, "w");
  if(!file_logger) {
    printf("[file_logger] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[file_logger] Start logging to %s...\n", filename);

  file_logger_write_header(file_logger);
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file    */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  file_logger_write_row(file_logger);
}
