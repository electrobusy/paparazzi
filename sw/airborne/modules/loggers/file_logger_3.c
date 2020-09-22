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
// #include "modules/gcnet/gcnet_main.h"

// other libraries for logging (lower-level controls and sensing)
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h" // -- INDI
#include "subsystems/actuators/motor_mixing.h" // -- Motor Mixing
#include "subsystems/imu.h" // -- IMU
#include "autopilot.h"

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
  
  /*
  // > Logs from body - In NED frame:
  */ 
  fprintf(file, "pos_x_NED,pos_y_NED,pos_z_NED,"); 
  fprintf(file, "vel_x_NED,vel_y_NED,vel_z_NED,"); 
  fprintf(file, "acc_x_NED,acc_y_NED,acc_z_NED,");
  fprintf(file, "att_phi_NED,att_theta_NED,att_psi_NED,");
  fprintf(file, "att_qi_NED,att_qx_NED,att_qy_NED,att_qz_NED,");
  fprintf(file, "rate_p_NED,rate_q_NED,rate_r_NED,");

  
  /* 
  // > Lower level control - INDI controller: 
  fprintf(file, "INDI_p_dot_ref,INDI_q_dot_ref,INDI_r_dot_ref,"); // [ADD MORE VARIABLES LATER]
  
  // > Lower level controls - Rotors Mixing Module: 
  // -- Motor Mixing commands:
  fprintf(file, "motor_cmd_1,motor_cmd_2,motor_cmd_3,motor_cmd_4,");
  // -- Motor Commands - Trim:
  fprintf(file, "motor_cmd_trim_1,motor_cmd_trim_2,motor_cmd_trim_3,motor_cmd_trim_4,");
  // -- Motor Commands - Thrust:
  fprintf(file, "motor_cmd_thrust_1,motor_cmd_thrust_2,motor_cmd_thrust_3,motor_cmd_thrust_4,");
  // -- Motor Commands - Roll:
  fprintf(file, "motor_cmd_roll_1,motor_cmd_roll_2,motor_cmd_roll_3,motor_cmd_roll_4,");
  // -- Motor Commands - Pitch:
  fprintf(file, "motor_cmd_pitch_1,motor_cmd_pitch_2,motor_cmd_pitch_3,motor_cmd_pitch_4,");
  // -- Motor Commands - Yaw:
  fprintf(file, "motor_cmd_yaw_1,motor_cmd_yaw_2,motor_cmd_yaw_3,motor_cmd_yaw_4,");
  */ 
  // > IMU accelerations: 
  fprintf(file, "IMU_ax,IMU_ay,IMU_az,");
  // [ADD MORE - IN THE END]

  // > Auto-pilot mode: 
  fprintf(file, "Auto-Pilot Mode,");
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
  
  fprintf(file, "%f,", get_sys_time_float());

  /* 
  // > Logs from body - In NED frame:
  */ 
  struct NedCoor_f *pos = stateGetPositionNed_f(); 
  struct NedCoor_f *vel = stateGetSpeedNed_f(); 
  struct NedCoor_f *acc = stateGetAccelNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatQuat *att_q = stateGetNedToBodyQuat_f();
  struct FloatRates *rates = stateGetBodyRates_f();   

  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", acc->x, acc->y, acc->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);  
  fprintf(file, "%f,%f,%f,%f,", att_q->qi, att_q->qx, att_q->qy, att_q->qz); 
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);  

  /*
  // > Lower level control - INDI controller: 
  fprintf(file, "%f,%f,%f,", indi.angular_accel_ref.p, indi.angular_accel_ref.q, indi.angular_accel_ref.r); // [ADD MORE VARIABLES LATER]
   
  // > Lower level controls - Rotors Mixing Module: 
  // -- Motor Mixing commands:
  fprintf(file, "%d,%d,%d,%d,", motor_mixing.commands[0], motor_mixing.commands[1], motor_mixing.commands[2], motor_mixing.commands[3]);
  // -- Motor Commands - Trim:
  fprintf(file, "%d,%d,%d,%d,", motor_mixing.trim[0], motor_mixing.trim[1], motor_mixing.trim[2], motor_mixing.trim[3]);
  // -- Motor Commands - Thrust:
  fprintf(file, "%d,%d,%d,%d,", motor_commands.thrust[0], motor_commands.thrust[1], motor_commands.thrust[2], motor_commands.thrust[3]);
  // -- Motor Commands - Roll:
  fprintf(file, "%d,%d,%d,%d,", motor_commands.roll[0], motor_commands.roll[1], motor_commands.roll[2], motor_commands.roll[3]);
  // -- Motor Commands - Pitch:
  fprintf(file, "%d,%d,%d,%d,", motor_commands.pitch[0], motor_commands.pitch[1], motor_commands.pitch[2], motor_commands.pitch[3]);
  // -- Motor Commands - Yaw:
  fprintf(file, "%d,%d,%d,%d,", motor_commands.yaw[0], motor_commands.yaw[1], motor_commands.yaw[2], motor_commands.yaw[3]);
  */
  // > IMU accelerations: 
  fprintf(file, "%f,%f,%f,", ACCEL_FLOAT_OF_BFP(imu.accel.x), ACCEL_FLOAT_OF_BFP(imu.accel.y), ACCEL_FLOAT_OF_BFP(imu.accel.z));
  // fprintf(file, "%d,%d,%d,", imu.accel.x, imu.accel.y, imu.accel.z);
  
  // > Auto-pilot mode: 
  fprintf(file, "%d,", autopilot.mode);
  // [ADD MORE - IN THE END]
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
    /*
    fprintf(file_logger, "Happy happy\n");
    sleep(1);
    */
    fflush(file_logger);
    sleep(1);
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