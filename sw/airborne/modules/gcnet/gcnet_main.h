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
 * @file "modules/gcnet/gcnet_main.h"
 * @author Rohan Chotalal
 * This module includes the implementation and interaction of the G&CNet with the
 * lower-level control loops.  Based in the "ctrl_module_outerloop_demo" files.
 */

#ifndef GCNET_MAIN_H
#define GCNET_MAIN_H

#include <time.h>
#include <stdbool.h> // get "true" and "false" identifiers

// paparazzi-based library
#include "state.h"
#include "math/pprz_algebra_float.h"

// user-made libraries with:
// -- functions for nn operations
#include "nn_operations.h"
// -- variables with nn parameters (weights, biases and other information about the nets)
#include "nn_parameters.h"

// declare math variables
#ifndef PI
#define PI 3.14159
#endif

// declare environment and drone parameters
#ifndef GRAVITY_ACC
#define GRAVITY_ACC 9.81
#endif

#ifndef BEBOP_MASS 
#define BEBOP_MASS 0.38905 
#endif

// neural network state and control/action: 
extern float state_nn[NUM_STATES];
extern float control_nn[NUM_CONTROLS];

// Set the both "guidance_h" and "guidance_v" modules using the "module" mode
// NOTE: Therefore, it automatically uses the guidance_*_module functions implemented here 
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own Horizontal loops: 
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

// Implement own Vertical loops - even though we don't use them 
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif