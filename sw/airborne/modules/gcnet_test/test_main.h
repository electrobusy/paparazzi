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
 * @file "modules/gcnet_test/test_main.h"
 * @author Rohan Chotalal
 * This module includes the implementation and interaction of the G&CNet with the
 * lower-level control loops.  
 */

#ifndef TEST_MAIN_H
#define TEST_MAIN_H

#include <time.h>
#include <stdbool.h> // get "true" and "false" identifiers

// paparazzi-based library
#include "state.h"
#include "math/pprz_algebra_float.h"

// user-made libraries
// #include "nn_operations.h"

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

#ifndef BEBOP_DRAG_X
#define BEBOP_DRAG_X -0.5
#endif

#ifndef BEBOP_DRAG_Y
#define BEBOP_DRAG_Y -0.5
#endif

// For the dataset generation to train G&CNet, we considered beta_z = 0
#ifndef BEBOP_DRAG_Z
#define BEBOP_DRAG_Z 0
#endif

// Module functions (for paparazzi)
extern void gcnet_init(void);
extern void gcnet_run(void);

#endif