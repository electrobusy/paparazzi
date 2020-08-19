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

// Horizontal guidance module functions (inside paparazzi)
/*
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);
*/ 

// Vertical guidance module functions (inside paparazzi)
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif