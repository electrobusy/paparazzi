/*
File: nn_operations.h
Goal: Include header of different functions used to compute neural network operations
*/

#ifndef NN_OPERATIONS_H
#define NN_OPERATIONS_H

#include <stdio.h>
#include <math.h> // use tanh and softplus operations 

// #include "nn_parameters.h" // add nn properties and weights

void preprocess_input(float* input);

void postprocess_output(float* output);

void tanh_activation(float* vec, int numNodes);

void softplus_activation(float* vec, int numNodes);

void copy_vec(float* vec_1, float* vec_2, int len_vec_1);

void fully_connected_layer(float* in, float* out, const float* weight_ptr, const float* bias_ptr, int dim_curr_layer, int dim_next_layer);

void nn_predict(float* nn_input, float* nn_output);

void nn_control(float* state, float* control);

#endif
