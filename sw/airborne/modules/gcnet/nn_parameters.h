#ifndef NN_PARAMETERS_H
#define NN_PARAMETERS_H

#define NUM_STATES 10
#define NUM_CONTROLS 4
#define NUM_LAYERS 4
#define NUM_HIDDEN_LAYERS 2
#define NUM_NODES 40

const int num_nodes_vec[NUM_LAYERS];

// NN network parameters -- define variables
const float weights_in[NUM_NODES][NUM_STATES];

const float bias_in[NUM_NODES];

const float weights_out[NUM_CONTROLS][NUM_NODES];

const float bias_out[NUM_CONTROLS];

const float in_norm_mean[NUM_STATES];

const float in_norm_std[NUM_STATES];

const float out_norm_mean[NUM_CONTROLS];

const float out_norm_std[NUM_CONTROLS];

const float out_scale_min[NUM_CONTROLS];

const float out_scale_max[NUM_CONTROLS];

const float weights_hidden[NUM_HIDDEN_LAYERS-1][NUM_NODES][NUM_NODES];

const float bias_hidden[NUM_HIDDEN_LAYERS-1][NUM_NODES];

#endif
