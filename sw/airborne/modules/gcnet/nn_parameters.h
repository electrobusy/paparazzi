#ifndef NN_PARAMETERS_H
#define NN_PARAMETERS_H

#define NUM_STATES 10
#define NUM_CONTROLS 4
#define NUM_LAYERS 4
#define NUM_HIDDEN_LAYERS 2
#define NUM_NODES 40

// network dataset information: 
// quadrotor3d_RW_Traj_20000_Nodes_80_alpha_0_5_PC3
// 2500

extern const int num_nodes_vec[NUM_LAYERS];

// NN network parameters -- define variables
extern const float weights_in[NUM_NODES][NUM_STATES];

extern const float bias_in[NUM_NODES];

extern const float weights_out[NUM_CONTROLS][NUM_NODES];

extern const float bias_out[NUM_CONTROLS];

extern const float in_norm_mean[NUM_STATES];

extern const float in_norm_std[NUM_STATES];

extern const float out_norm_mean[NUM_CONTROLS];

extern const float out_norm_std[NUM_CONTROLS];

extern const float out_scale_min[NUM_CONTROLS];

extern const float out_scale_max[NUM_CONTROLS];

extern const float weights_hid[NUM_HIDDEN_LAYERS-1][NUM_NODES][NUM_NODES];

extern const float bias_hid[NUM_HIDDEN_LAYERS-1][NUM_NODES];

#endif
