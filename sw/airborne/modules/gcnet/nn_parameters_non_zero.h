#ifndef NN_PARAMETERS_NON_ZERO_H
#define NN_PARAMETERS_NON_ZERO_H

#define NUM_STATES 10
#define NUM_CONTROLS 4
#define NUM_LAYERS 4
#define NUM_HIDDEN_LAYERS 2
#define NUM_NODES 40

// network dataset information: 
// quadrotor3d_RW_Traj_20000_Nodes_80_alpha_0_9_3ms-1_PC5
// 2555

// extern const int num_nodes_vec[NUM_LAYERS];

// NN network parameters -- define variables
extern const float weights_in_nz[NUM_NODES][NUM_STATES];

extern const float bias_in_nz[NUM_NODES];

extern const float weights_out_nz[NUM_CONTROLS][NUM_NODES];

extern const float bias_out_nz[NUM_CONTROLS];

extern const float in_norm_mean_nz[NUM_STATES];

extern const float in_norm_std_nz[NUM_STATES];

extern const float out_norm_mean_nz[NUM_CONTROLS];

extern const float out_norm_std_nz[NUM_CONTROLS];

extern const float out_scale_min_nz[NUM_CONTROLS];

extern const float out_scale_max_nz[NUM_CONTROLS];

extern const float weights_hid_nz[NUM_HIDDEN_LAYERS-1][NUM_NODES][NUM_NODES];

extern const float bias_hid_nz[NUM_HIDDEN_LAYERS-1][NUM_NODES];

#endif
