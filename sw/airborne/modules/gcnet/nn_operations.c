#include "nn_operations.h"

/*
Function: Pre-process inputs (normalization layer)
*/
void preprocess_input(float *input, float *input_norm, const float *input_norm_mean, const float *input_norm_std)
{   
    int idx;

    for(idx = 0; idx < NUM_STATES; idx++)
    {
        // normalize the states   
        input_norm[idx] = (input[idx] - input_norm_mean[idx])/input_norm_std[idx];
    }
}

/*
Function: Post-process outputs (unscaling and unormalization layers)
*/
void postprocess_output(float *output, const float *output_scale_min, const float *output_scale_max, const float *output_norm_mean, const float *output_norm_std)
{
    int idx;
    float slope;
    float out_unscaled;

    for(idx = 0; idx < NUM_CONTROLS; idx++)
    {
        // unscale from [-1,1] to the normalized interval [control_min, control_max]
        slope = (output_scale_max[idx] - output_scale_min[idx])/(1 - (-1));
        out_unscaled = slope*(output[idx] - (-1)) + output_scale_min[idx];

        // unormalize from [control_min, control_max] to the real interval [u_min, u_max]
        output[idx] = out_unscaled*output_norm_std[idx] + output_norm_mean[idx];
    }
}

/*
Function: Tanh layer operation
*/
void tanh_activation(float *vec, int numNodes)
{
    int idx_nodes;

    for(idx_nodes = 0; idx_nodes < numNodes; idx_nodes++)
    {
        vec[idx_nodes] = tanh(vec[idx_nodes]);
    }
}

/*
Function: Softplus layer operation
*/
void softplus_activation(float *vec, int numNodes)
{
    int idx_nodes;

    for(idx_nodes = 0; idx_nodes < numNodes; idx_nodes++)
    {
        vec[idx_nodes] = log(1 + exp(vec[idx_nodes]));
    }
}

/*
Function: Operations in a fully connected layer
*/
void fully_connected_layer(float *in, float *out, const float *weight_ptr, const float *bias_ptr, int dim_curr_layer, int dim_next_layer)
{
    // index i - rows of the weight matrix 
    // index j - columns of the weight matrix 
    int i, j; 

    float sum; 
    
    for(i = 0; i < dim_next_layer; i++)
    {
        sum = 0; 

        for(j = 0; j < dim_curr_layer; j++)
        {
            sum += *(weight_ptr + i*dim_curr_layer + j)*in[j];
        }

        out[i] = sum + *(bias_ptr + i);
    }
}

/*
Function: Copy vector
*/ 
void copy_vec(float *vec_1, float *vec_2, int len_vec_1)
{
    for(int i = 0; i < len_vec_1; i++)
    {
        vec_2[i] = vec_1[i];
    }
}

/*
Function: Neural network prediction
*/
void nn_predict(float *nn_input, float *nn_output, const float (*weights_input)[NUM_STATES], const float *bias_input, const float weights_hidden[NUM_HIDDEN_LAYERS-1][NUM_NODES][NUM_NODES], const float (*bias_hidden)[NUM_NODES], const float (*weights_output)[NUM_NODES], const float *bias_output)
{   
    int i; 

    const float* ptr_weights = NULL;
    const float* ptr_bias = NULL;

    // fully connected layer
    ptr_weights = &weights_input[0][0];
    ptr_bias = &bias_input[0];

    float aux_vec[NUM_NODES];
    float aux_vec_hidden[NUM_NODES];
    
    fully_connected_layer(nn_input, aux_vec, ptr_weights, ptr_bias, num_nodes_vec[0], num_nodes_vec[1]);

    // softplus activation 
    softplus_activation(aux_vec, num_nodes_vec[1]);

    for (i = 1; i < NUM_HIDDEN_LAYERS; i++)
    {
        ptr_weights = &weights_hidden[i-1][0][0];
        ptr_bias = &bias_hidden[i-1][0];

        copy_vec(aux_vec, aux_vec_hidden, num_nodes_vec[i]); // copy vector aux

        // fully connected layer
        fully_connected_layer(aux_vec_hidden, aux_vec, ptr_weights, ptr_bias, num_nodes_vec[i], num_nodes_vec[i+1]);

        // softplus activation 
        softplus_activation(aux_vec, num_nodes_vec[i+1]);

    }

    // fully connected layer
    ptr_weights = &weights_output[0][0];
    ptr_bias = &bias_output[0];
    fully_connected_layer(aux_vec, nn_output, ptr_weights, ptr_bias, num_nodes_vec[i], num_nodes_vec[i+1]);
    
    // tanh activation 
    tanh_activation(nn_output, NUM_CONTROLS);
    
}

/*
Function: Neural network control
*/
void nn_control(float* state, float* control)
{
    float state_norm[NUM_STATES];

    // 1 - pre-processing input
    preprocess_input(state, state_norm, in_norm_mean, in_norm_std);

    // 2 - neural network prediction
    nn_predict(state_norm, control, weights_in, bias_in, weights_hid, bias_hid, weights_out, bias_out);
    
    // 3 - post-processing output
    postprocess_output(control, out_scale_min, out_scale_max, out_norm_mean, out_norm_std);
}