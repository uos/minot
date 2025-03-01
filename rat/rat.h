#ifndef RAT_H
#define RAT_H

#include <stddef.h>

// Call once at the beginning of the program to connect with the network.
int rat_init(const char *node_name);

// Call once at the end of the program to disconnect from the network.
int rat_deinit();

// Handle the variable by syncing it with the network.
// The datatype is a dynamic 2D column-major array (like Eigen).
int rat_bacon_f32(const char *variable_name, float *data, size_t rows, size_t cols);
int rat_bacon_f64(const char *variable_name, double *data, size_t rows, size_t cols);
int rat_bacon_i32(const char *variable_name, int *data, size_t rows, size_t cols);
int rat_bacon_u8(const char *variable_name, uchar *data, size_t rows, size_t cols);

#endif
