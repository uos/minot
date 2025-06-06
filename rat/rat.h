#ifndef RAT_H
#define RAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#ifdef __arm__
typedef char ratbaconbyte;
#else
typedef unsigned char ratbaconbyte;
#endif

// Call once at the beginning of the program to connect with the network.
// negative or zero mute_timeout will disable all communications of the rat.
int rat_init(const char *node_name, int timeout_secs);

// Call once at the end of the program to disconnect from the network.
int rat_deinit();

// Handle the variable by syncing it with the network.
// The datatype is a dynamic 2D column-major array (like Eigen).
int rat_bacon_f32(const char *variable_name, float *data, size_t rows, size_t cols);
int rat_bacon_f64(const char *variable_name, double *data, size_t rows, size_t cols);
int rat_bacon_i32(const char *variable_name, int *data, size_t rows, size_t cols);
int rat_bacon_u8(const char *variable_name, ratbaconbyte *data, size_t rows, size_t cols);

#ifdef __cplusplus
}
#endif

#endif
