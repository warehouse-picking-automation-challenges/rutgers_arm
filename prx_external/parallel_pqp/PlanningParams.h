#ifndef PLANNING_PARAMS_H
#define PLANNING_PARAMS_H

const int RANDOM_SEED = 1131;

// Precomputation constants
const float BASE_RADIUS = 30.f;

const int MAX_LEVEL = 12;
const unsigned int MAX_PRECOMPUTATION_LEVEL = 2;

const double POISSON_DISK_SAMPLING_CONSTANT_A = 1.0;
const int POISSON_DISK_SAMPLING_CONSTANT_B = 2;

// Runtime planning constants
const int NUM_THREADS = 32;
const unsigned int MAX_NUM_NEAREST_NEIGHBOR = 8;

#endif