#ifndef CUDA_TIMING_H
#define CUDA_TIMING_H

//#define CUDA_TIMING
#ifdef CUDA_TIMING
#define CUDA_TIMING_BEGIN() static float timeSum = 0.f; static int timeCount = 0; cudaEvent_t start, stop; float time; cudaEventCreate(&start); cudaEventCreate(&stop); cudaEventRecord( start, 0 );
#define CUDA_TIMING_END(NAME) cudaEventRecord( stop, 0 ); cudaEventSynchronize( stop ); cudaEventElapsedTime( &time, start, stop );\
	cudaEventDestroy( start ); cudaEventDestroy( stop );\
	timeSum += time; ++timeCount; printf(NAME); printf(" : %f ms (Sum: %f Avg:%f)\n", time, timeSum, timeSum / timeCount);
#else
#define CUDA_TIMING_BEGIN()
#define CUDA_TIMING_END(NAME)
#endif

#endif