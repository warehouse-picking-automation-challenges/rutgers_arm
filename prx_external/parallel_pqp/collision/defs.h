#ifndef __DEFS_H_
#define __DEFS_H_

#include <float.h>

//#define GPLANNER_DEBUG

#define PROXIMITY_THREADS 128

#define TWOPOWER32 (1 << 32)

#define BUCKET_ITEMS 512
#define CUCKOO_TABLE_NUM 3
#define CUCKOO_BLOCK_SIZE 512
#define CUCKOO_MULTIHASH_PER_BLOCK_SIZE 576
#define CUCKOO_HASH_PER_BLOCK_SIZE (CUCKOO_MULTIHASH_PER_BLOCK_SIZE / CUCKOO_TABLE_NUM)
#define CUCKOO_MAX_ATTEMPT 25
#define CUCKOO_HASH_PARAMETER 409
#define CUCKOO_HASH_THREADS 128

#define LSH_M 32
#define LSH_nP 32
#define LSH_MAXSORT 800

#define MAX_BLOCKS_PER_DIMENSION 65535

#ifndef CUDART_NORM_HUGE_F
#define CUDART_NORM_HUGE_F FLT_MAX
#endif

#define INVALIDID ((unsigned int)-1)
#define NOKNNRESULT ((unsigned int)-1)
#define SEARCHGRAPH_NOPARENT_NODE ((unsigned int)-1)
#define NODEPTH ((unsigned int)-1)
#define NOCOLLISIONFREEPATH ((unsigned int)-1)
#define ROBOT_NOPARENT_NODE ((short)-1)
#define ROBOT_NOPARAM_NODE ((short)-1)

#endif