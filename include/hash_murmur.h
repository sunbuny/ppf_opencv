#pragma once


#include <cstdint>

namespace ppf_match_3d {


#if defined(_MSC_VER)

#define FORCE_INLINE    inline static

#include <stdlib.h>

#define ROTL32(x,y)     _rotl(x,y)
#define ROTL64(x,y)     _rotl64(x,y)

#else
//#define FORCE_INLINE __attribute__((always_inline))
#define FORCE_INLINE inline static

/* gcc recognises this code and generates a rotate instruction for CPUs with one */
#define ROTL32(x, r)  (((uint32_t)x << r) | ((uint32_t)x >> (32 - r)))

    inline static long long ROTL64(long long x, int8_t r) {
        return (x << r) | (x >> (64 - r));
    }

#endif // !defined(_MSC_VER)

#if (defined __x86_64__ || defined _M_X64)
typedef unsigned char uchar;
#include "hash_murmur64.h"

#define murmurHash hashMurmurx64
#else
#include "hash_murmur86.hpp"
#define murmurHash hashMurmurx86
#endif
}
