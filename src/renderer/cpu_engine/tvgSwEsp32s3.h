#ifndef _TVG_SW_ESP32S3_H_
#define _TVG_SW_ESP32S3_H_

#include <stdint.h>

static inline int32_t tvgSwAbsI32(int32_t x)
{
#ifdef THORVG_ESP32S3_VECTOR_SUPPORT
    int32_t result;
    __asm__ volatile("abs %0, %1" : "=r"(result) : "r"(x));
    return result;
#else
    return (x < 0) ? -x : x;
#endif
}


static inline int32_t tvgSwMinI32(int32_t a, int32_t b)
{
#ifdef THORVG_ESP32S3_VECTOR_SUPPORT
    int32_t result;
    __asm__ volatile("min %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
    return result;
#else
    return (a < b) ? a : b;
#endif
}


static inline int32_t tvgSwMaxI32(int32_t a, int32_t b)
{
#ifdef THORVG_ESP32S3_VECTOR_SUPPORT
    int32_t result;
    __asm__ volatile("max %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
    return result;
#else
    return (a > b) ? a : b;
#endif
}


static inline uint32_t tvgSwMinU32(uint32_t a, uint32_t b)
{
#ifdef THORVG_ESP32S3_VECTOR_SUPPORT
    uint32_t result;
    __asm__ volatile("minu %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
    return result;
#else
    return (a < b) ? a : b;
#endif
}


static inline uint32_t tvgSwMaxU32(uint32_t a, uint32_t b)
{
#ifdef THORVG_ESP32S3_VECTOR_SUPPORT
    uint32_t result;
    __asm__ volatile("maxu %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
    return result;
#else
    return (a > b) ? a : b;
#endif
}


static inline uint32_t tvgSwMulHiU32(uint32_t a, uint32_t b)
{
#ifdef THORVG_ESP32S3_VECTOR_SUPPORT
    uint32_t result;
    __asm__ volatile("muluh %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
    return result;
#else
    return static_cast<uint32_t>((uint64_t(a) * uint64_t(b)) >> 32);
#endif
}


static inline int32_t tvgSwRecipI32(int32_t d)
{
    return (d != 0) ? static_cast<int32_t>(int64_t(0xffffffffu) / d) : 0;
}


static inline int32_t tvgSwCoverageAbsClampI32(int32_t coverage)
{
    coverage = tvgSwAbsI32(coverage);
    return tvgSwMinI32(coverage, 255);
}


static inline uint32_t tvgSwAbsDiffU32(int32_t a, int32_t b)
{
    auto diff = int64_t(a) - int64_t(b);
    return static_cast<uint32_t>(diff < 0 ? -diff : diff);
}

#endif
