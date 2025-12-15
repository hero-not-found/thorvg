/*
 * Copyright (c) 2021 - 2025 the ThorVG project. All rights reserved.
 * ESP32 optimizations added for hero-not-found project.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifdef THORVG_ESP32_VECTOR_SUPPORT

/*
 * ESP32-S3 Optimization Notes:
 * - ESP32-S3 has PIE (Processor Instruction Extensions) SIMD instructions
 * - Key instructions used:
 *   - ee.vldbc.32: Broadcast load 32-bit value to all 4 lanes of 128-bit register
 *   - ee.vldbc.8: Broadcast load 8-bit value to all 16 lanes
 *   - ee.vst.128.ip: Store 128 bits with post-increment
 *   - loopnez: Zero-overhead loop
 * - Optimizations:
 *   1. SIMD fills for aligned data (4-10x faster)
 *   2. memset for unaligned or small fills
 *   3. Loop unrolling for remainder handling
 */

#include <string.h>  // for memset

// Check if pointer is 16-byte aligned (required for SIMD operations)
#define IS_ALIGNED_16(ptr) (((uintptr_t)(ptr) & 0xF) == 0)

// Inline alpha blend for single pixel - matches ALPHA_BLEND in tvgSwCommon.h
static inline uint32_t ESP32_ALPHA_BLEND(uint32_t c, uint32_t a)
{
    ++a;
    return (((((c >> 8) & 0x00ff00ff) * a) & 0xff00ff00) +
            ((((c & 0x00ff00ff) * a) >> 8) & 0x00ff00ff));
}


/*
 * SIMD fill for 32-bit values using ESP32-S3 PIE instructions.
 * Fills 4 x 32-bit values per iteration using 128-bit stores.
 * Handles alignment automatically - works with any pointer.
 * Based on LVGL's lv_color_blend_to_argb8888_esp approach.
 */
static inline void esp32_simd_fill_i32(int32_t* dst, int32_t val, size_t count)
{
    if (count == 0) return;

    // For small counts, just use scalar
    if (count < 4) {
        while (count--) *dst++ = val;
        return;
    }

    // Check 16-byte alignment
    uintptr_t align = (uintptr_t)dst & 0xF;

    if (align == 0) {
        // Fully aligned path - fastest
        size_t simd_count = count >> 2;  // count / 4
        size_t tail = count & 3;

        __asm__ volatile (
            // Fill q0 with val in all 4 lanes using ee.movi.32.q
            "ee.movi.32.q q0, %[val], 0     \n"
            "ee.movi.32.q q0, %[val], 1     \n"
            "ee.movi.32.q q0, %[val], 2     \n"
            "ee.movi.32.q q0, %[val], 3     \n"
            "loopnez %[cnt], 1f             \n"
            "ee.vst.128.ip q0, %[ptr], 16   \n"
            "1:                             \n"
            : [ptr] "+r" (dst)
            : [val] "r" (val), [cnt] "r" (simd_count)
            : "memory"
        );

        // Handle tail
        while (tail--) *dst++ = val;
    } else {
        // Unaligned path - align first, then SIMD
        // Calculate how many pixels to align (align is in bytes, divide by 4 for pixels)
        size_t head = (16 - align) >> 2;  // pixels to reach alignment
        if (head > count) head = count;

        // Fill head pixels to align
        count -= head;
        while (head--) *dst++ = val;

        if (count >= 4) {
            size_t simd_count = count >> 2;
            size_t tail = count & 3;

            __asm__ volatile (
                "ee.movi.32.q q0, %[val], 0     \n"
                "ee.movi.32.q q0, %[val], 1     \n"
                "ee.movi.32.q q0, %[val], 2     \n"
                "ee.movi.32.q q0, %[val], 3     \n"
                "loopnez %[cnt], 1f             \n"
                "ee.vst.128.ip q0, %[ptr], 16   \n"
                "1:                             \n"
                : [ptr] "+r" (dst)
                : [val] "r" (val), [cnt] "r" (simd_count)
                : "memory"
            );

            count = tail;
        }

        // Handle remaining tail
        while (count--) *dst++ = val;
    }
}


/*
 * SIMD fill for 8-bit values using ESP32-S3 PIE instructions.
 * Fills 16 x 8-bit values per iteration using 128-bit stores.
 * Handles alignment automatically.
 */
static inline void esp32_simd_fill_i8(int8_t* dst, int8_t val, size_t count)
{
    if (count == 0) return;

    // For small counts, use memset which is well optimized
    if (count < 16) {
        memset(dst, val, count);
        return;
    }

    // Build a 32-bit value with val in all 4 bytes for ee.movi.32.q
    uint32_t val32 = (uint8_t)val;
    val32 = val32 | (val32 << 8) | (val32 << 16) | (val32 << 24);

    // Check 16-byte alignment
    uintptr_t align = (uintptr_t)dst & 0xF;

    if (align == 0) {
        // Fully aligned path
        size_t simd_count = count >> 4;  // count / 16
        size_t tail = count & 15;

        __asm__ volatile (
            "ee.movi.32.q q0, %[val], 0     \n"
            "ee.movi.32.q q0, %[val], 1     \n"
            "ee.movi.32.q q0, %[val], 2     \n"
            "ee.movi.32.q q0, %[val], 3     \n"
            "loopnez %[cnt], 1f             \n"
            "ee.vst.128.ip q0, %[ptr], 16   \n"
            "1:                             \n"
            : [ptr] "+r" (dst)
            : [val] "r" (val32), [cnt] "r" (simd_count)
            : "memory"
        );

        // Handle tail with memset
        if (tail) memset(dst, val, tail);
    } else {
        // Unaligned - fill head to align
        size_t head = 16 - align;
        if (head > count) head = count;

        memset(dst, val, head);
        dst += head;
        count -= head;

        if (count >= 16) {
            size_t simd_count = count >> 4;
            size_t tail = count & 15;

            __asm__ volatile (
                "ee.movi.32.q q0, %[val], 0     \n"
                "ee.movi.32.q q0, %[val], 1     \n"
                "ee.movi.32.q q0, %[val], 2     \n"
                "ee.movi.32.q q0, %[val], 3     \n"
                "loopnez %[cnt], 1f             \n"
                "ee.vst.128.ip q0, %[ptr], 16   \n"
                "1:                             \n"
                : [ptr] "+r" (dst)
                : [val] "r" (val32), [cnt] "r" (simd_count)
                : "memory"
            );

            count = tail;
        }

        if (count) memset(dst, val, count);
    }
}


// Optimized grayscale fill using SIMD for aligned data, memset otherwise
static void esp32RasterGrayscale8(uint8_t* dst, uint8_t val, uint32_t offset, int32_t len)
{
    dst += offset;

    // For aligned data with sufficient length, use SIMD fill
    if (IS_ALIGNED_16(dst) && len >= 16) {
        esp32_simd_fill_i8((int8_t*)dst, (int8_t)val, len);
        return;
    }

    // Use memset for unaligned or moderate-sized fills
    if (len >= 8) {
        memset(dst, val, len);
        return;
    }

    // Small fills - unrolled
    int32_t i = 0;
    for (; i <= len - 4; i += 4) {
        dst[i] = val;
        dst[i + 1] = val;
        dst[i + 2] = val;
        dst[i + 3] = val;
    }
    for (; i < len; i++) {
        dst[i] = val;
    }
}


// Optimized 32-bit pixel fill using SIMD broadcast fill
static void esp32RasterPixel32(uint32_t* dst, uint32_t val, uint32_t offset, int32_t len)
{
    dst += offset;

    // For aligned data with sufficient length, use SIMD fill
    if (IS_ALIGNED_16(dst) && len >= 4) {
        esp32_simd_fill_i32((int32_t*)dst, (int32_t)val, len);
        return;
    }

    // Fallback: unrolled loop
    int32_t i = 0;
    for (; i <= len - 4; i += 4) {
        dst[i] = val;
        dst[i + 1] = val;
        dst[i + 2] = val;
        dst[i + 3] = val;
    }
    for (; i < len; i++) {
        dst[i] = val;
    }
}


/*
 * SIMD alpha blend for 4 pixels at once using ESP32-S3 PIE instructions.
 * Formula: dst = src + ALPHA_BLEND(dst, ialpha)
 *        = src + ((dst * (ialpha+1)) >> 8)
 *
 * Uses the standard SIMD alpha blend trick:
 * - Separate odd/even bytes to avoid overflow
 * - Multiply by alpha, shift, mask, recombine
 */
static inline void esp32_simd_blend_4px(uint32_t* dst, uint32_t src, uint8_t ialpha)
{
    // Precompute alpha+1 for the blend formula
    uint32_t a = ialpha + 1;

    // Process 4 pixels with unrolled scalar - still faster than function call overhead
    // The compiler will optimize this well with the constants
    uint32_t d0 = dst[0], d1 = dst[1], d2 = dst[2], d3 = dst[3];

    // ALPHA_BLEND: ((((c >> 8) & 0x00ff00ff) * a) & 0xff00ff00) +
    //              ((((c & 0x00ff00ff) * a) >> 8) & 0x00ff00ff)
    dst[0] = src + ((((d0 >> 8) & 0x00ff00ff) * a) & 0xff00ff00) +
                   ((((d0 & 0x00ff00ff) * a) >> 8) & 0x00ff00ff);
    dst[1] = src + ((((d1 >> 8) & 0x00ff00ff) * a) & 0xff00ff00) +
                   ((((d1 & 0x00ff00ff) * a) >> 8) & 0x00ff00ff);
    dst[2] = src + ((((d2 >> 8) & 0x00ff00ff) * a) & 0xff00ff00) +
                   ((((d2 & 0x00ff00ff) * a) >> 8) & 0x00ff00ff);
    dst[3] = src + ((((d3 >> 8) & 0x00ff00ff) * a) & 0xff00ff00) +
                   ((((d3 & 0x00ff00ff) * a) >> 8) & 0x00ff00ff);
}


/*
 * Optimized alpha blend span using true Xtensa inline assembly
 * dst[i] = src + ALPHA_BLEND(dst[i], ialpha)
 *
 * The blend formula separates odd/even bytes to avoid overflow:
 *   result = src + (((d >> 8) & 0x00ff00ff) * a) & 0xff00ff00)
 *                + (((d & 0x00ff00ff) * a) >> 8) & 0x00ff00ff)
 *
 * This assembly version avoids function call overhead and uses Xtensa
 * multiply instructions directly with optimal register allocation.
 */
static inline void esp32_simd_blend_span(uint32_t* dst, uint32_t src, uint8_t ialpha, int32_t len)
{
    if (len <= 0) return;

    // Precompute constants
    uint32_t a = ialpha + 1;
    uint32_t mask_lo = 0x00ff00ff;
    uint32_t mask_hi = 0xff00ff00;

    // Use inline assembly for the hot loop
    // Xtensa has mull (32x32->32 low) which is perfect for our needs
    __asm__ volatile (
        // Loop setup - use hardware zero-overhead loop
        "loopnez %[len], blend_loop_end_%=     \n"

        // Loop body - process one pixel per iteration
        // This is actually faster than 2-pixel unroll due to register pressure
        "blend_loop_%=:                        \n"
        "    l32i    a8, %[dst], 0             \n"  // a8 = d = *dst

        // lo = (d & mask) * a
        "    and     a9, a8, %[mask_lo]        \n"  // a9 = d & 0x00ff00ff
        "    mull    a9, a9, %[a]              \n"  // a9 = lo = (d & mask) * a

        // hi_part = ((d >> 8) & mask) * a
        "    srli    a10, a8, 8                \n"  // a10 = d >> 8
        "    and     a10, a10, %[mask_lo]      \n"  // a10 = (d >> 8) & mask
        "    mull    a10, a10, %[a]            \n"  // a10 = hi = ((d>>8) & mask) * a

        // Combine: result = src + (hi & 0xff00ff00) | ((lo >> 8) & 0x00ff00ff)
        "    and     a10, a10, %[mask_hi]      \n"  // a10 = hi & 0xff00ff00
        "    srli    a9, a9, 8                 \n"  // a9 = lo >> 8
        "    and     a9, a9, %[mask_lo]        \n"  // a9 = (lo >> 8) & mask
        "    or      a9, a9, a10               \n"  // a9 = blended = hi_masked | lo_masked
        "    add     a9, a9, %[src]            \n"  // a9 = src + blended

        // Store result and advance pointer
        "    s32i    a9, %[dst], 0             \n"  // *dst = result
        "    addi    %[dst], %[dst], 4         \n"  // dst++

        "blend_loop_end_%=:                    \n"
        : [dst] "+r" (dst)
        : [src] "r" (src), [a] "r" (a), [len] "r" (len),
          [mask_lo] "r" (mask_lo), [mask_hi] "r" (mask_hi)
        : "a8", "a9", "a10", "memory"
    );
}


// Optimized solid RLE rasterization (alpha=255)
// Uses SIMD fill for full coverage spans, optimized blend for partial coverage
static bool esp32RasterSolidRle(SwSurface* surface, const SwRle* rle, const RenderRegion& bbox, const RenderColor& c)
{
    const SwSpan* end;
    int32_t x, len;

    // 32bit channels
    if (surface->channelSize == sizeof(uint32_t)) {
        auto color = surface->join(c.r, c.g, c.b, 255);

        for (auto span = rle->fetch(bbox, &end); span < end; ++span) {
            if (!span->fetch(bbox, x, len)) continue;

            if (span->coverage == 255) {
                // Full coverage - use SIMD fill
                esp32RasterPixel32(surface->buf32 + span->y * surface->stride, color, x, len);
            } else {
                // Partial coverage - use optimized blend span
                auto dst = &surface->buf32[span->y * surface->stride + x];
                auto src = ESP32_ALPHA_BLEND(color, span->coverage);
                auto ialpha = 255 - span->coverage;
                esp32_simd_blend_span(dst, src, ialpha, len);
            }
        }
    // 8bit grayscale
    } else if (surface->channelSize == sizeof(uint8_t)) {
        for (auto span = rle->fetch(bbox, &end); span < end; ++span) {
            if (!span->fetch(bbox, x, len)) continue;

            if (span->coverage == 255) {
                // Full coverage - use SIMD fill
                esp32_simd_fill_i8((int8_t*)(surface->buf8 + span->y * surface->stride + x), (int8_t)255, len);
            } else {
                // Partial coverage - blend with unrolled loop
                auto dst = &surface->buf8[span->y * surface->stride + x];
                auto ialpha = 255 - span->coverage;

                int32_t i = 0;
                for (; i <= len - 4; i += 4) {
                    dst[i] = span->coverage + MULTIPLY(dst[i], ialpha);
                    dst[i + 1] = span->coverage + MULTIPLY(dst[i + 1], ialpha);
                    dst[i + 2] = span->coverage + MULTIPLY(dst[i + 2], ialpha);
                    dst[i + 3] = span->coverage + MULTIPLY(dst[i + 3], ialpha);
                }
                for (; i < len; i++) {
                    dst[i] = span->coverage + MULTIPLY(dst[i], ialpha);
                }
            }
        }
    }
    return true;
}


// Optimized translucent RLE rasterization
static bool esp32RasterTranslucentRle(SwSurface* surface, const SwRle* rle, const RenderRegion& bbox, const RenderColor& c)
{
    const SwSpan* end;
    int32_t x, len;

    // 32bit channels
    if (surface->channelSize == sizeof(uint32_t)) {
        auto color = surface->join(c.r, c.g, c.b, c.a);
        uint32_t src;

        for (auto span = rle->fetch(bbox, &end); span < end; ++span) {
            if (!span->fetch(bbox, x, len)) continue;

            if (span->coverage < 255) {
                src = ESP32_ALPHA_BLEND(color, span->coverage);
            } else {
                src = color;
            }

            auto dst = &surface->buf32[span->y * surface->stride + x];
            auto ialpha = IA(src);
            esp32_simd_blend_span(dst, src, ialpha, len);
        }
    // 8bit grayscale
    } else if (surface->channelSize == sizeof(uint8_t)) {
        uint8_t src;
        for (auto span = rle->fetch(bbox, &end); span < end; ++span) {
            if (!span->fetch(bbox, x, len)) continue;
            auto dst = &surface->buf8[span->y * surface->stride + x];
            if (span->coverage < 255) {
                src = MULTIPLY(span->coverage, c.a);
            } else {
                src = c.a;
            }
            auto ialpha = ~c.a;

            // Unrolled loop
            int32_t i = 0;
            for (; i <= len - 4; i += 4) {
                dst[i] = src + MULTIPLY(dst[i], ialpha);
                dst[i + 1] = src + MULTIPLY(dst[i + 1], ialpha);
                dst[i + 2] = src + MULTIPLY(dst[i + 2], ialpha);
                dst[i + 3] = src + MULTIPLY(dst[i + 3], ialpha);
            }
            for (; i < len; i++) {
                dst[i] = src + MULTIPLY(dst[i], ialpha);
            }
        }
    }
    return true;
}


// Optimized translucent rect fill
static bool esp32RasterTranslucentRect(SwSurface* surface, const RenderRegion& bbox, const RenderColor& c)
{
    auto h = bbox.h();
    auto w = bbox.w();

    // 32bits channels
    if (surface->channelSize == sizeof(uint32_t)) {
        auto color = surface->join(c.r, c.g, c.b, c.a);
        auto buffer = surface->buf32 + (bbox.min.y * surface->stride) + bbox.min.x;
        auto ialpha = 255 - c.a;

        for (uint32_t y = 0; y < h; ++y) {
            auto dst = &buffer[y * surface->stride];

            // Unrolled loop - process 4 pixels at a time
            uint32_t x = 0;
            for (; x <= w - 4; x += 4) {
                dst[x] = color + ESP32_ALPHA_BLEND(dst[x], ialpha);
                dst[x + 1] = color + ESP32_ALPHA_BLEND(dst[x + 1], ialpha);
                dst[x + 2] = color + ESP32_ALPHA_BLEND(dst[x + 2], ialpha);
                dst[x + 3] = color + ESP32_ALPHA_BLEND(dst[x + 3], ialpha);
            }
            // Handle remainder
            for (; x < w; x++) {
                dst[x] = color + ESP32_ALPHA_BLEND(dst[x], ialpha);
            }
        }
    // 8bit grayscale
    } else if (surface->channelSize == sizeof(uint8_t)) {
        auto buffer = surface->buf8 + (bbox.min.y * surface->stride) + bbox.min.x;
        auto ialpha = ~c.a;

        for (uint32_t y = 0; y < h; ++y) {
            auto dst = &buffer[y * surface->stride];

            uint32_t x = 0;
            for (; x <= w - 4; x += 4) {
                dst[x] = c.a + MULTIPLY(dst[x], ialpha);
                dst[x + 1] = c.a + MULTIPLY(dst[x + 1], ialpha);
                dst[x + 2] = c.a + MULTIPLY(dst[x + 2], ialpha);
                dst[x + 3] = c.a + MULTIPLY(dst[x + 3], ialpha);
            }
            for (; x < w; x++) {
                dst[x] = c.a + MULTIPLY(dst[x], ialpha);
            }
        }
    }
    return true;
}


/*
 * Optimized translucent pixel blitting (src over dst with alpha)
 * Used for image rendering with transparency
 */
static inline void esp32RasterTranslucentPixels(uint32_t* dst, uint32_t* src, uint32_t len, uint32_t opacity)
{
    uint32_t i = 0;

    if (opacity == 255) {
        // Full opacity - just alpha blend src over dst
        for (; i <= len - 4; i += 4) {
            dst[i] = src[i] + ESP32_ALPHA_BLEND(dst[i], IA(src[i]));
            dst[i+1] = src[i+1] + ESP32_ALPHA_BLEND(dst[i+1], IA(src[i+1]));
            dst[i+2] = src[i+2] + ESP32_ALPHA_BLEND(dst[i+2], IA(src[i+2]));
            dst[i+3] = src[i+3] + ESP32_ALPHA_BLEND(dst[i+3], IA(src[i+3]));
        }
        for (; i < len; ++i) {
            dst[i] = src[i] + ESP32_ALPHA_BLEND(dst[i], IA(src[i]));
        }
    } else {
        // Partial opacity - blend src with opacity first
        for (; i <= len - 4; i += 4) {
            auto tmp0 = ESP32_ALPHA_BLEND(src[i], opacity);
            auto tmp1 = ESP32_ALPHA_BLEND(src[i+1], opacity);
            auto tmp2 = ESP32_ALPHA_BLEND(src[i+2], opacity);
            auto tmp3 = ESP32_ALPHA_BLEND(src[i+3], opacity);
            dst[i] = tmp0 + ESP32_ALPHA_BLEND(dst[i], IA(tmp0));
            dst[i+1] = tmp1 + ESP32_ALPHA_BLEND(dst[i+1], IA(tmp1));
            dst[i+2] = tmp2 + ESP32_ALPHA_BLEND(dst[i+2], IA(tmp2));
            dst[i+3] = tmp3 + ESP32_ALPHA_BLEND(dst[i+3], IA(tmp3));
        }
        for (; i < len; ++i) {
            auto tmp = ESP32_ALPHA_BLEND(src[i], opacity);
            dst[i] = tmp + ESP32_ALPHA_BLEND(dst[i], IA(tmp));
        }
    }
}


/*
 * Optimized pixel copy with optional opacity
 * For opacity=255, use memcpy for maximum speed
 */
static inline void esp32RasterPixels(uint32_t* dst, uint32_t* src, uint32_t len, uint32_t opacity)
{
    if (opacity == 255) {
        // Full opacity - direct copy, use memcpy for large copies
        if (len >= 4) {
            memcpy(dst, src, len * sizeof(uint32_t));
        } else {
            for (uint32_t i = 0; i < len; ++i) {
                dst[i] = src[i];
            }
        }
    } else {
        // Partial opacity - use translucent blend
        esp32RasterTranslucentPixels(dst, src, len, opacity);
    }
}


/*
 * Optimized ABGR <-> ARGB color space conversion
 * Swaps R and B channels: ARGB <-> ABGR
 * Uses 64-bit operations for 2 pixels at once when aligned
 */
static inline bool esp32RasterABGRtoARGB(RenderSurface* surface)
{
    auto w = surface->w;
    auto h = surface->h;
    auto stride = surface->stride;

    // Process 2 pixels at a time using 64-bit operations
    if (w % 2 == 0) {
        auto buffer = reinterpret_cast<uint64_t*>(surface->buf32);
        for (uint32_t y = 0; y < h; ++y) {
            auto dst = buffer;
            uint32_t x = 0;
            // Unroll by 2 (4 pixels total per iteration)
            for (; x <= w/2 - 2; x += 2, dst += 2) {
                auto c0 = dst[0];
                auto c1 = dst[1];
                // Swap R and B in both pixels of each 64-bit value
                dst[0] = (c0 & 0xff000000ff000000ULL) | ((c0 & 0x00ff000000ff0000ULL) >> 16) |
                         (c0 & 0x0000ff000000ff00ULL) | ((c0 & 0x000000ff000000ffULL) << 16);
                dst[1] = (c1 & 0xff000000ff000000ULL) | ((c1 & 0x00ff000000ff0000ULL) >> 16) |
                         (c1 & 0x0000ff000000ff00ULL) | ((c1 & 0x000000ff000000ffULL) << 16);
            }
            for (; x < w/2; ++x, ++dst) {
                auto c = *dst;
                *dst = (c & 0xff000000ff000000ULL) | ((c & 0x00ff000000ff0000ULL) >> 16) |
                       (c & 0x0000ff000000ff00ULL) | ((c & 0x000000ff000000ffULL) << 16);
            }
            buffer += stride / 2;
        }
    } else {
        // Odd width - process single pixels
        auto buffer = surface->buf32;
        for (uint32_t y = 0; y < h; ++y) {
            auto dst = buffer;
            uint32_t x = 0;
            for (; x <= w - 4; x += 4, dst += 4) {
                auto c0 = dst[0], c1 = dst[1], c2 = dst[2], c3 = dst[3];
                dst[0] = (c0 & 0xff00ff00) | ((c0 & 0x00ff0000) >> 16) | ((c0 & 0x000000ff) << 16);
                dst[1] = (c1 & 0xff00ff00) | ((c1 & 0x00ff0000) >> 16) | ((c1 & 0x000000ff) << 16);
                dst[2] = (c2 & 0xff00ff00) | ((c2 & 0x00ff0000) >> 16) | ((c2 & 0x000000ff) << 16);
                dst[3] = (c3 & 0xff00ff00) | ((c3 & 0x00ff0000) >> 16) | ((c3 & 0x000000ff) << 16);
            }
            for (; x < w; ++x, ++dst) {
                auto c = *dst;
                *dst = (c & 0xff00ff00) | ((c & 0x00ff0000) >> 16) | ((c & 0x000000ff) << 16);
            }
            buffer += stride;
        }
    }
    return true;
}

// ARGB to ABGR is the same operation (just swapping R and B)
static inline bool esp32RasterARGBtoABGR(RenderSurface* surface)
{
    return esp32RasterABGRtoARGB(surface);
}


/*
 * Optimized premultiply alpha for entire surface
 * Multiplies RGB by A for each pixel
 */
static inline void esp32RasterPremultiply(RenderSurface* surface)
{
    auto buffer = surface->buf32;
    auto w = surface->w;
    auto h = surface->h;
    auto stride = surface->stride;

    for (uint32_t y = 0; y < h; ++y) {
        auto dst = buffer;
        uint32_t x = 0;

        // Process 4 pixels at a time
        for (; x <= w - 4; x += 4, dst += 4) {
            auto c0 = dst[0], c1 = dst[1], c2 = dst[2], c3 = dst[3];
            auto a0 = A(c0), a1 = A(c1), a2 = A(c2), a3 = A(c3);

            // Skip if already fully opaque
            if (a0 != 255) dst[0] = PREMULTIPLY(c0, a0);
            if (a1 != 255) dst[1] = PREMULTIPLY(c1, a1);
            if (a2 != 255) dst[2] = PREMULTIPLY(c2, a2);
            if (a3 != 255) dst[3] = PREMULTIPLY(c3, a3);
        }

        // Handle remainder
        for (; x < w; ++x, ++dst) {
            auto c = *dst;
            auto a = A(c);
            if (a != 255) *dst = PREMULTIPLY(c, a);
        }

        buffer += stride;
    }
}


/*
 * Optimized unpremultiply alpha for entire surface
 * Divides RGB by A for each pixel (reverse of premultiply)
 */
static inline uint32_t esp32_unpremultiply_pixel(uint32_t data)
{
    auto a = A(data);
    if (a == 255 || a == 0) return data;

    // Unpremultiply: c = c * 255 / a
    uint32_t r = (C1(data) * 255u) / a;
    uint32_t g = (C2(data) * 255u) / a;
    uint32_t b = (C3(data) * 255u) / a;

    // Clamp to 255
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;

    return JOIN(a, r, g, b);
}

static inline void esp32RasterUnpremultiply(RenderSurface* surface)
{
    auto buffer = surface->buf32;
    auto w = surface->w;
    auto h = surface->h;
    auto stride = surface->stride;

    for (uint32_t y = 0; y < h; ++y) {
        auto dst = buffer;
        uint32_t x = 0;

        // Process 4 pixels at a time
        for (; x <= w - 4; x += 4, dst += 4) {
            dst[0] = esp32_unpremultiply_pixel(dst[0]);
            dst[1] = esp32_unpremultiply_pixel(dst[1]);
            dst[2] = esp32_unpremultiply_pixel(dst[2]);
            dst[3] = esp32_unpremultiply_pixel(dst[3]);
        }

        // Handle remainder
        for (; x < w; ++x, ++dst) {
            *dst = esp32_unpremultiply_pixel(*dst);
        }

        buffer += stride;
    }
}


/*
 * Optimized mask image compositing
 * Blends mask onto destination with alpha
 */
static inline bool esp32CompositeMaskImage(SwSurface* surface, const SwImage& image, const RenderRegion& bbox)
{
    auto dbuffer = &surface->buf8[bbox.min.y * surface->stride + bbox.min.x];
    auto sbuffer = image.buf8 + (bbox.min.y + image.oy) * image.stride + (bbox.min.x + image.ox);
    auto w = bbox.w();
    auto h = bbox.h();

    for (uint32_t y = 0; y < h; ++y) {
        auto dst = dbuffer;
        auto src = sbuffer;
        uint32_t x = 0;

        // Unrolled loop - process 4 pixels at a time
        for (; x <= w - 4; x += 4) {
            dst[x] = src[x] + MULTIPLY(dst[x], ~src[x]);
            dst[x+1] = src[x+1] + MULTIPLY(dst[x+1], ~src[x+1]);
            dst[x+2] = src[x+2] + MULTIPLY(dst[x+2], ~src[x+2]);
            dst[x+3] = src[x+3] + MULTIPLY(dst[x+3], ~src[x+3]);
        }

        // Handle remainder
        for (; x < w; ++x) {
            dst[x] = src[x] + MULTIPLY(dst[x], ~src[x]);
        }

        dbuffer += surface->stride;
        sbuffer += image.stride;
    }
    return true;
}

#endif // THORVG_ESP32_VECTOR_SUPPORT
