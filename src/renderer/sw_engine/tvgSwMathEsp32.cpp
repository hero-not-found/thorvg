/*
 * Copyright (c) 2020 - 2025 the ThorVG project. All rights reserved.
 * ESP32-S3 optimized version of tvgSwMath.cpp
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

#include "tvgMath.h"
#include "tvgSwCommon.h"

/************************************************************************/
/* ESP32-S3 Optimized Fast Math                                         */
/************************************************************************/

// Constants
static constexpr float CONST_PI = 3.14159265358979f;
static constexpr float CONST_TWO_PI = 6.28318530717959f;
static constexpr float CONST_HALF_PI = 1.57079632679f;

// Sine lookup table - 256 entries for [0, PI/2]
// Values are Q15 fixed-point (multiply by 1/32768 to get float)
// Table covers 0 to 90 degrees, other quadrants derived by symmetry
static const int16_t SIN_TABLE[257] = {
        0,   402,   804,  1206,  1608,  2009,  2411,  2811,
     3212,  3612,  4011,  4410,  4808,  5205,  5602,  5998,
     6393,  6787,  7180,  7571,  7962,  8351,  8740,  9127,
     9512,  9896, 10279, 10660, 11039, 11417, 11793, 12167,
    12540, 12910, 13279, 13646, 14010, 14373, 14733, 15091,
    15447, 15800, 16151, 16500, 16846, 17190, 17531, 17869,
    18205, 18538, 18868, 19195, 19520, 19841, 20160, 20475,
    20788, 21097, 21403, 21706, 22006, 22302, 22595, 22884,
    23170, 23453, 23732, 24008, 24279, 24548, 24812, 25073,
    25330, 25583, 25833, 26078, 26320, 26557, 26791, 27020,
    27246, 27467, 27684, 27897, 28106, 28311, 28511, 28707,
    28899, 29086, 29269, 29448, 29622, 29792, 29957, 30118,
    30274, 30425, 30572, 30715, 30853, 30986, 31114, 31238,
    31357, 31471, 31581, 31686, 31786, 31881, 31972, 32058,
    32138, 32214, 32286, 32352, 32413, 32470, 32522, 32568,
    32610, 32647, 32679, 32706, 32729, 32746, 32758, 32766,
    32767, 32766, 32758, 32746, 32729, 32706, 32679, 32647,
    32610, 32568, 32522, 32470, 32413, 32352, 32286, 32214,
    32138, 32058, 31972, 31881, 31786, 31686, 31581, 31471,
    31357, 31238, 31114, 30986, 30853, 30715, 30572, 30425,
    30274, 30118, 29957, 29792, 29622, 29448, 29269, 29086,
    28899, 28707, 28511, 28311, 28106, 27897, 27684, 27467,
    27246, 27020, 26791, 26557, 26320, 26078, 25833, 25583,
    25330, 25073, 24812, 24548, 24279, 24008, 23732, 23453,
    23170, 22884, 22595, 22302, 22006, 21706, 21403, 21097,
    20788, 20475, 20160, 19841, 19520, 19195, 18868, 18538,
    18205, 17869, 17531, 17190, 16846, 16500, 16151, 15800,
    15447, 15091, 14733, 14373, 14010, 13646, 13279, 12910,
    12540, 12167, 11793, 11417, 11039, 10660, 10279,  9896,
     9512,  9127,  8740,  8351,  7962,  7571,  7180,  6787,
     6393,  5998,  5602,  5205,  4808,  4410,  4011,  3612,
     3212,  2811,  2411,  2009,  1608,  1206,   804,   402,
        0
};

// Fast sine using lookup table with linear interpolation
// Input: angle in radians
// Output: sine value as float [-1, 1]
static inline float fast_sinf(float x)
{
    // Normalize to [0, 2*PI)
    // Use multiplication instead of fmod for speed
    const float INV_TWO_PI = 0.15915494309f;
    x = x - CONST_TWO_PI * floorf(x * INV_TWO_PI);
    if (x < 0) x += CONST_TWO_PI;

    // Convert to table index: 256 entries per PI (512 per 2*PI)
    // index = x * (256 / PI) = x * 81.4873...
    const float SCALE = 256.0f / CONST_PI;
    float idx_f = x * SCALE;
    int idx = (int)idx_f;
    float frac = idx_f - idx;

    // Handle quadrants using symmetry
    int16_t val;
    if (idx < 256) {
        // First quadrant [0, PI/2) or second quadrant [PI/2, PI)
        val = SIN_TABLE[idx] + (int16_t)((SIN_TABLE[idx + 1] - SIN_TABLE[idx]) * frac);
    } else if (idx < 512) {
        // Third quadrant [PI, 3*PI/2) or fourth quadrant [3*PI/2, 2*PI)
        idx -= 256;
        val = -(SIN_TABLE[idx] + (int16_t)((SIN_TABLE[idx + 1] - SIN_TABLE[idx]) * frac));
    } else {
        val = 0;
    }

    return val * (1.0f / 32767.0f);
}

static inline float fast_cosf(float x)
{
    return fast_sinf(x + CONST_HALF_PI);
}

// Fast atan2 approximation using polynomial
// Accurate to ~0.07 radians (~4 degrees)
static inline float fast_atan2f(float y, float x)
{
    if (x == 0.0f) {
        if (y > 0.0f) return CONST_HALF_PI;
        if (y < 0.0f) return -CONST_HALF_PI;
        return 0.0f;
    }

    float abs_y = (y < 0) ? -y : y;
    abs_y += 1e-10f;  // Prevent div by zero
    float angle;

    if (x >= 0.0f) {
        float r = (x - abs_y) / (x + abs_y);
        float r3 = r * r * r;
        angle = 0.1963f * r3 - 0.9817f * r + CONST_HALF_PI * 0.5f;
    } else {
        float r = (x + abs_y) / (abs_y - x);
        float r3 = r * r * r;
        angle = 0.1963f * r3 - 0.9817f * r + CONST_HALF_PI * 1.5f;
    }

    return (y < 0.0f) ? -angle : angle;
}

// Fast tan using sin/cos
static inline float fast_tanf(float x)
{
    float s = fast_sinf(x);
    float c = fast_cosf(x);
    // Avoid division when cos is near zero
    if (c > -1e-6f && c < 1e-6f) return (s >= 0) ? 1e10f : -1e10f;
    return s / c;
}

/************************************************************************/
/* Internal Class Implementation                                        */
/************************************************************************/

static inline float TO_RADIAN(int64_t angle)
{
    return (float(angle) * (1.0f / 65536.0f)) * (CONST_PI / 180.0f);
}

/************************************************************************/
/* External Class Implementation                                        */
/************************************************************************/

int64_t mathMean(int64_t angle1, int64_t angle2)
{
    return angle1 + mathDiff(angle1, angle2) / 2;
}


int mathCubicAngle(const SwPoint* base, int64_t& angleIn, int64_t& angleMid, int64_t& angleOut)
{
    auto d1 = base[2] - base[3];
    auto d2 = base[1] - base[2];
    auto d3 = base[0] - base[1];

    if (d1.tiny()) {
        if (d2.tiny()) {
            if (d3.tiny()) {
                angleIn = angleMid = angleOut = 0;
                return -1;  //ignoreable
            } else {
                angleIn = angleMid = angleOut = mathAtan(d3);
            }
        } else {
            if (d3.tiny()) {
                angleIn = angleMid = angleOut = mathAtan(d2);
            } else {
                angleIn = angleMid = mathAtan(d2);
                angleOut = mathAtan(d3);
            }
        }
    } else {
        if (d2.tiny()) {
            if (d3.tiny()) {
                angleIn = angleMid = angleOut = mathAtan(d1);
            } else {
                angleIn = mathAtan(d1);
                angleOut = mathAtan(d3);
                angleMid = mathMean(angleIn, angleOut);
            }
        } else {
            if (d3.tiny()) {
                angleIn = mathAtan(d1);
                angleMid = angleOut = mathAtan(d2);
            } else {
                angleIn = mathAtan(d1);
                angleMid = mathAtan(d2);
                angleOut = mathAtan(d3);
            }
        }
    }

    auto theta1 = abs(mathDiff(angleIn, angleMid));
    auto theta2 = abs(mathDiff(angleMid, angleOut));

    if ((theta1 < (SW_ANGLE_PI / 8)) && (theta2 < (SW_ANGLE_PI / 8))) return 0; //small size
    return 1;
}


// Optimized 16.16 fixed-point multiply
int64_t mathMultiply(int64_t a, int64_t b)
{
    // Handle signs separately for better branch prediction
    int32_t s = ((a ^ b) < 0) ? -1 : 1;
    if (a < 0) a = -a;
    if (b < 0) b = -b;

    // Fixed-point multiply with rounding: (a * b + 0x8000) >> 16
    int64_t c = (a * b + 0x8000L) >> 16;
    return (s > 0) ? c : -c;
}


// Optimized 16.16 fixed-point divide
int64_t mathDivide(int64_t a, int64_t b)
{
    if (b == 0) return 0x7FFFFFFFL;

    int32_t s = ((a ^ b) < 0) ? -1 : 1;
    if (a < 0) a = -a;
    if (b < 0) b = -b;

    // Fixed-point divide with rounding: ((a << 16) + (b >> 1)) / b
    int64_t q = ((a << 16) + (b >> 1)) / b;
    return (s > 0) ? q : -q;
}


int64_t mathMulDiv(int64_t a, int64_t b, int64_t c)
{
    if (c == 0) return 0x7FFFFFFFL;

    // Calculate combined sign
    int32_t s = 1;
    if (a < 0) { a = -a; s = -s; }
    if (b < 0) { b = -b; s = -s; }
    if (c < 0) { c = -c; s = -s; }

    int64_t d = (a * b + (c >> 1)) / c;
    return (s > 0) ? d : -d;
}


void mathRotate(SwPoint& pt, int64_t angle)
{
    if (angle == 0 || pt.zero()) return;

    Point v = pt.toPoint();

    auto radian = TO_RADIAN(angle);
    auto cosv = fast_cosf(radian);
    auto sinv = fast_sinf(radian);

    // Compute rotation: [cos -sin; sin cos] * [x; y]
    float rx = v.x * cosv - v.y * sinv;
    float ry = v.x * sinv + v.y * cosv;

    // Scale and round to fixed-point
    pt.x = int32_t(rx * 64.0f + 0.5f);
    pt.y = int32_t(ry * 64.0f + 0.5f);
}


int64_t mathTan(int64_t angle)
{
    if (angle == 0) return 0;
    return int64_t(fast_tanf(TO_RADIAN(angle)) * 65536.0f);
}


int64_t mathAtan(const SwPoint& pt)
{
    if (pt.zero()) return 0;
    return int64_t(fast_atan2f(TO_FLOAT(pt.y), TO_FLOAT(pt.x)) * (180.0f / CONST_PI) * 65536.0f);
}


int64_t mathSin(int64_t angle)
{
    if (angle == 0) return 0;
    return mathCos(SW_ANGLE_PI2 - angle);
}


int64_t mathCos(int64_t angle)
{
    return int64_t(fast_cosf(TO_RADIAN(angle)) * 65536.0f);
}


// Integer-only vector length using alpha-max-beta-min algorithm
// Avoids all float operations - ~7% max error
// ESP32-S3 optimized with ABS/MIN/MAX assembly instructions
int64_t mathLength(const SwPoint& pt)
{
    if (pt.zero()) return 0;

    // Use ESP32-S3 ABS instruction for absolute values
    int32_t ax, ay;
    __asm__ volatile("abs %0, %1" : "=r"(ax) : "r"(pt.x));
    __asm__ volatile("abs %0, %1" : "=r"(ay) : "r"(pt.y));

    // Trivial cases
    if (ax == 0) return ay;
    if (ay == 0) return ax;

    // Alpha-max-beta-min: max + 3/8 * min
    // Use ESP32-S3 MIN/MAX instructions
    int32_t max_val, min_val;
    __asm__ volatile("max %0, %1, %2" : "=r"(max_val) : "r"(ax), "r"(ay));
    __asm__ volatile("min %0, %1, %2" : "=r"(min_val) : "r"(ax), "r"(ay));

    // Using shift: 3/8 = (1/4 + 1/8) = (x >> 2) + (x >> 3)
    return max_val + (min_val >> 2) + (min_val >> 3);
}


void mathSplitCubic(SwPoint* base)
{
    int32_t a, b, c, d;

    // X coordinates
    base[6].x = base[3].x;
    c = base[1].x;
    d = base[2].x;
    base[1].x = a = (base[0].x + c) >> 1;
    base[5].x = b = (base[3].x + d) >> 1;
    c = (c + d) >> 1;
    base[2].x = a = (a + c) >> 1;
    base[4].x = b = (b + c) >> 1;
    base[3].x = (a + b) >> 1;

    // Y coordinates
    base[6].y = base[3].y;
    c = base[1].y;
    d = base[2].y;
    base[1].y = a = (base[0].y + c) >> 1;
    base[5].y = b = (base[3].y + d) >> 1;
    c = (c + d) >> 1;
    base[2].y = a = (a + c) >> 1;
    base[4].y = b = (b + c) >> 1;
    base[3].y = (a + b) >> 1;
}


void mathSplitLine(SwPoint* base)
{
    base[2] = base[1];
    base[1].x = (base[0].x + base[1].x) >> 1;
    base[1].y = (base[0].y + base[1].y) >> 1;
}


int64_t mathDiff(int64_t angle1, int64_t angle2)
{
    auto delta = angle2 - angle1;

    delta %= SW_ANGLE_2PI;
    if (delta < 0) delta += SW_ANGLE_2PI;
    if (delta > SW_ANGLE_PI) delta -= SW_ANGLE_2PI;

    return delta;
}


SwPoint mathTransform(const Point* to, const Matrix& transform)
{
    auto tx = to->x * transform.e11 + to->y * transform.e12 + transform.e13;
    auto ty = to->x * transform.e21 + to->y * transform.e22 + transform.e23;

    return {TO_SWCOORD(tx), TO_SWCOORD(ty)};
}


// ESP32-S3 optimized bounding box calculation with MIN/MAX assembly
bool mathUpdateOutlineBBox(const SwOutline* outline, const RenderRegion& clipBox, RenderRegion& renderBox, bool fastTrack)
{
    if (!outline || outline->pts.empty() || outline->cntrs.empty()) {
        renderBox.reset();
        return false;
    }

    auto pt = outline->pts.begin();
    auto end = outline->pts.end();

    int32_t xMin = pt->x;
    int32_t xMax = pt->x;
    int32_t yMin = pt->y;
    int32_t yMax = pt->y;
    ++pt;

    // Process points - unroll by 4 with ESP32-S3 MIN/MAX assembly
    size_t count = end - pt;
    size_t chunks = count >> 2;

    for (size_t i = 0; i < chunks; ++i) {
        // Load 4 points worth of data
        int32_t x0 = pt[0].x, y0 = pt[0].y;
        int32_t x1 = pt[1].x, y1 = pt[1].y;
        int32_t x2 = pt[2].x, y2 = pt[2].y;
        int32_t x3 = pt[3].x, y3 = pt[3].y;

        // Find local min/max for x using ESP32-S3 MIN/MAX instructions
        int32_t localXMin = x0, localXMax = x0;
        __asm__ volatile("min %0, %0, %1" : "+r"(localXMin) : "r"(x1));
        __asm__ volatile("max %0, %0, %1" : "+r"(localXMax) : "r"(x1));
        __asm__ volatile("min %0, %0, %1" : "+r"(localXMin) : "r"(x2));
        __asm__ volatile("max %0, %0, %1" : "+r"(localXMax) : "r"(x2));
        __asm__ volatile("min %0, %0, %1" : "+r"(localXMin) : "r"(x3));
        __asm__ volatile("max %0, %0, %1" : "+r"(localXMax) : "r"(x3));

        // Find local min/max for y using ESP32-S3 MIN/MAX instructions
        int32_t localYMin = y0, localYMax = y0;
        __asm__ volatile("min %0, %0, %1" : "+r"(localYMin) : "r"(y1));
        __asm__ volatile("max %0, %0, %1" : "+r"(localYMax) : "r"(y1));
        __asm__ volatile("min %0, %0, %1" : "+r"(localYMin) : "r"(y2));
        __asm__ volatile("max %0, %0, %1" : "+r"(localYMax) : "r"(y2));
        __asm__ volatile("min %0, %0, %1" : "+r"(localYMin) : "r"(y3));
        __asm__ volatile("max %0, %0, %1" : "+r"(localYMax) : "r"(y3));

        // Merge with global using MIN/MAX
        __asm__ volatile("min %0, %0, %1" : "+r"(xMin) : "r"(localXMin));
        __asm__ volatile("max %0, %0, %1" : "+r"(xMax) : "r"(localXMax));
        __asm__ volatile("min %0, %0, %1" : "+r"(yMin) : "r"(localYMin));
        __asm__ volatile("max %0, %0, %1" : "+r"(yMax) : "r"(localYMax));

        pt += 4;
    }

    // Handle remaining points with MIN/MAX
    for (; pt < end; ++pt) {
        __asm__ volatile("min %0, %0, %1" : "+r"(xMin) : "r"(pt->x));
        __asm__ volatile("max %0, %0, %1" : "+r"(xMax) : "r"(pt->x));
        __asm__ volatile("min %0, %0, %1" : "+r"(yMin) : "r"(pt->y));
        __asm__ volatile("max %0, %0, %1" : "+r"(yMax) : "r"(pt->y));
    }

    if (fastTrack) {
        // Use integer rounding: (x + 32) >> 6 is approximately round(x/64)
        renderBox.min = {(xMin + 32) >> 6, (yMin + 32) >> 6};
        renderBox.max = {(xMax + 32) >> 6, (yMax + 32) >> 6};
    } else {
        renderBox.min = {xMin >> 6, yMin >> 6};
        renderBox.max = {(xMax + 63) >> 6, (yMax + 63) >> 6};
    }

    renderBox.intersect(clipBox);
    return renderBox.valid();
}
