/*
 * Copyright (c) 2021 - 2025 the ThorVG project. All rights reserved.
 * ESP32-S3 optimized version using ESP-DSP library

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "tvgMath.h"

// ESP-DSP function declaration - resolved at final firmware link time
// ESP-DSP is linked via modgfx.cmake linker group
extern "C" {
    int dspm_mult_3x3x3_f32_ae32(const float *A, const float *B, float *C);
}

#define BEZIER_EPSILON 1e-2f


/************************************************************************/
/* Internal Class Implementation                                        */
/************************************************************************/

static float _lineLengthApprox(const Point& pt1, const Point& pt2)
{
    /* approximate sqrt(x*x + y*y) using alpha max plus beta min algorithm.
       With alpha = 1, beta = 3/8, giving results with the largest error less
       than 7% compared to the exact value. */
    Point diff = {pt2.x - pt1.x, pt2.y - pt1.y};
    if (diff.x < 0) diff.x = -diff.x;
    if (diff.y < 0) diff.y = -diff.y;
    return (diff.x > diff.y) ? (diff.x + diff.y * 0.375f) : (diff.y + diff.x * 0.375f);
}


static float _lineLength(const Point& pt1, const Point& pt2)
{
    Point diff = {pt2.x - pt1.x, pt2.y - pt1.y};
    return sqrtf(diff.x * diff.x + diff.y * diff.y);
}


template<typename LengthFunc>
float _bezLength(const Bezier& cur, LengthFunc lineLengthFunc)
{
    Bezier left, right;
    auto len = lineLengthFunc(cur.start, cur.ctrl1) + lineLengthFunc(cur.ctrl1, cur.ctrl2) + lineLengthFunc(cur.ctrl2, cur.end);
    auto chord = lineLengthFunc(cur.start, cur.end);

    if (fabsf(len - chord) > BEZIER_EPSILON) {
        cur.split(left, right);
        return _bezLength(left, lineLengthFunc) + _bezLength(right, lineLengthFunc);
    }
    return len;
}


template<typename LengthFunc>
float _bezAt(const Bezier& bz, float at, float length, LengthFunc lineLengthFunc)
{
    auto biggest = 1.0f;
    auto smallest = 0.0f;
    auto t = 0.5f;

    //just in case to prevent an infinite loop
    if (at <= 0) return 0.0f;
    if (at >= length) return 1.0f;

    while (true) {
        auto right = bz;
        Bezier left;
        right.split(t, left);
        length = _bezLength(left, lineLengthFunc);
        if (fabsf(length - at) < BEZIER_EPSILON || fabsf(smallest - biggest) < 1e-3f) {
            break;
        }
        if (length < at) {
            smallest = t;
            t = (t + biggest) * 0.5f;
        } else {
            biggest = t;
            t = (smallest + t) * 0.5f;
        }
    }
    return t;
}


/************************************************************************/
/* External Class Implementation                                        */
/************************************************************************/

namespace tvg {


uint8_t lerp(const uint8_t &start, const uint8_t &end, float t)
{
    return static_cast<uint8_t>(tvg::clamp(static_cast<int>(start + (end - start) * t), 0, 255));
}


float length(const PathCommand* cmds, uint32_t cmdsCnt, const Point* pts, uint32_t ptsCnt)
{
    if (ptsCnt < 2) return 0.0f;

    auto start = pts;
    auto length = 0.0f;

    while (cmdsCnt-- > 0) {
        switch (*cmds) {
            case PathCommand::Close: {
                length += tvg::length(*(pts - 1), *start);
                break;
            }
            case PathCommand::MoveTo: {
                start = pts;
                ++pts;
                break;
            }
            case PathCommand::LineTo: {
                length += tvg::length(*(pts - 1), *pts);
                ++pts;
                break;
            }
            case PathCommand::CubicTo: {
                length += Bezier{*(pts - 1), *pts, *(pts + 1), *(pts + 2)}.length();
                pts += 3;
                break;
            }
        }
        ++cmds;
    }

    return length;
}


//https://en.wikipedia.org/wiki/Remez_algorithm
float atan2(float y, float x)
{
    if (y == 0.0f && x == 0.0f) return 0.0f;
    auto a = std::min(fabsf(x), fabsf(y)) / std::max(fabsf(x), fabsf(y));
    auto s = a * a;
    auto r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (fabsf(y) > fabsf(x)) r = 1.57079637f - r;
    if (x < 0) r = 3.14159274f - r;
    if (y < 0) return -r;
    return r;
}


bool inverse(const Matrix* m, Matrix* out)
{
    auto det = m->e11 * (m->e22 * m->e33 - m->e32 * m->e23) -
               m->e12 * (m->e21 * m->e33 - m->e23 * m->e31) +
               m->e13 * (m->e21 * m->e32 - m->e22 * m->e31);

    auto invDet = 1.0f / det;
    if (std::isinf(invDet)) return false;

    out->e11 = (m->e22 * m->e33 - m->e32 * m->e23) * invDet;
    out->e12 = (m->e13 * m->e32 - m->e12 * m->e33) * invDet;
    out->e13 = (m->e12 * m->e23 - m->e13 * m->e22) * invDet;
    out->e21 = (m->e23 * m->e31 - m->e21 * m->e33) * invDet;
    out->e22 = (m->e11 * m->e33 - m->e13 * m->e31) * invDet;
    out->e23 = (m->e21 * m->e13 - m->e11 * m->e23) * invDet;
    out->e31 = (m->e21 * m->e32 - m->e31 * m->e22) * invDet;
    out->e32 = (m->e31 * m->e12 - m->e11 * m->e32) * invDet;
    out->e33 = (m->e11 * m->e22 - m->e21 * m->e12) * invDet;

    return true;
}


bool identity(const Matrix* m)
{
    if (m->e11 != 1.0f || m->e12 != 0.0f || m->e13 != 0.0f ||
        m->e21 != 0.0f || m->e22 != 1.0f || m->e23 != 0.0f ||
        m->e31 != 0.0f || m->e32 != 0.0f || m->e33 != 1.0f) {
        return false;
    }
    return true;
}


void rotate(Matrix* m, float degree)
{
    if (degree == 0.0f) return;

    auto radian = degree / 180.0f * MATH_PI;
    auto cosVal = cosf(radian);
    auto sinVal = sinf(radian);

    m->e12 = m->e11 * -sinVal;
    m->e11 *= cosVal;
    m->e21 = m->e22 * sinVal;
    m->e22 *= cosVal;
}


// ESP-DSP optimized 3x3 matrix multiplication
Matrix operator*(const Matrix& lhs, const Matrix& rhs)
{
    Matrix m;
    dspm_mult_3x3x3_f32_ae32(&lhs.e11, &rhs.e11, &m.e11);
    return m;
}


bool operator==(const Matrix& lhs, const Matrix& rhs)
{
    if (!tvg::equal(lhs.e11, rhs.e11) || !tvg::equal(lhs.e12, rhs.e12) || !tvg::equal(lhs.e13, rhs.e13) ||
        !tvg::equal(lhs.e21, rhs.e21) || !tvg::equal(lhs.e22, rhs.e22) || !tvg::equal(lhs.e23, rhs.e23) ||
        !tvg::equal(lhs.e31, rhs.e31) || !tvg::equal(lhs.e32, rhs.e32) || !tvg::equal(lhs.e33, rhs.e33)) {
       return false;
    }
    return true;
}


// ESP32-S3 FPU optimized point-matrix multiplication
// Uses madd.s (fused multiply-add) for better performance
void operator*=(Point& pt, const Matrix& m)
{
    float tx, ty;
    __asm__ __volatile__(
        "wfr f0, %2\n"           // f0 = pt.x
        "wfr f1, %3\n"           // f1 = pt.y
        "wfr f2, %4\n"           // f2 = m.e11
        "wfr f3, %5\n"           // f3 = m.e12
        "wfr f4, %6\n"           // f4 = m.e13
        "wfr f5, %7\n"           // f5 = m.e21
        "wfr f6, %8\n"           // f6 = m.e22
        "wfr f7, %9\n"           // f7 = m.e23
        // tx = pt.x * m.e11 + pt.y * m.e12 + m.e13
        "mul.s f8, f0, f2\n"     // f8 = pt.x * m.e11
        "madd.s f8, f1, f3\n"    // f8 += pt.y * m.e12
        "add.s f8, f8, f4\n"     // f8 += m.e13
        // ty = pt.x * m.e21 + pt.y * m.e22 + m.e23
        "mul.s f9, f0, f5\n"     // f9 = pt.x * m.e21
        "madd.s f9, f1, f6\n"    // f9 += pt.y * m.e22
        "add.s f9, f9, f7\n"     // f9 += m.e23
        "rfr %0, f8\n"           // tx = f8
        "rfr %1, f9\n"           // ty = f9
        : "=r"(tx), "=r"(ty)
        : "r"(pt.x), "r"(pt.y), "r"(m.e11), "r"(m.e12), "r"(m.e13),
          "r"(m.e21), "r"(m.e22), "r"(m.e23)
        : "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9"
    );
    pt.x = tx;
    pt.y = ty;
}


Point operator*(const Point& pt, const Matrix& m)
{
    float tx, ty;
    __asm__ __volatile__(
        "wfr f0, %2\n"           // f0 = pt.x
        "wfr f1, %3\n"           // f1 = pt.y
        "wfr f2, %4\n"           // f2 = m.e11
        "wfr f3, %5\n"           // f3 = m.e12
        "wfr f4, %6\n"           // f4 = m.e13
        "wfr f5, %7\n"           // f5 = m.e21
        "wfr f6, %8\n"           // f6 = m.e22
        "wfr f7, %9\n"           // f7 = m.e23
        // tx = pt.x * m.e11 + pt.y * m.e12 + m.e13
        "mul.s f8, f0, f2\n"     // f8 = pt.x * m.e11
        "madd.s f8, f1, f3\n"    // f8 += pt.y * m.e12
        "add.s f8, f8, f4\n"     // f8 += m.e13
        // ty = pt.x * m.e21 + pt.y * m.e22 + m.e23
        "mul.s f9, f0, f5\n"     // f9 = pt.x * m.e21
        "madd.s f9, f1, f6\n"    // f9 += pt.y * m.e22
        "add.s f9, f9, f7\n"     // f9 += m.e23
        "rfr %0, f8\n"           // tx = f8
        "rfr %1, f9\n"           // ty = f9
        : "=r"(tx), "=r"(ty)
        : "r"(pt.x), "r"(pt.y), "r"(m.e11), "r"(m.e12), "r"(m.e13),
          "r"(m.e21), "r"(m.e22), "r"(m.e23)
        : "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9"
    );
    return {tx, ty};
}


Point normal(const Point& p1, const Point& p2)
{
    auto dir = p2 - p1;
    auto len = length(dir);
    if (tvg::zero(len)) return {};

    auto unitDir = dir / len;
    return {-unitDir.y, unitDir.x};
}


void normalize(Point& pt)
{
    if (zero(pt)) return;       //prevent zero division
    auto ilength = 1.0f / sqrtf((pt.x * pt.x) + (pt.y * pt.y));
    pt.x *= ilength;
    pt.y *= ilength;
}


// ESP32-S3 optimized batch point transform
// Preloads matrix into FPU registers, processes points in a tight loop
void transformPoints(Point* pts, uint32_t count, const Matrix& m)
{
    if (count == 0) return;

    // Preload matrix elements into local vars for register allocation
    float m11 = m.e11, m12 = m.e12, m13 = m.e13;
    float m21 = m.e21, m22 = m.e22, m23 = m.e23;

    for (uint32_t i = 0; i < count; ++i) {
        float x = pts[i].x, y = pts[i].y;
        float tx, ty;
        __asm__ __volatile__(
            "wfr f0, %2\n"           // f0 = x
            "wfr f1, %3\n"           // f1 = y
            "wfr f2, %4\n"           // f2 = m11
            "wfr f3, %5\n"           // f3 = m12
            "wfr f4, %6\n"           // f4 = m13
            "wfr f5, %7\n"           // f5 = m21
            "wfr f6, %8\n"           // f6 = m22
            "wfr f7, %9\n"           // f7 = m23
            // tx = x * m11 + y * m12 + m13
            "mul.s f8, f0, f2\n"
            "madd.s f8, f1, f3\n"
            "add.s f8, f8, f4\n"
            // ty = x * m21 + y * m22 + m23
            "mul.s f9, f0, f5\n"
            "madd.s f9, f1, f6\n"
            "add.s f9, f9, f7\n"
            "rfr %0, f8\n"
            "rfr %1, f9\n"
            : "=r"(tx), "=r"(ty)
            : "r"(x), "r"(y), "r"(m11), "r"(m12), "r"(m13),
              "r"(m21), "r"(m22), "r"(m23)
            : "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9"
        );
        pts[i].x = tx;
        pts[i].y = ty;
    }
}


// ESP32-S3 optimized batch point interpolation with optional transform
// Fuses lerp + transform into a single pass for better cache efficiency
// out[i] = lerp(start[i], end[i], t) * transform (if transform != nullptr)
// or out[i] = lerp(start[i], end[i], t) (if transform == nullptr)
void lerpTransformPoints(Point* out, const Point* start, const Point* end, uint32_t count, float t, const Matrix* m)
{
    if (count == 0) return;

    if (m) {
        // Fused lerp + transform path
        float m11 = m->e11, m12 = m->e12, m13 = m->e13;
        float m21 = m->e21, m22 = m->e22, m23 = m->e23;

        for (uint32_t i = 0; i < count; ++i) {
            // Lerp first: p = start + (end - start) * t
            float px = start[i].x + (end[i].x - start[i].x) * t;
            float py = start[i].y + (end[i].y - start[i].y) * t;

            // Then transform: out = p * matrix
            float tx, ty;
            __asm__ __volatile__(
                "wfr f0, %2\n"           // f0 = px
                "wfr f1, %3\n"           // f1 = py
                "wfr f2, %4\n"           // f2 = m11
                "wfr f3, %5\n"           // f3 = m12
                "wfr f4, %6\n"           // f4 = m13
                "wfr f5, %7\n"           // f5 = m21
                "wfr f6, %8\n"           // f6 = m22
                "wfr f7, %9\n"           // f7 = m23
                "mul.s f8, f0, f2\n"     // f8 = px * m11
                "madd.s f8, f1, f3\n"    // f8 += py * m12
                "add.s f8, f8, f4\n"     // f8 += m13
                "mul.s f9, f0, f5\n"     // f9 = px * m21
                "madd.s f9, f1, f6\n"    // f9 += py * m22
                "add.s f9, f9, f7\n"     // f9 += m23
                "rfr %0, f8\n"
                "rfr %1, f9\n"
                : "=r"(tx), "=r"(ty)
                : "r"(px), "r"(py), "r"(m11), "r"(m12), "r"(m13),
                  "r"(m21), "r"(m22), "r"(m23)
                : "f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9"
            );
            out[i].x = tx;
            out[i].y = ty;
        }
    } else {
        // Lerp only path (no transform)
        for (uint32_t i = 0; i < count; ++i) {
            out[i].x = start[i].x + (end[i].x - start[i].x) * t;
            out[i].y = start[i].y + (end[i].y - start[i].y) * t;
        }
    }
}


float Line::length() const
{
    return _lineLength(pt1, pt2);
}


void Line::split(float at, Line& left, Line& right) const
{
    auto len = length();
    auto dx = ((pt2.x - pt1.x) / len) * at;
    auto dy = ((pt2.y - pt1.y) / len) * at;
    left.pt1 = pt1;
    left.pt2.x = left.pt1.x + dx;
    left.pt2.y = left.pt1.y + dy;
    right.pt1 = left.pt2;
    right.pt2 = pt2;
}


Bezier::Bezier(const Point& st, const Point& ed, float radius)
{
    // Calculate the angle between the start and end points
    auto angle = tvg::atan2(ed.y - st.y, ed.x - st.x);

    // Calculate the control points of the cubic bezier curve
    auto c = radius * PATH_KAPPA;  // c = radius * (4/3) * tan(pi/8)

    start = {st.x, st.y};
    ctrl1 = {st.x + radius * cos(angle), st.y + radius * sin(angle)};
    ctrl2 = {ed.x - c * cos(angle), ed.y - c * sin(angle)};
    end = {ed.x, ed.y};
}


void Bezier::split(Bezier& left, Bezier& right) const
{
    auto c = (ctrl1.x + ctrl2.x) * 0.5f;
    left.ctrl1.x = (start.x + ctrl1.x) * 0.5f;
    right.ctrl2.x = (ctrl2.x + end.x) * 0.5f;
    left.start.x = start.x;
    right.end.x = end.x;
    left.ctrl2.x = (left.ctrl1.x + c) * 0.5f;
    right.ctrl1.x = (right.ctrl2.x + c) * 0.5f;
    left.end.x = right.start.x = (left.ctrl2.x + right.ctrl1.x) * 0.5f;

    c = (ctrl1.y + ctrl2.y) * 0.5f;
    left.ctrl1.y = (start.y + ctrl1.y) * 0.5f;
    right.ctrl2.y = (ctrl2.y + end.y) * 0.5f;
    left.start.y = start.y;
    right.end.y = end.y;
    left.ctrl2.y = (left.ctrl1.y + c) * 0.5f;
    right.ctrl1.y = (right.ctrl2.y + c) * 0.5f;
    left.end.y = right.start.y = (left.ctrl2.y + right.ctrl1.y) * 0.5f;
}


void Bezier::split(float at, Bezier& left, Bezier& right) const
{
    right = *this;
    auto t = right.at(at, right.length());
    right.split(t, left);
}


float Bezier::length() const
{
    return _bezLength(*this, _lineLength);
}


float Bezier::lengthApprox() const
{
    return _bezLength(*this, _lineLengthApprox);
}


void Bezier::split(float t, Bezier& left)
{
    left.start = start;

    left.ctrl1.x = start.x + t * (ctrl1.x - start.x);
    left.ctrl1.y = start.y + t * (ctrl1.y - start.y);

    left.ctrl2.x = ctrl1.x + t * (ctrl2.x - ctrl1.x); //temporary holding spot
    left.ctrl2.y = ctrl1.y + t * (ctrl2.y - ctrl1.y); //temporary holding spot

    ctrl2.x = ctrl2.x + t * (end.x - ctrl2.x);
    ctrl2.y = ctrl2.y + t * (end.y - ctrl2.y);

    ctrl1.x = left.ctrl2.x + t * (ctrl2.x - left.ctrl2.x);
    ctrl1.y = left.ctrl2.y + t * (ctrl2.y - left.ctrl2.y);

    left.ctrl2.x = left.ctrl1.x + t * (left.ctrl2.x - left.ctrl1.x);
    left.ctrl2.y = left.ctrl1.y + t * (left.ctrl2.y - left.ctrl1.y);

    left.end.x = start.x = left.ctrl2.x + t * (ctrl1.x - left.ctrl2.x);
    left.end.y = start.y = left.ctrl2.y + t * (ctrl1.y - left.ctrl2.y);
}


float Bezier::at(float at, float length) const
{
    return _bezAt(*this, at, length, _lineLength);
}


float Bezier::atApprox(float at, float length) const
{
    return _bezAt(*this, at, length, _lineLengthApprox);
}


Point Bezier::at(float t) const
{
    Point cur;
    auto it = 1.0f - t;

    auto ax = start.x * it + ctrl1.x * t;
    auto bx = ctrl1.x * it + ctrl2.x * t;
    auto cx = ctrl2.x * it + end.x * t;
    ax = ax * it + bx * t;
    bx = bx * it + cx * t;
    cur.x = ax * it + bx * t;

    float ay = start.y * it + ctrl1.y * t;
    float by = ctrl1.y * it + ctrl2.y * t;
    float cy = ctrl2.y * it + end.y * t;
    ay = ay * it + by * t;
    by = by * it + cy * t;
    cur.y = ay * it + by * t;

    return cur;
}


float Bezier::angle(float t) const
{
    if (t < 0 || t > 1) return 0;

    //derivate
    // p'(t) = 3 * (-(1-2t+t^2) * p0 + (1 - 4 * t + 3 * t^2) * p1 + (2 * t - 3 *
    // t^2) * p2 + t^2 * p3)
    float mt = 1.0f - t;
    float d = t * t;
    float a = -mt * mt;
    float b = 1 - 4 * t + 3 * d;
    float c = 2 * t - 3 * d;

    Point pt ={a * start.x + b * ctrl1.x + c * ctrl2.x + d * end.x, a * start.y + b * ctrl1.y + c * ctrl2.y + d * end.y};
    pt.x *= 3;
    pt.y *= 3;

    return rad2deg(tvg::atan2(pt.y, pt.x));
}


void Bezier::bounds(BBox& box, const Point& start, const Point& ctrl1, const Point& ctrl2, const Point& end)
{
    if (box.min.x > start.x) box.min.x = start.x;
    if (box.min.y > start.y) box.min.y = start.y;
    if (box.min.x > end.x) box.min.x = end.x;
    if (box.min.y > end.y) box.min.y = end.y;

    if (box.max.x < start.x) box.max.x = start.x;
    if (box.max.y < start.y) box.max.y = start.y;
    if (box.max.x < end.x) box.max.x = end.x;
    if (box.max.y < end.y) box.max.y = end.y;

    //find x/y-direction extrema (solving derivative of Bezier curve)
    auto findMinMax = [&](float start, float ctrl1, float ctrl2, float end, float& min, float& max) -> void {
        auto a = -1.0f * start + 3.0f * ctrl1 - 3.0f * ctrl2 + end;
        auto b = start - 2.0f * ctrl1 + ctrl2;
        auto c = -1.0f * start + ctrl1;
        auto h = b * b - a * c;
        if (h <= 0.0f) return;
        h = sqrtf(h);
        float t[2] = {(-b - h) / a, (-b + h) / a};
        for (int i = 0; i < 2; ++i) {
            if (t[i] <= 0.0f || t[i] >= 1.0f) continue;
            auto s = 1.0f - t[i];
            auto q = s * s * s * start + 3.0f * s * s * t[i] * ctrl1 + 3.0f * s * t[i] * t[i] * ctrl2 + t[i] * t[i] * t[i] * end;
            if (q < min) min = q;
            if (q > max) max = q;
        }
    };

    findMinMax(start.x, ctrl1.x, ctrl2.x, end.x, box.min.x, box.max.x);
    findMinMax(start.y, ctrl1.y, ctrl2.y, end.y, box.min.y, box.max.y);
}


bool Bezier::flatten() const
{
    float diff1_x = fabsf((ctrl1.x * 3.f) - (start.x * 2.f) - end.x);
    float diff1_y = fabsf((ctrl1.y * 3.f) - (start.y * 2.f) - end.y);
    float diff2_x = fabsf((ctrl2.x * 3.f) - (end.x * 2.f) - start.x);
    float diff2_y = fabsf((ctrl2.y * 3.f) - (end.y * 2.f) - start.y);
    if (diff1_x < diff2_x) diff1_x = diff2_x;
    if (diff1_y < diff2_y) diff1_y = diff2_y;
    return (diff1_x + diff1_y <= 0.5f);
}


//TODO: Consider to use while() instead of recursive stack calls
uint32_t Bezier::segments() const
{
    if (flatten()) return 1;
    Bezier left, right;
    split(left, right);
    return left.segments() + right.segments();
}


Bezier Bezier::operator*(const Matrix& m)
{
    return Bezier{start * m, ctrl1* m, ctrl2 * m, end * m};
}

}
