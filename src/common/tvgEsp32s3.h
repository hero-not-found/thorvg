#ifndef _TVG_ESP32S3_H_
#define _TVG_ESP32S3_H_

#include <math.h>
#include "tvgCommon.h"

namespace tvg {

static inline bool esp32s3Affine(const Matrix& m)
{
#ifdef THORVG_ESP32S3_VECTOR_SUPPORT
    return m.e31 == 0.0f && m.e32 == 0.0f && m.e33 == 1.0f;
#else
    (void) m;
    return false;
#endif
}


static inline float esp32s3AffineX(const Point& pt, const Matrix& m)
{
    return fmaf(pt.y, m.e12, fmaf(pt.x, m.e11, m.e13));
}


static inline float esp32s3AffineY(const Point& pt, const Matrix& m)
{
    return fmaf(pt.y, m.e22, fmaf(pt.x, m.e21, m.e23));
}


static inline Point esp32s3TransformPoint(const Point& pt, const Matrix& m)
{
    return {esp32s3AffineX(pt, m), esp32s3AffineY(pt, m)};
}

}

#endif
