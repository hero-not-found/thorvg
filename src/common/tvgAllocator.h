/*
 * Copyright (c) 2026 ThorVG project. All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _TVG_ALLOCATOR_H_
#define _TVG_ALLOCATOR_H_

#include <cstddef>
#include <cstdlib>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

// separate memory allocators for clean customization
namespace tvg {
template <typename T = void> static inline T* malloc(size_t size) {
#ifdef ESP_PLATFORM
  // Try SPIRAM first for large allocations on ESP32
  auto ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ptr) {
    // Fallback to internal RAM if SPIRAM allocation fails
    ptr = std::malloc(size);
  }
  return reinterpret_cast<T*>(ptr);
#else
  return reinterpret_cast<T*>(std::malloc(size));
#endif
}

template <typename T = void> static inline T* calloc(size_t nmem, size_t size) {
#ifdef ESP_PLATFORM
  // Try SPIRAM first for large allocations on ESP32
  auto ptr = heap_caps_calloc(nmem, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ptr) {
    // Fallback to internal RAM if SPIRAM allocation fails
    ptr = std::calloc(nmem, size);
  }
  return reinterpret_cast<T*>(ptr);
#else
  return reinterpret_cast<T*>(std::calloc(nmem, size));
#endif
}

template <typename T = void> static inline T* realloc(T* ptr, size_t size) {
#ifdef ESP_PLATFORM
  // For ESP32, use heap_caps_realloc if the pointer was allocated with
  // heap_caps
  auto new_ptr =
      heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!new_ptr && size > 0) {
    // Fallback to standard realloc
    new_ptr = std::realloc(ptr, size);
  }
  return reinterpret_cast<T*>(new_ptr);
#else
  return reinterpret_cast<T*>(std::realloc(ptr, size));
#endif
}

template <typename T = void> static inline void free(T* ptr) { std::free(ptr); }
} // namespace tvg

#endif //_TVG_ALLOCATOR_H_