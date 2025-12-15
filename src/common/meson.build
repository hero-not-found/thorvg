if simd_type == 'esp32'
    math_source = 'tvgMathEsp32.cpp'
else
    math_source = 'tvgMath.cpp'
endif

source_file = [
   'tvgArray.h',
   'tvgCommon.h',
   'tvgColor.h',
   'tvgCompressor.h',
   'tvgInlist.h',
   'tvgLock.h',
   'tvgMath.h',
   'tvgStr.h',
   'tvgColor.cpp',
   'tvgCompressor.cpp',
   math_source,
   'tvgStr.cpp'
]

utils_dep = declare_dependency(
    include_directories : include_directories('.'),
    sources : source_file)
