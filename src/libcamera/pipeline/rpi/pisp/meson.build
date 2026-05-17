# SPDX-License-Identifier: CC0-1.0

libcamera_internal_sources += files([
    'ccmp_unwrap.cpp',
    'pisp.cpp',
    'qbc_remosaic.cpp',
    'raw_stats.cpp',
    'tuning_helpers.cpp',
])

librt = cc.find_library('rt', required : true)
libpisp_dep = dependency('libpisp', fallback : ['libpisp', 'libpisp_dep'])

libcamera_deps += [libpisp_dep, librt]

subdir('data')
