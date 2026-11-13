cmake_minimum_required(VERSION 3.18)
project(OriginalPTAM LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(ORIGINAL_PTAM_BUILD_TOOLS "Build source-paced recorded host driver" ON)
option(ORIGINAL_PTAM_BUILD_DRIVER "Build source-paced recorded host driver (iOS: OFF)" ${ORIGINAL_PTAM_BUILD_TOOLS})
set(DEP ${CMAKE_CURRENT_SOURCE_DIR}/generated-deps)
add_library(original_gvars STATIC
  ${DEP}/gvars/src/gvars3.cc ${DEP}/gvars/src/GUI.cc ${DEP}/gvars/src/GStringUtil.cc
  ${DEP}/gvars/src/gvars2.cc ${DEP}/gvars/src/GUI_language.cc ${DEP}/gvars/src/serialize.cc
  ${DEP}/gvars/src/GUI_none.cc ${DEP}/gvars/src/GUI_impl_headless.cc ${DEP}/gvars/src/inst.cc)
target_include_directories(original_gvars PUBLIC ${DEP}/gvars ${DEP})
set(CVD ${DEP}/libcvd)
file(GLOB FAST_SOURCES ${CVD}/cvd_src/fast/*.cxx)
add_library(original_cvd STATIC
  ${CVD}/cvd_src/exceptions.cc ${CVD}/cvd_src/half_sample.cc
  ${CVD}/cvd_src/convolution.cc ${CVD}/cvd_src/cvd_timer.cc
  ${CVD}/cvd_src/fast_corner.cxx ${CVD}/cvd_src/nonmax_suppression.cxx
  ${CVD}/cvd_src/slower_corner_10.cxx ${CVD}/cvd_src/thread.cpp
  ${CVD}/cvd_src/noarch/convolve_gaussian.cc ${CVD}/cvd_src/noarch/gradient.cc
  ${CVD}/cvd_src/noarch/utility_byte_differences.cc ${CVD}/cvd_src/noarch/utility_float.cc
  ${CVD}/cvd_src/noarch/utility_double_int.cc ${CVD}/cvd_src/noarch/posix_memalign.cpp
  ${FAST_SOURCES})
target_include_directories(original_cvd PUBLIC ${CVD} ${DEP})
set(NAMES ATANCamera Bundle HomographyInit KeyFrame Map MapMaker MapPoint MiniPatch PatchFinder Relocaliser ShiTomasi SmallBlurryImage Tracker)
foreach(name IN LISTS NAMES)
  list(APPEND ORIGINAL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/generated/${name}.cc)
endforeach()
add_library(original_ptam_core STATIC ${ORIGINAL_SOURCES} adapter/OriginalPTAM.cc)
target_include_directories(original_ptam_core PUBLIC adapter PRIVATE generated compat)
target_link_libraries(original_ptam_core PUBLIC original_cvd original_gvars)
foreach(target original_gvars original_cvd original_ptam_core)
  target_compile_options(${target} PRIVATE -Wno-deprecated-declarations -Wno-unused-value)
endforeach()
if(APPLE)
  target_link_libraries(original_ptam_core PUBLIC "-framework Accelerate")
else()
  find_package(LAPACK REQUIRED)
  find_package(Threads REQUIRED)
  target_link_libraries(original_ptam_core PUBLIC ${LAPACK_LIBRARIES} Threads::Threads)
endif()
if(ORIGINAL_PTAM_BUILD_TOOLS AND ORIGINAL_PTAM_BUILD_DRIVER)
  add_executable(original_ptam_replay tools/replay.cc)
  target_link_libraries(original_ptam_replay PRIVATE original_ptam_core)
endif()
