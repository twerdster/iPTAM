cmake_minimum_required(VERSION 3.22)
project(iPTAMiOS LANGUAGES C CXX OBJCXX)
set(IPTAM_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(IPTAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(IPTAM_CORE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "PTAM core checkout used for this build")
set(IPTAM_ENGINE_VARIANT "current" CACHE STRING "current, upstream or mobile")
set_property(CACHE IPTAM_ENGINE_VARIANT PROPERTY STRINGS current upstream mobile)
set(IPTAM_BUNDLE_PREFIX "$ENV{IPTAM_BUNDLE_PREFIX}" CACHE STRING "App bundle namespace")
if(IPTAM_BUNDLE_PREFIX STREQUAL "")
  set(IPTAM_BUNDLE_PREFIX "org.example.iptam")
endif()
set(IPTAM_DEVELOPMENT_TEAM "$ENV{IPTAM_DEVELOPMENT_TEAM}" CACHE STRING "Apple signing team; empty for unsigned builds")
if(IPTAM_ENGINE_VARIANT STREQUAL "upstream")
  set(IPTAM_UPSTREAM_PORT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../PTAM-upstream-port" CACHE PATH "Pinned compatibility port")
  set(ORIGINAL_PTAM_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(ORIGINAL_PTAM_BUILD_DRIVER OFF CACHE BOOL "" FORCE)
  add_subdirectory("${IPTAM_UPSTREAM_PORT_DIR}" upstream)
  set(APP_TARGET PTAMUpstream)
  set(APP_BUNDLE "${IPTAM_BUNDLE_PREFIX}.PTAMUpstream")
  set(APP_CORE original_ptam_core)
elseif(IPTAM_ENGINE_VARIANT STREQUAL "current")
  set(IPTAM_MOBILE_PAPER OFF CACHE BOOL "" FORCE)
  add_subdirectory("${IPTAM_CORE_SOURCE_DIR}" core)
  set(APP_TARGET iPTAM)
  set(APP_BUNDLE "${IPTAM_BUNDLE_PREFIX}.iPTAM")
  set(APP_CORE iptam_core)
elseif(IPTAM_ENGINE_VARIANT STREQUAL "mobile")
  set(IPTAM_MOBILE_PAPER ON CACHE BOOL "" FORCE)
  add_subdirectory("${IPTAM_CORE_SOURCE_DIR}" core)
  set(APP_TARGET PTAMMobile)
  set(APP_BUNDLE "${IPTAM_BUNDLE_PREFIX}.PTAMMobile")
  set(APP_CORE iptam_core)
else()
  message(FATAL_ERROR "Unknown IPTAM_ENGINE_VARIANT")
endif()
configure_file(Info.plist "${CMAKE_CURRENT_BINARY_DIR}/AppInfo.plist" @ONLY)
add_executable(${APP_TARGET} MACOSX_BUNDLE App.mm)
if(IPTAM_ENGINE_VARIANT STREQUAL "upstream")
  target_compile_definitions(${APP_TARGET} PRIVATE IPTAM_UPSTREAM=1)
endif()
target_compile_features(${APP_TARGET} PRIVATE cxx_std_14)
target_compile_options(${APP_TARGET} PRIVATE -fobjc-arc)
target_link_libraries(${APP_TARGET} PRIVATE ${APP_CORE}
  "-framework UIKit" "-framework AVFoundation" "-framework CoreMedia"
  "-framework CoreVideo" "-framework CoreGraphics" "-framework ImageIO")
set_target_properties(${APP_TARGET} PROPERTIES
  MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_BINARY_DIR}/AppInfo.plist"
  XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER ${APP_BUNDLE}
  XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "${IPTAM_DEVELOPMENT_TEAM}"
  XCODE_ATTRIBUTE_CODE_SIGN_STYLE Automatic
  XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY 1
  XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET 16.0
  XCODE_ATTRIBUTE_CLANG_ENABLE_MODULES YES
  XCODE_ATTRIBUTE_ENABLE_BITCODE NO
  XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT dwarf-with-dsym)
