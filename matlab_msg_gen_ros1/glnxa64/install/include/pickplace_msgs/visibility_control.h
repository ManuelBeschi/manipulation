#ifndef PICKPLACE_MSGS__VISIBILITY_CONTROL_H_
#define PICKPLACE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PICKPLACE_MSGS_EXPORT __attribute__ ((dllexport))
    #define PICKPLACE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PICKPLACE_MSGS_EXPORT __declspec(dllexport)
    #define PICKPLACE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PICKPLACE_MSGS_BUILDING_LIBRARY
    #define PICKPLACE_MSGS_PUBLIC PICKPLACE_MSGS_EXPORT
  #else
    #define PICKPLACE_MSGS_PUBLIC PICKPLACE_MSGS_IMPORT
  #endif
  #define PICKPLACE_MSGS_PUBLIC_TYPE PICKPLACE_MSGS_PUBLIC
  #define PICKPLACE_MSGS_LOCAL
#else
  #define PICKPLACE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PICKPLACE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PICKPLACE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PICKPLACE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PICKPLACE_MSGS_PUBLIC
    #define PICKPLACE_MSGS_LOCAL
  #endif
  #define PICKPLACE_MSGS_PUBLIC_TYPE
#endif
#endif  // PICKPLACE_MSGS__VISIBILITY_CONTROL_H_
// Generated 14-Oct-2020 13:58:56
// Copyright 2019-2020 The MathWorks, Inc.
