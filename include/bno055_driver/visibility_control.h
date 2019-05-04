#ifndef BNO055_DRIVER__VISIBILITY_CONTROL_H_
#define BNO055_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BNO055_DRIVER_EXPORT __attribute__ ((dllexport))
    #define BNO055_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define BNO055_DRIVER_EXPORT __declspec(dllexport)
    #define BNO055_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef BNO055_DRIVER_BUILDING_LIBRARY
    #define BNO055_DRIVER_PUBLIC BNO055_DRIVER_EXPORT
  #else
    #define BNO055_DRIVER_PUBLIC BNO055_DRIVER_IMPORT
  #endif
  #define BNO055_DRIVER_PUBLIC_TYPE BNO055_DRIVER_PUBLIC
  #define BNO055_DRIVER_LOCAL
#else
  #define BNO055_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define BNO055_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define BNO055_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define BNO055_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BNO055_DRIVER_PUBLIC
    #define BNO055_DRIVER_LOCAL
  #endif
  #define BNO055_DRIVER_PUBLIC_TYPE
#endif

#endif  // BNO055_DRIVER__VISIBILITY_CONTROL_H_
