#ifndef WAYPOINT_MANAGER_CPP__VISIBILITY_CONTROL_H_
#define WAYPOINT_MANAGER_CPP__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define WAYPOINT_MANAGER_CPP_EXPORT __attribute__((dllexport))
#define WAYPOINT_MANAGER_CPP_IMPORT __attribute__((dllimport))
#else
#define WAYPOINT_MANAGER_CPP_EXPORT __declspec(dllexport)
#define WAYPOINT_MANAGER_CPP_IMPORT __declspec(dllimport)
#endif
#ifdef WAYPOINT_MANAGER_CPP_BUILDING_LIBRARY
#define WAYPOINT_MANAGER_CPP_PUBLIC WAYPOINT_MANAGER_CPP_EXPORT
#else
#define WAYPOINT_MANAGER_CPP_PUBLIC WAYPOINT_MANAGER_CPP_IMPORT
#endif
#define WAYPOINT_MANAGER_CPP_PUBLIC_TYPE WAYPOINT_MANAGER_CPP_PUBLIC
#define WAYPOINT_MANAGER_CPP_LOCAL
#else
#define WAYPOINT_MANAGER_CPP_EXPORT __attribute__((visibility("default")))
#define WAYPOINT_MANAGER_CPP_IMPORT
#if __GNUC__ >= 4
#define WAYPOINT_MANAGER_CPP_PUBLIC __attribute__((visibility("default")))
#define WAYPOINT_MANAGER_CPP_LOCAL __attribute__((visibility("hidden")))
#else
#define WAYPOINT_MANAGER_CPP_PUBLIC
#define WAYPOINT_MANAGER_CPP_LOCAL
#endif
#define WAYPOINT_MANAGER_CPP_PUBLIC_TYPE
#endif

#endif  // WAYPOINT_MANAGER_CPP__VISIBILITY_CONTROL_H_