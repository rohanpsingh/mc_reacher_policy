#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define HandReacher_DLLIMPORT __declspec(dllimport)
#  define HandReacher_DLLEXPORT __declspec(dllexport)
#  define HandReacher_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define HandReacher_DLLIMPORT __attribute__((visibility("default")))
#    define HandReacher_DLLEXPORT __attribute__((visibility("default")))
#    define HandReacher_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define HandReacher_DLLIMPORT
#    define HandReacher_DLLEXPORT
#    define HandReacher_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef HandReacher_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define HandReacher_DLLAPI
#  define HandReacher_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef HandReacher_EXPORTS
#    define HandReacher_DLLAPI HandReacher_DLLEXPORT
#  else
#    define HandReacher_DLLAPI HandReacher_DLLIMPORT
#  endif // HandReacher_EXPORTS
#  define HandReacher_LOCAL HandReacher_DLLLOCAL
#endif // HandReacher_STATIC
