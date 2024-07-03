#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define PolytopeController_DLLIMPORT __declspec(dllimport)
#  define PolytopeController_DLLEXPORT __declspec(dllexport)
#  define PolytopeController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define PolytopeController_DLLIMPORT __attribute__((visibility("default")))
#    define PolytopeController_DLLEXPORT __attribute__((visibility("default")))
#    define PolytopeController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define PolytopeController_DLLIMPORT
#    define PolytopeController_DLLEXPORT
#    define PolytopeController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef PolytopeController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PolytopeController_DLLAPI
#  define PolytopeController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef PolytopeController_EXPORTS
#    define PolytopeController_DLLAPI PolytopeController_DLLEXPORT
#  else
#    define PolytopeController_DLLAPI PolytopeController_DLLIMPORT
#  endif // PolytopeController_EXPORTS
#  define PolytopeController_LOCAL PolytopeController_DLLLOCAL
#endif // PolytopeController_STATIC
