#pragma once

#if defined _WIN32 || defined __CYGWIN__
#define DemoController_DLLIMPORT __declspec(dllimport)
#define DemoController_DLLEXPORT __declspec(dllexport)
#define DemoController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#if __GNUC__ >= 4
#define DemoController_DLLIMPORT __attribute__((visibility("default")))
#define DemoController_DLLEXPORT __attribute__((visibility("default")))
#define DemoController_DLLLOCAL __attribute__((visibility("hidden")))
#else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#define DemoController_DLLIMPORT
#define DemoController_DLLEXPORT
#define DemoController_DLLLOCAL
#endif // __GNUC__ >= 4
#endif     // defined _WIN32 || defined __CYGWIN__

#ifdef DemoController_STATIC
// If one is using the library statically, get rid of
// extra information.
#define DemoController_DLLAPI
#define DemoController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#ifdef DemoController_EXPORTS
#define DemoController_DLLAPI DemoController_DLLEXPORT
#else
#define DemoController_DLLAPI DemoController_DLLIMPORT
#endif // DemoController_EXPORTS
#define DemoController_LOCAL DemoController_DLLLOCAL
#endif // DemoController_STATIC