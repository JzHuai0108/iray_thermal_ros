#pragma once
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#define MY_WIN32
#elif defined(linux) || defined(__linux) || defined(__linux__)
#define MY_LINUX
#endif

#ifdef MY_WIN32
#ifdef USBSDK_EXPORTS
#		ifdef __cplusplus
#			define USBSDK_API extern "C" __declspec(dllexport)
#		else
#			define USBSDK_API __declspec(dllexport)
#		endif
#else
#		ifdef __cplusplus
#			define USBSDK_API extern "C" __declspec(dllimport)
#		else
#			define USBSDK_API __declspec(dllimport)
#		endif
#endif
#elif defined MY_LINUX
#	ifdef __cplusplus
#		define USBSDK_API extern "C"
#	else
#		define USBSDK_API
#	endif
#endif

#ifdef MY_LINUX
typedef unsigned long       DWORD;
typedef void*				HWND;
#define WINAPI
#define __stdcall
#endif
