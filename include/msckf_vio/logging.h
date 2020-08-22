#ifndef CG_LOGGING_H
#define CG_LOGGING_H

#define LOG_ON true

#if defined(__ANDROID__)

#include <android/log.h>

#define LOG_TAG "MSCKF"

#if LOG_ON
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL, LOG_TAG, __VA_ARGS__)
#else
#define LOGD(...)
#define LOGI(...)
#define LOGW(...)
#define LOGE(...)
#define LOGF(...)
#endif

#ifndef LOG_ROOT
#define LOG_ROOT "/sdcard/"
#endif

#else

#if defined(__linux__) || defined(_WIN32)

#include <stdarg.h>
#include <stdio.h>

#if LOG_ON
#define LOGI(...) printf(__VA_ARGS__)
#else
#define LOGI(...)
#endif

#ifndef LOG_ROOT
#define LOG_ROOT "/tmp/"
#endif

#endif

#endif

#endif // CG_LOGGING_H
