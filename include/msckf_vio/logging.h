#pragma once

#if defined(__ANDROID__)

#include <android/log.h>
#define LOG_TAG "LARVIO"  
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL, LOG_TAG, __VA_ARGS__)

#ifndef LOG_ROOT
#define LOG_ROOT "/sdcard/"
#endif

#else

#if defined(__linux__) || defined(_WIN32)

#include <stdarg.h>
#include <stdio.h>
#define LOGI(...) printf(__VA_ARGS__)

#ifndef LOG_ROOT
#define LOG_ROOT "/tmp/"
#endif

#endif

#endif