//
// Created by qkgautier on 10/30/15.
//

#ifndef ANDROID_LOGGING_H
#define ANDROID_LOGGING_H


#include <android/log.h>


#define  LOG_TAG    "InfiniTAM"

#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)


#define WRITE_LOG(fn, ...) do { \
  FILE *f = fopen(fn, "a+"); \
  fprintf(f, __VA_ARGS__); \
  fflush(f); \
  fclose(f); \
} while (0)



#endif //ANDROID_LOGGING_H
