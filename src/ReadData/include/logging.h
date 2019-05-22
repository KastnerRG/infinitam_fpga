
#ifndef CUDAAPP_LOGGING_H
#define CUDAAPP_LOGGING_H



#define  LOGE(...)  { printf(__VA_ARGS__); printf("\n"); } 
#define  LOGW(...)  { printf(__VA_ARGS__); printf("\n"); } 
#define  LOGD(...)  { printf(__VA_ARGS__); printf("\n"); } 
#define  LOGI(...)  { printf(__VA_ARGS__); printf("\n"); } 


#define WRITE_LOG(fn, ...) do { \
  FILE *f = fopen(fn, "a+"); \
  fprintf(f, __VA_ARGS__); \
  fflush(f); \
  fclose(f); \
} while (0)



#endif //CUDAAPP_LOGGING_H


