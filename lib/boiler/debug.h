#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef TEST_NATIVE
#include <stdio.h>
#define TRACE(...) do { \
    fprintf(stderr, "%s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, __VA_ARGS__);                   \
    fputc('\n', stderr);                            \
} while(0)
#define DEBUG_LOG(...) TRACE(__VA_ARGS__)
#else
#include <ArduinoLog.h>

#define TRACE(FMT, ...) Log.trace(F(FMT "\n"), ##__VA_ARGS__);
#define DEBUG_LOG(...) do{} while(0)
#endif

#endif
