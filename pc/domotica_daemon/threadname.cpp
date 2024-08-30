#include "threadname.hpp"

#if defined(_WIN32) || defined(_WIN64)
#include <processthreadsapi.h>
#include <stringapiset.h>
#define WSTRING_SIZE (80)

void setThreadName(string threadName) {
    wchar_t buff[WSTRING_SIZE];
    auto result = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, threadName.c_str(), -1, buff, WSTRING_SIZE);
    if (result > 0)
        SetThreadDescription(GetCurrentThread(), buff);
}
#else
#include <pthread.h>
void setThreadName(string threadName) {
    /*
       The thread name is a
       meaningful C language string, whose length is restricted to 16
       characters, including the terminating null byte ('\0').
     */

    int result = pthread_setname_np(pthread_self(), threadName.c_str());
    (void)(result);
}
#endif
