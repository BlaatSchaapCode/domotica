#include "threadName.hpp"

#if defined(_WIN32) || defined(_WIN64)
// setThreadName for Windows
// https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-setthreaddescription
// Documentation states minimal version is Windows 10, version 1607
// Thus it might need some work to make something that can at least run as no-op
// on older versions
#include <processthreadsapi.h>
#include <stringapiset.h>
#define WSTRING_SIZE (80)

void setThreadName(std::string threadName) {
    wchar_t buff[WSTRING_SIZE];
    auto result = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, threadName.c_str(), -1, buff, WSTRING_SIZE);
    if (result > 0)
        SetThreadDescription(GetCurrentThread(), buff);
}
#elif __GNUC__
// setThreadName for the GNU C Library
#include <pthread.h>

void setThreadName(std::string threadName) {
    /*
       The thread name is a
       meaningful C language string, whose length is restricted to 16
       characters, including the terminating null byte ('\0').
     */

    int result = pthread_setname_np(pthread_self(), threadName.c_str());
    (void)(result);
}

#else
// Empty implementation for non-supported OS'es/ C Libraries
// TODO: add others, see
// https://stackoverflow.com/questions/2369738/how-to-set-the-name-of-a-thread-in-linux-pthreads
// for NetBSD, FreeBSD, OpenBSD
void setThreadName(std::string threadName) {}
#endif
