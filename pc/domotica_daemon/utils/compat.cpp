
#if defined __clang__ && defined _WIN32
/*
    This fixes a linker error when (cross-)compiling for MINGW using clang
    /usr/bin/x86_64-w64-mingw32-ld:
   bld/clang/mingw/x86_64/debug/src/main.cpp.o:main.cpp:(.rdata$.refptr.__libc_single_threaded[.refptr.__libc_single_threaded]+0x0):
   undefined reference to `__libc_single_threaded'

    /usr/bin/x86_64-w64-mingw32-ld: bld/clang/mingw/x86_64/debug/src/utils/compat.cpp.o:compat.cpp:(.debug_info+0x2b): undefined
   reference to `__libc_single_threaded' /usr/bin/x86_64-w64-mingw32-ld:
   bld/clang/mingw/x86_64/debug/src/main.cpp.o:main.cpp:(.rdata$.refptr.__libc_single_threaded[.refptr.__libc_single_threaded]+0x0):
   undefined reference to `__libc_single_threaded' clang-13: error: linker command failed with exit code 1 (use -v to see
   invocation)


    ld.lld: error: undefined symbol: __libc_single_threaded
    >>> referenced by bld/clang/mingw/x86_64/debug/src/main.cpp.o:(.refptr.__libc_single_threaded)
*/
#include <cstdbool>
extern "C" {
bool __libc_single_threaded = false;
}
#endif