// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>

// well...
// https://stackoverflow.com/questions/55556200/convert-between-stdu8string-and-stdstring
// UTF-8 "support" in C++20 seems to be a bad joke.

#ifndef GIT_BRANCH
#define GIT_BRANCH "Unknown"
#endif

#ifndef GIT_COMMIT
#define GIT_COMMIT "Unknown"
#endif

namespace utils {

class Version {
  public:
// Note: Clang also defines GCC macros, therefore we check for clang first
#ifdef __clang__
    // https://clang.llvm.org/docs/LanguageExtensions.html#builtin-macros
    const uint32_t m_compiler_major = __clang_major__;
    const uint32_t m_compiler_minor = __clang_minor__;
    const uint32_t m_compiler_patch = __clang_patchlevel__;
    const std::string m_compiler_name = "C language family frontend for LLVM (Clang)";
    // __clang_version__	// string
#elif __GNUC__
    // https://gcc.gnu.org/onlinedocs/cpp/Common-Predefined-Macros.html
    const uint32_t m_compiler_major = __GNUC__;
    const uint32_t m_compiler_minor = __GNUC_MINOR__;
    const uint32_t m_compiler_patch = __GNUC_PATCHLEVEL__;
    const std::string m_compiler_name = "The GNU Compiler Collection (GCC)";
    // __VERSION__			// string
#elif _MSC_FULL_VER
    //  https://docs.microsoft.com/en-us/cpp/preprocessor/predefined-macros
    const uint32_t m_compiler_major = (_MSC_FULL_VER / 10000000);
    const uint32_t m_compiler_minor = ((_MSC_FULL_VER / 100000) % 100);
    const uint32_t m_compiler_patch = (_MSC_FULL_VER % 100000);
    const std::string m_compiler_name = "Microsoft C/C++ Optimizing Compiler (CL)";
#endif

    const std::string m_compiler_full_name = m_compiler_name + " " + std::to_string(m_compiler_major) + "." +
                                             std::to_string(m_compiler_minor) + "." + std::to_string(m_compiler_patch);

    const std::string m_git_branch = GIT_BRANCH;
    const std::string m_git_commit = GIT_COMMIT;
};
}; // namespace utils
