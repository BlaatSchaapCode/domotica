// SPDX-License-Identifier: MIT

#include "logger.hpp"

#include <chrono>
#include <cstdio>
#include <format>

namespace utils::logger {

FILE *logfile = nullptr;
template <size_t N>
static consteval size_t compile_time_strlen(char const (&)[N]) {
  return N - 1;
}
const constexpr size_t FILE_SKIP =
    compile_time_strlen(__FILE__) - compile_time_strlen("utils/logger.cpp");
// Note: FILE_SKIP is machine dependant

const char *get_label(LogLevel level) {
  switch (level) {
  case LogLevel::Warning:
    return "[WARNING] ";
  case LogLevel::Error:
    return "[ERROR]   ";
  case LogLevel::DebugLog:
    return "[DBG_LOG] ";
  case LogLevel::TestLog:
    return "[TEST_LOG]";
  default:
    return "[LOG]     ";
  }
}

void log_impl2(LogLevel level, const char *file, int line,
               std::string_view msg) {

  const auto zt{std::chrono::zoned_time{std::chrono::current_zone(),
                                        std::chrono::system_clock::now()}};

  printf("[%s]%s%24s:%-4d%s\n", std::format("{:%FT%T%z}", zt).c_str(),
         get_label(level), file + FILE_SKIP, line, msg.data());

  // Simple logging to file
  if (!logfile)
    logfile = fopen("domotica.log", "a");
  if (logfile) {
    fprintf(logfile, "[%s]%s%24s:%-4d%s\n",
            std::format("{:%FT%T%z}", zt).c_str(), get_label(level),
            file + FILE_SKIP, line, msg.data());
    fflush(logfile);
  }
}
} // namespace utils::logger
