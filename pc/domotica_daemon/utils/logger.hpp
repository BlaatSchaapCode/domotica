// SPDX-License-Identifier: MIT

#pragma once

#include <cstdio>
#include <string_view>

#ifndef __FILENAME__
#define __FILENAME__ "unknown"
// Are you using the correct Makefiles????
#endif

/*
 * How to use logging:
 * For logging we use printf like macro's.
 * Most of the logging should be used with LOG, WARNING and ERROR, but there are
 * also two special ones: DBG_LOG and TEST_LOG
 * - LOG:      Anything that is relevant when reading the logs but is not odd.
 * - WARNING:  Something odd but not nessesary a bug.
 * - ERROR:    A runtime detected problem.
 * - DBG_LOG:  A log message relevant when debugging certain subsystems. These
 * are usually disabled but can be enabled on a file basis.
 * - TEST_LOG: A log message specifically for tests. They will not compile in
 * release, forcing the user to remove it.
 */

#ifdef _MSC_FULL_VER
#define LOG_INFO(format, ...) utils::logger::log_impl(utils::logger::LogLevel::Info, __FILENAME__, __LINE__, format, __VA_ARGS__)
#define LOG_WARNING(format, ...)                                                                                                   \
    utils::logger::log_impl(utils::logger::LogLevel::Warning, __FILENAME__, __LINE__, format, __VA_ARGS__)
#define LOG_ERROR(format, ...) utils::logger::log_impl(utils::logger::LogLevel::Error, __FILENAME__, __LINE__, format, __VA_ARGS__)
#else
#define LOG_INFO(format, ...)                                                                                                      \
    utils::logger::log_impl(utils::logger::LogLevel::Info, __FILENAME__, __LINE__, format __VA_OPT__(, ) __VA_ARGS__)
#define LOG_WARNING(format, ...)                                                                                                   \
    utils::logger::log_impl(utils::logger::LogLevel::Warning, __FILENAME__, __LINE__, format __VA_OPT__(, ) __VA_ARGS__)
#define LOG_ERROR(format, ...)                                                                                                     \
    utils::logger::log_impl(utils::logger::LogLevel::Error, __FILENAME__, __LINE__, format __VA_OPT__(, ) __VA_ARGS__)
#endif

#ifdef ENABLE_LOG_DEBUG
#define LOG_DEBUG(format, ...)                                                                                                     \
    utils::logger::log_impl(utils::logger::LogLevel::DebugLog, __FILENAME__, __LINE__, format __VA_OPT__(, ) __VA_ARGS__)
#else
#define LOG_DEBUG(format, ...)
#endif

#ifdef _DEBUG
#define LOG_TEST(format, ...)                                                                                                      \
    utils::logger::log_impl(utils::logger::LogLevel::TestLog, __FILENAME__, __LINE__, format __VA_OPT__(, ) __VA_ARGS__)
#endif

namespace utils::logger {
enum class LogLevel {
    Info,
    TestLog,
    DebugLog,
    Warning,
    Error,
};

void log_impl2(LogLevel level, const char *file, int line, std::string_view msg);

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-security"
#endif
// Clang (version 13.0.1) does not play nice with
//		format __VA_OPT__(,)
// in the LOG_ macros when there are no parameters to the log
// That is exactly when __VA_OPT__(,) comes into play, to allow such
// But then clang will errourously generate a
//		error: format string is not a string literal (potentially
// insecure) [-Werror,-Wformat-security]
// error, even though the format is a literal, it apparently gets confused by
// the __VA_OPT__
inline void log_impl(LogLevel level, const char *file, int line, const char *format, auto... args) {
    const int MAX_BUFFER = 1024;
    char buffer[MAX_BUFFER];
    snprintf(buffer, MAX_BUFFER, format, args...);
    log_impl2(level, file, line, buffer);
}
#ifdef __clang__
#pragma clang diagnostic pop
#endif

} // namespace utils::logger
