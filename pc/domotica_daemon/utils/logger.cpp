// SPDX-License-Identifier: MIT

#include "logger.hpp"

#include "time.hpp"
#include <cstring>


namespace utils::logger {
// template <size_t N> static consteval size_t compile_time_strlen(char const
// (&)[N]) { return N - 1; } const constexpr size_t FILE_SKIP =
// compile_time_strlen(__FILE__) - compile_time_strlen("utils/logger.cpp");
//// Note: FILE_SKIP is machine dependent

const char *get_label(LogLevel level) {
    switch (level) {
    case LogLevel::Warning:
        return "[WARNING] ";
    case LogLevel::Error:
        return "[ERROR]   ";
    case LogLevel::DebugLog:
        return "[DEBUG]   ";
    case LogLevel::TestLog:
        return "[TEST]    ";
    default:
        return "[LOG]     ";
    }
}

void log_impl2(LogLevel level, const char *file, int line, std::string_view msg) {
    printf("%s %s%16s:%-4d %s\n", getTimeString().c_str(), get_label(level), file, line, msg.data());

	static FILE * logfile = nullptr;
	if (!logfile) {
		logfile = fopen("domotica.log","a");
	}
	if (logfile) {
		fprintf(logfile, "%s %s%16s:%-4d %s\n", getTimeString().c_str(), get_label(level), file, line, msg.data());
	}

}
} // namespace utils::logger
