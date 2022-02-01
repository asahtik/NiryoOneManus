#pragma once

#include <string>
#include <stdio.h>
#include <cstdarg>

void format_string(const char *format, ...) {
    va_list arg;
    int done;
    va_start(arg, format);
    done = vfprintf(stdout, format, arg);
    va_end(arg);
}

template<typename... Args>
void OUTPUT_INFO(std::string format, Args... args) {
    const char *f = ("[INFO]" + format + "\n").c_str();
    format_string(f, args...);
}

template<typename... Args>
void OUTPUT_WARNING(std::string format, Args... args) {
    const char *f = ("[WARNING]" + format + "\n").c_str();
    format_string(f, args...);
}

template<typename... Args>
void OUTPUT_ERROR(std::string format, Args... args) {
    const char *f = ("[ERROR]" + format + "\n").c_str();
    format_string(f, args...);
}