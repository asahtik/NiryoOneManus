#pragma once

#include <string>
#include <stdio.h>

template<typename... Args>
void format_string(const char *format, Args... args) {
    printf(format, args...);
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