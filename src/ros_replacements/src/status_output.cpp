#include "ros_replacements/status_output.h"

void OUTPUT_INFO(std::string format, ...) {
    std::string str = "[INFO] " + format + "\n";
    const char *f = str.c_str();
    va_list arg;
    va_start(arg, format);
    vprintf(f, arg);
    va_end(arg);
}

void OUTPUT_WARNING(std::string format, ...) {
    std::string str = "[WARNING] " + format + "\n";
    const char *f = str.c_str();
    va_list arg;
    va_start(arg, format);
    vprintf(f, arg);
    va_end(arg);
}

void OUTPUT_ERROR(std::string format, ...) {
    std::string str = "[ERROR] " + format + "\n";
    const char *f = str.c_str();
    va_list arg;
    va_start(arg, format);
    vprintf(f, arg);
    va_end(arg);
}