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
    const char *f = ("[WARNING]" + format + "\n").c_str();
    va_list arg;
    va_start(arg, format);
    vprintf(f, arg);
    va_end(arg);
}

void OUTPUT_ERROR(std::string format, ...) {
    const char *f = ("[ERROR]" + format + "\n").c_str();
    va_list arg;
    va_start(arg, format);
    vprintf(f, arg);
    va_end(arg);
}