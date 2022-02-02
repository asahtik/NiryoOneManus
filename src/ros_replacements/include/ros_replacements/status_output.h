#include <string>
#include <stdio.h>
#include <cstdarg>

#ifndef REPL_STATUS_H
#define REPL_STATUS_H

void format_string(const char*, ...);
template<typename... Args>
extern void OUTPUT_INFO(std::string, Args...);

template<typename... Args>
extern void OUTPUT_WARNING(std::string, Args...);

template<typename... Args>
extern void OUTPUT_ERROR(std::string, Args...);

#endif