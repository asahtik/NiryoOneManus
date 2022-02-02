#include <string>
#include <stdio.h>
#include <cstdarg>

#ifndef REPL_STATUS_H
#define REPL_STATUS_H

extern void OUTPUT_INFO(std::string, ...);

extern void OUTPUT_WARNING(std::string, ...);

extern void OUTPUT_ERROR(std::string, ...);

#endif