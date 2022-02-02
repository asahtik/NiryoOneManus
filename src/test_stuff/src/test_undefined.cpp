#include <iostream>
#include "ros_replacements/ros_time_repl.h"

int main() {
    std::cout << "ASD1" << std::endl;
    repl::sleep(2);
    std::cout << "ASD2" << std::endl;
}