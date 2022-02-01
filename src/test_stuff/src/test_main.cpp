#include "manus_interface/manus_interface.h"
#include "ros_replacements/ros_time_repl.hpp"

#include <iostream>

NiryoOneManusInterface* mi;

int main(int argc, char** argv) {
    mi = new NiryoOneManusInterface();
    controlLoop();
}

void controlLoop() 
    {
        auto last_time = repl::time_now();
        auto current_time = repl::time_now();
        // ros::Duration elapsed_time;
        
        repl::sleep(5);

        mi->read();

        std::cout << mi->pos[0] << ", " << mi->pos[1] << ", " << mi->pos[2] << ", " << mi->pos[3] << ", " << mi->pos[4] << ", " << mi->pos[5] << ", " << mi->pos[6] << std::endl;

        bool ok = true;
        while(ok) {

        }
    }