#include "manus_interface/manus_interface.h"
#include "ros_replacements/ros_time_repl.h"

#include <iostream>

NiryoOneManusInterface* mi;

void controlLoop() 
    {
        auto last_time = repl::time_now();
        auto current_time = repl::time_now();
        // ros::Duration elapsed_time;

        std::cout << mi->pos[0] << ", " << mi->pos[1] << ", " << mi->pos[2] << ", " << mi->pos[3] << ", " << mi->pos[4] << ", " << mi->pos[5] << ", " << mi->pos[6] << std::endl;

        std::string err;
        mi->comm->allowMotorsCalibrationToStart(1, err);

        std::cout << err << std::endl;

        while (mi->comm->isCalibrationInProgress());

        repl::sleep(5);

        mi->pos[3] = 1;
        mi->write();
        bool ok = true;
        while(ok) {

        }
    }

int main(int argc, char** argv) {
    mi = new NiryoOneManusInterface();
    mi->init();
    controlLoop();
}