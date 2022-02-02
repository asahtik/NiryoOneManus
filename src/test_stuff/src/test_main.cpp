#include "manus_interface/manus_interface.h"
#include "ros_replacements/ros_time_repl.h"
#include "ros_replacements/status_output.h"

#include <iostream>

NiryoOneManusInterface* mi;

void controlLoop() 
    {
        auto last_time = repl::time_now();
        auto current_time = repl::time_now();
        // ros::Duration elapsed_time;

        // std::cout << mi->pos[0] << ", " << mi->pos[1] << ", " << mi->pos[2] << ", " << mi->pos[3] << ", " << mi->pos[4] << ", " << mi->pos[5] << ", " << mi->pos[6] << std::endl;

        std::string err;
        mi->comm->allowMotorsCalibrationToStart(1, err);

        std::cout << err << std::endl;

        repl::sleep(1);

        while (mi->comm->isCalibrationInProgress()) repl::sleep(0.25);

        repl::sleep(1);

        mi->comm->activateLearningMode(false);

        repl::sleep(1);

        OUTPUT_INFO("Writing position");
        mi->cmd[0] = 1; mi->cmd[1] = 1; mi->cmd[2] = 1; mi->cmd[3] = 1; mi->cmd[4] = 1; mi->cmd[5] = 1; mi->cmd[6] = 1;
        for (int i = 0; i < 10000; ++i) {
            mi->read();
            mi->write();
            repl::sleep(0.01);
        }
        bool ok = true;
        repl::sleep(10);
    }

int main(int argc, char** argv) {
    mi = new NiryoOneManusInterface();
    mi->init();
    controlLoop();
}