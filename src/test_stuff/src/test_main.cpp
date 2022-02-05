#include "manus_interface/manus_interface.h"
#include "ros_replacements/ros_time_repl.h"
#include "ros_replacements/status_output.h"

#include <iostream>

NiryoOneManusInterface* mi;

void controlLoop() {
    // std::cout << mi->pos[0] << ", " << mi->pos[1] << ", " << mi->pos[2] << ", " << mi->pos[3] << ", " << mi->pos[4] << ", " << mi->pos[5] << ", " << mi->pos[6] << std::endl;
    
    mi->comm->requestNewCalibration();

    repl::sleep(1);

    std::string err;
    mi->comm->allowMotorsCalibrationToStart(1, err);
    OUTPUT_WARNING("%s", err.c_str());

    repl::sleep(1);
    // if (!mi->comm->isCalibrationInProgress()) mi->comm->requestNewCalibration();
    while (mi->comm->isCalibrationInProgress()) repl::sleep(0.25);

    repl::sleep(1);

    mi->comm->activateLearningMode(false);

    repl::sleep(1);

    OUTPUT_INFO("Writing position");
    mi->cmd[0] = 1; mi->cmd[1] = 1; mi->cmd[2] = 1; mi->cmd[3] = 1; mi->cmd[4] = 1; mi->cmd[5] = 1; mi->cmd[6] = 1;        
    mi->write();

    repl::sleep(10);

    mi->cmd[0] = 0; mi->cmd[1] = 0; mi->cmd[2] = 0; mi->cmd[3] = 0; mi->cmd[4] = 0; mi->cmd[5] = 0; mi->cmd[6] = 0;
    mi->write();

    repl::sleep(10);

    mi->comm->activateLearningMode(true);

    repl::sleep(10);
}

int main(int argc, char** argv) {
    mi = new NiryoOneManusInterface();
    mi->init();
    controlLoop();
}