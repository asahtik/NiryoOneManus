#include "manus_interface/manus_interface.h"
#include "ros_replacements/status_output.h"

NiryoOneManusInterface::NiryoOneManusInterface() {
    comm.reset(new NiryoOneCommunication());
    int init_result = comm->init();
    if (init_result != 0) {
        OUTPUT_ERROR("Error initialising niryo communication");
        return;
    }
}

void NiryoOneManusInterface::init() {
    comm->scanAndCheckMotors();
    repl::sleep(0.1);
    comm->manageHardwareConnection();
    repl::sleep(0.8);
    comm->activateLearningMode(true);
    repl::sleep(0.2);
}

void NiryoOneManusInterface::read() {
    double pos_to_read[6] = {0.0};
    
    comm->getCurrentPosition(pos_to_read);
   
    pos[0] = pos_to_read[0];
    pos[1] = pos_to_read[1];
    pos[2] = pos_to_read[2];
    pos[3] = pos_to_read[3];
    pos[4] = pos_to_read[4];
    pos[5] = pos_to_read[5];
}

void NiryoOneManusInterface::write() {
    // TODO: velocity not yet implemented on niryo's side
    comm->sendPositionToRobot(cmd);
}

void NiryoOneManusInterface::syncNextGoal(bool beginTrajectory) {
    comm->synchronizeMotors(beginTrajectory);
}

void NiryoOneManusInterface::calibrate() {
    comm->requestNewCalibration();

    repl::sleep(1);

    std::string err;
    comm->allowMotorsCalibrationToStart(1, err);
    OUTPUT_WARNING("%s", err.c_str());

    repl::sleep(1);
    // if (!mi->comm->isCalibrationInProgress()) mi->comm->requestNewCalibration();
    while (comm->isCalibrationInProgress()) repl::sleep(0.25);

    repl::sleep(1);

    comm->activateLearningMode(false);

    repl::sleep(1);

    OUTPUT_INFO("Torque enabled");

};