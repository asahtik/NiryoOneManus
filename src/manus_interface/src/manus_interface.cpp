#include "manus_interface/manus_interface.h"
#include "ros_replacements/status_output.hpp"

NiryoOneManusInterface::NiryoOneManusInterface() {
    comm.reset(new NiryoOneCommunication(2));
    int init_result = comm->init();
    if (init_result != 0) {
        OUTPUT_ERROR("Error initialising niryo communication");
        return;
    }
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
    comm->sendPositionToRobot(pos);
}