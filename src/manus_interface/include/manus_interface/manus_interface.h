#include "niryo_one_driver/niryo_one_communication.h"
#include "ros_replacements/ros_time_repl.hpp"

class NiryoOneManusInterface {
public:
    NiryoOneManusInterface();
    void read();
    void write();

    double pos[6];
    double vel[6];

    std::shared_ptr<CommunicationBase> comm;
};