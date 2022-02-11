#include <memory>

#include "niryo_one_driver/niryo_one_communication.h"
#include "niryo_one_driver/rpi_diagnostics.h"
#include "ros_replacements/ros_time_repl.h"

class NiryoOneManusInterface {
public:
    NiryoOneManusInterface();
    void init();
    void read();
    void write();
    // Call before setting new goal
    void syncNextGoal(bool beginTrajectory);
    void calibrate();

    void shutdown();

    double pos[6] {0};
    double vel[6] {0};
    double cmd[6] {0};
    double eff[6] {0};

    std::shared_ptr<NiryoOneCommunication> comm;
    std::unique_ptr<RpiDiagnostics> rpiDiagnostics;
};