#include <memory>

#include "niryo_one_driver/niryo_one_communication.h"
#include "niryo_one_driver/rpi_diagnostics.h"
#include "ros_replacements/ros_time_repl.h"

enum class JointMotorState {IDLE, MOVING, ERROR};
class NiryoOneManusInterface {
public:
    NiryoOneManusInterface();
    void init();
    void read();
    void write();

    void openGripper(double position);

    // Call before setting new goal
    // If false add to next goal, else move
    void syncNextGoal(bool beginTrajectory);
    void calibrate();

    void shutdown();

    void activateTorque(bool on);

    double gripperPos = 1.0;
    double gripperVel = 1.0;
    double gripperEff = 0;
    double gripperCmd = 1.0;

    double pos[6] {0};
    double vel[6] {1.0};
    double cmd[6] {0};
    double eff[6] {0};
    JointMotorState state[6] {JointMotorState::IDLE};

    std::shared_ptr<NiryoOneCommunication> comm;
    std::unique_ptr<RpiDiagnostics> rpiDiagnostics;
};