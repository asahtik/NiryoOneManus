#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <functional>
#include <stdlib.h>

#include "manus/manipulator.h"
#include "manus/files.h"

#include "yaml-cpp/yaml.h"

#include "manus_interface/manus_interface.h"
#include "utils/change_led.h"
#include "ros_replacements/ros_time_repl.h"
#include "ros_replacements/status_output.h"

const int BTN_PIN = 4;
volatile long noBtnPresses = 0;
volatile bool btnPressed = false;
void btnStateSwitchISR();

volatile bool shuttingDown = false;
void shutdown(bool shutdownSystem = false);

void rwCtrlLoop(std::shared_ptr<NiryoOneManusInterface> i);

std::shared_ptr<NiryoOneManusInterface> mi;

volatile int lastPlanSize = 0;

class NiryoOneManipulator : public Manipulator {
public:
    NiryoOneManipulator();
	~NiryoOneManipulator();

	virtual int size();
	virtual bool move(int joint, float position, float speed);

	virtual ManipulatorDescription describe();
	virtual ManipulatorState state();

	virtual void prepareNewGoal(bool begin_trajectory);
private:
    std::unique_ptr<std::thread> rwThread;
	ManipulatorDescription mDescription;
	ManipulatorState mState;

	void loadDescription();
};