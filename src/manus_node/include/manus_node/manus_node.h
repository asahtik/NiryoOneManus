#include <vector>
#include <memory>

#include "manus/manipulator.h"
#include "manus/files.h"

#include "yaml-cpp/yaml.h"

#include "manus_interface/manus_interface.h"
#include "manus_interface/parse_description.h"
#include "utils/change_led.h"
#include "ros_replacements/ros_time_repl.h"
#include "ros_replacements/status_output.h"

const int BTN_PIN = 4;
bool calibrationRequested = false;
int btnPressState = 0;
void btnPressedISR();

class NiryoOneManipulator : public Manipulator {
public:
    NiryoOneManipulator(const NiryoOneManusInterface* mi, const string& device, const string& model, const string& servos);
	~NiryoOneManipulator();

	virtual int size();
	virtual bool move(int joint, float position, float speed);

	virtual ManipulatorDescription describe();
	virtual ManipulatorState state();
private:
	NiryoOneManusInterface* mi;
	std::vector<JointDescription> joints;
};

class NiryoOneManipulatorManager : public ManipulatorManager {
private:
    void step(bool force = false);
};