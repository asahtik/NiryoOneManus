#include "manus/manipulator.h"
#include "manus/files.h"

#include "yaml-cpp/yaml.h"

class NiryoOneManipulator : public Manipulator {
public:
    NiryoOneManipulator(const string& device, const string& model, const string& servos);
	~NiryoOneManipulator();

	virtual int size();
	virtual bool move(int joint, float position, float speed);

	virtual ManipulatorDescription describe();
	virtual ManipulatorState state();
};

class NiryoOneManipulatorManager : public ManipulatorManager {
private:
    void step(bool force = false);
};