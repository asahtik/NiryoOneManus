#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

enum JOINT_TYPE { ROTATIONAL, LINEAR, TOOL };

class CJointDescription {
public:
    double tx, ty, tz;
    double rr, rp, ry;

    double min, max;

    JOINT_TYPE type;

    CJointDescription(JOINT_TYPE, double, double, double, double, double, double, double, double);
};

void parse_description(std::string, std::vector<CJointDescription>&);