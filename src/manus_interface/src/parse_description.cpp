#include "manus_interface/parse_description.h"

CJointDescription::CJointDescription(JOINT_TYPE type, double tx, double ty, double tz, double rr, double rp, double ry, double min, double max) {
    this->type = type;
    this->tx = tx;
    this->ty = ty;    
    this->tz = tz;    
    this->rr = rr;    
    this->rp = rp;    
    this->ry = ry;
    this->min = min;
    this->max = max;
}

JOINT_TYPE getJointType(std::string& type) {
    if (type == "rotation") return ROTATIONAL;
    else if (type == "translation") return LINEAR;
    else return TOOL;
}

void parse_description(std::string& filename, std::vector<CJointDescription>& jointsOut) {
    YAML::Node file = YAML::LoadFile(filename);

    const YAML::Node& joints = file["joints"];

    jointsOut.clear();
    
    for (const auto& joint : joints) {
        if (joint.IsDefined()) {
            const YAML::Node& transform = joint["transform"];
            std::string type = joint["type"].as<std::string>();
            double tx = transform["tx"].as<double>();
            double ty = transform["ty"].as<double>();
            double tz = transform["tz"].as<double>();
            double rr = transform["rr"].as<double>();
            double rp = transform["rp"].as<double>();
            double ry = transform["ry"].as<double>();
            double min = joint["min"].as<double>();
            double max = joint["max"].as<double>();
            jointsOut.push_back(CJointDescription(getJointType(type), tx, ty, tz, rr, rp, ry, min, max));
        }
    }
}