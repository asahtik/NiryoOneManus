#ifndef NIRYO_TEST_MOTORS_H
#define NIRYO_TEST_MOTORS_H


#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>

// #include <urdf/model.h>

#include "ros_replacements/ros_time_repl.hpp"

class NiryoOneTestMotor {

    private:
        std::vector<double> pose_start{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        bool enable_test;
        int _n_joints = 6;
        std::vector<std::string>  _joint_names;
        std::vector<double>  _joint_upper_limits;
        std::vector<double>  _joint_lower_limits;
        std::vector<double>  _joint_has_position_limits;

        std::vector<double> _current_joint_pose;


    public:
        NiryoOneTestMotor();

        void callbackJointSate();
        
        bool getJointsLimits();

        bool runTest(int nb_loops);
        void stopTest();
        void startTrajectory();
        bool playTrajectory();

        // control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(std::vector<double> joint_positions);
        // actionlib::SimpleClientGoalState getState();

};
#endif
