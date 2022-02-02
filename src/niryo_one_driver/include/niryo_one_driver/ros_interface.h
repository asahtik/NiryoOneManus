/*
    ros_interface.h
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <vector>
#include <thread>

#include "niryo_one_driver/communication_base.h"
#include "niryo_one_driver/rpi_diagnostics.h"
#include "niryo_one_driver/change_hardware_version.h"
#include "niryo_one_driver/test_motors.h"

#include "ros_replacements/ros_time_repl.h"

class RosInterface {

    public:

        RosInterface(CommunicationBase* niryo_one_comm, RpiDiagnostics* rpi_diagnostics,
                bool *flag_reset_controllers, bool learning_mode_on, int hardware_version);

        void startServiceServers();
        void startPublishers();
        void startSubscribers();

    private:

        CommunicationBase* comm;
        RpiDiagnostics* rpi_diagnostics;
        // ros::NodeHandle nh_;

        NiryoOneTestMotor test_motor; 

        bool* flag_reset_controllers;
        int hardware_version;
        bool learning_mode_on;
        int calibration_needed;
        bool calibration_in_progress;
        bool last_connection_up_flag;
        int motor_test_status;

        std::string rpi_image_version;
        std::string ros_niryo_one_version;

        // publishers

        std::shared_ptr<std::thread> publish_hardware_status_thread;

        std::shared_ptr<std::thread> publish_software_version_thread;

        std::shared_ptr<std::thread> publish_learning_mode_thread;

        // publish methods

        void publishHardwareStatus();
        void publishSoftwareVersion();
        void publishLearningMode(); 

        // callbacks
        bool callbackTestMotors();

        bool callbackCalibrateMotors();
        bool callbackRequestNewCalibration();

        bool callbackActivateLearningMode();
        bool callbackActivateLeds();

        bool callbackPingAndSetDxlTool();

        bool callbackOpenGripper();
        bool callbackCloseGripper();

        bool callbackPullAirVacuumPump();
        bool callbackPushAirVacuumPump();

        bool callbackChangeHardwareVersion();

        bool callbackSendCustomDxlValue();

        bool callbackRebootMotors();

};

#endif
