/*
    niryo_one_communication.cpp
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

#include "niryo_one_driver/niryo_one_communication.h"
#include "ros_replacements/status_output.h"

#include <iostream>

NiryoOneCommunication::NiryoOneCommunication()
{
    // TODO: get params
    // ros::param::get("~can_enabled", can_enabled);
    can_enabled = true;
    // ros::param::get("~dxl_enabled", dxl_enabled);
    dxl_enabled = true;
    // ros::param::get("~niryo_one_hw_check_connection_frequency", niryo_one_hw_check_connection_frequency);
    niryo_one_hw_check_connection_frequency = 2.0;

    if (!can_enabled) {
        OUTPUT_WARNING("CAN communication is disabled for debug purposes");
    }
    if (!dxl_enabled) {
        OUTPUT_WARNING("DXL communication is disabled for debug purposes");
    }

    if (can_enabled) {
        canComm.reset(new CanCommunication());
    }

    if (dxl_enabled) {
        dxlComm.reset(new DxlCommunication());
    }

    new_calibration_requested = false;
    niryo_one_comm_ok = false;
    can_comm_ok = false;
    dxl_comm_ok = false;
}

int NiryoOneCommunication::init()
{
    int result = 0;
    if (can_enabled) {
        result = canComm->init();
        if (result != 0) {
            return result;
        }
        OUTPUT_INFO("Initialised can");
    }
    if (dxl_enabled) {
        result =  dxlComm->init();
        if (result != 0) {
            return result;
        }
        OUTPUT_INFO("Initialised dxl");
    }
    return result;
}

bool NiryoOneCommunication::scanAndCheckMotors()
{
    bool result = true;
    if (can_enabled) {
        result = ((canComm->scanAndCheck() == CAN_SCAN_OK) && result); 
    }
    if (dxl_enabled) {
        result = ((dxlComm->scanAndCheck() == DXL_SCAN_OK) && result);
    }
    niryo_one_comm_ok = result;
    return result;
}

bool NiryoOneCommunication::isConnectionOk()
{
    bool result = true;
    if (can_enabled) {
        result = ((canComm->isConnectionOk()) && result);
    }
    if (dxl_enabled) {
        result = ((dxlComm->isConnectionOk()) && result);
    }
    niryo_one_comm_ok = result;
    return result;
}

void NiryoOneCommunication::manageCanConnectionLoop()
{
    if (!can_enabled) { 
        return; 
    }
    bool ok = true;

    bool motors_ok = false;
    repl::Rate rate(niryo_one_hw_check_connection_frequency);
    while (ok) {
        if (!canComm->isConnectionOk() || new_calibration_requested) {
            new_calibration_requested = false;
            OUTPUT_WARNING("Stop Can hw control");
            canComm->stopHardwareControlLoop();
            repl::sleep(0.1);
           
            while (canComm->scanAndCheck() != CAN_SCAN_OK) { // wait for connection to be up
                OUTPUT_WARNING("Scan to find stepper motors...");
                repl::sleep(0.25);
            }
            
            // once connected, set calibration flag
            OUTPUT_INFO("Set calibration flag");
            canComm->setCalibrationFlag(true);
                
            // deactivate motors (?)
            canComm->setTorqueOn(false);
    
            canComm->startHardwareControlLoop(true); // limited mode
            motors_ok = false;

            while (!motors_ok) {
                int calibration_step1_result = CAN_STEPPERS_CALIBRATION_FAIL;
                int calibration_step2_result = CAN_STEPPERS_CALIBRATION_FAIL;
                
                calibration_step1_result = canComm->calibrateMotors(1);
                if (calibration_step1_result == CAN_STEPPERS_CALIBRATION_OK) {
                    if (dxl_enabled) {
                        if (canComm->getCalibrationMode() == CAN_STEPPERS_CALIBRATION_MODE_AUTO) {
                            OUTPUT_INFO("Asking Dynamixel motors to go to home position");
                            dxlComm->moveAllMotorsToHomePosition();
                        }
                    }
                    calibration_step2_result = canComm->calibrateMotors(2);
                }

                if ((calibration_step1_result == CAN_STEPPERS_CALIBRATION_OK) 
                        && (calibration_step2_result == CAN_STEPPERS_CALIBRATION_OK)) {
                    motors_ok = true;
                    new_calibration_requested = false;
                    activateLearningMode(true);
                }
                else { // if calibration is not ok, wait and retry 
                    // check if connection is still ok
                    if (!canComm->isConnectionOk()) {
                        while (canComm->scanAndCheck() != CAN_SCAN_OK) { // wait for connection to be up
                            OUTPUT_WARNING("Scan to find stepper motors...");
                            repl::sleep(0.25);
                        }
                    }

                    // last calibration has failed, reset flag
                    if (calibration_step1_result != CAN_STEPPERS_CALIBRATION_WAITING_USER_INPUT) {
                        canComm->setCalibrationFlag(true);
                        // go back to limited mode (during calibration, hw control loop is stopped)
                        canComm->startHardwareControlLoop(true); 
                    }
                    
                    repl::sleep(0.25);
                }
            }

            OUTPUT_WARNING("Resume can hw control");
            activateLearningMode(true);
            if (dxl_enabled) {
                canComm->startHardwareControlLoop(!dxlComm->isConnectionOk());
            }
            else {
                canComm->startHardwareControlLoop(false);
            }
        }
        else { // can connection ok + calibrated
            if (dxl_enabled && !dxlComm->isConnectionOk()) {
                if (!canComm->isOnLimitedMode()) {
                    canComm->startHardwareControlLoop(true);
                }
            }
            else {
                if (canComm->isOnLimitedMode()) {
                    canComm->setTorqueOn(false);
                    canComm->startHardwareControlLoop(false);
                }
            }
        }
        rate.sleep();
    }
}

void NiryoOneCommunication::manageDxlConnectionLoop()
{
    if (!dxl_enabled) {
        return;
    }

    bool ok = true;

    repl::Rate rate(niryo_one_hw_check_connection_frequency);
    while (ok) {
        if (!dxlComm->isConnectionOk()) {
            OUTPUT_WARNING("Stop Dxl hw control");
            dxlComm->stopHardwareControlLoop();
            repl::sleep(0.1);

            while (dxlComm->scanAndCheck() != DXL_SCAN_OK) { // wait for connection to be up
                // output warning
                // ROS_WARN("Scan to find Dxl motors");
                repl::sleep(0.25);
            }

            OUTPUT_WARNING("Resume Dxl hw control");
            dxlComm->setTorqueOn(false);
            activateLearningMode(true);
            if (can_enabled) {
                dxlComm->startHardwareControlLoop(!canComm->isConnectionOk());
            }
            else {
                dxlComm->startHardwareControlLoop(false);
            }
        }
        else { // dxl connection ok
            if (can_enabled && !canComm->isConnectionOk()) {
                if (!dxlComm->isOnLimitedMode()) {
                    dxlComm->startHardwareControlLoop(true);
                }
            }
            else {
                if (dxlComm->isOnLimitedMode()) {
                    dxlComm->setTorqueOn(false);
                    dxlComm->startHardwareControlLoop(false);
                }
            }
        }
        rate.sleep();
    }
}

void NiryoOneCommunication::manageHardwareConnection()
{
    can_connection_loop_thread.reset(new std::thread(std::bind(&NiryoOneCommunication::manageCanConnectionLoop, this)));
    dxl_connection_loop_thread.reset(new std::thread(std::bind(&NiryoOneCommunication::manageDxlConnectionLoop, this)));
}

void NiryoOneCommunication::startHardwareControlLoop()
{
    if (can_enabled) { canComm->startHardwareControlLoop(false); }
    if (dxl_enabled) { dxlComm->startHardwareControlLoop(false); }
}

void NiryoOneCommunication::stopHardwareControlLoop()
{
    if (can_enabled) { canComm->stopHardwareControlLoop(); }
    if (dxl_enabled) { dxlComm->stopHardwareControlLoop(); }
}

void NiryoOneCommunication::resumeHardwareControlLoop()
{
    if (can_enabled) { canComm->startHardwareControlLoop(false); }
    if (dxl_enabled) { dxlComm->startHardwareControlLoop(false); }
}

void NiryoOneCommunication::synchronizeMotors(bool begin_traj)
{
    if (can_enabled) {
        canComm->synchronizeSteppers(begin_traj);
    }
}

int NiryoOneCommunication::allowMotorsCalibrationToStart(int mode, std::string &result_message)
{
    if (can_enabled) {
        if (mode == CAN_STEPPERS_CALIBRATION_MODE_MANUAL) {
            if (!canComm->canProcessManualCalibration(result_message)) {
                return 400;
            }
        }
        canComm->validateMotorsCalibrationFromUserInput(mode);
    }
    if (dxl_enabled) {
        // todo check dxl in bounds
    }

    result_message = "Calibration is starting";
    return 200;
}

void NiryoOneCommunication::requestNewCalibration()
{
    new_calibration_requested = true;
}

bool NiryoOneCommunication::isCalibrationInProgress()
{
    if (can_enabled) {
        return canComm->isCalibrationInProgress();
    }
    return false;
}

void NiryoOneCommunication::getHardwareStatus(bool *is_connection_ok, std::string &error_message,
        int *calibration_needed, bool *calibration_in_progress,
        std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
        std::vector<int32_t> &temperatures, std::vector<double> &voltages,
        std::vector<int32_t> &hw_errors)
{
    bool can_connection_ok = !can_enabled; // if CAN disabled, declare connection ok
    int can_calibration_needed = 0;
    bool can_calibration_in_progress = false;
    std::string can_error_message = "";
    std::vector<std::string> can_motor_names;
    std::vector<std::string> can_motor_types;
    std::vector<int32_t> can_temperatures;
    std::vector<double> can_voltages;
    std::vector<int32_t> can_hw_errors;

    bool dxl_connection_ok = !dxl_enabled; // if Dxl disabled, declare connection ok
    int dxl_calibration_needed = 0;
    bool dxl_calibration_in_progress = false;
    std::string dxl_error_message = "";
    std::vector<std::string> dxl_motor_names;
    std::vector<std::string> dxl_motor_types;
    std::vector<int32_t> dxl_temperatures;
    std::vector<double> dxl_voltages;
    std::vector<int32_t> dxl_hw_errors;

    if (can_enabled) {
        canComm->getHardwareStatus(&can_connection_ok, can_error_message, &can_calibration_needed,
                &can_calibration_in_progress, can_motor_names, can_motor_types, can_temperatures, can_voltages, can_hw_errors);
    }
    if (dxl_enabled) {
        dxlComm->getHardwareStatus(&dxl_connection_ok, dxl_error_message, &dxl_calibration_needed,
                &dxl_calibration_in_progress, dxl_motor_names, dxl_motor_types, dxl_temperatures, dxl_voltages, dxl_hw_errors);
    }

    motor_names.clear();
    motor_types.clear();
    temperatures.clear();
    voltages.clear();
    hw_errors.clear();

    motor_names.insert(motor_names.end(), can_motor_names.begin(), can_motor_names.end());
    motor_names.insert(motor_names.end(), dxl_motor_names.begin(), dxl_motor_names.end());
    motor_types.insert(motor_types.end(), can_motor_types.begin(), can_motor_types.end());
    motor_types.insert(motor_types.end(), dxl_motor_types.begin(), dxl_motor_types.end());
    temperatures.insert(temperatures.end(), can_temperatures.begin(), can_temperatures.end());
    temperatures.insert(temperatures.end(), dxl_temperatures.begin(), dxl_temperatures.end());
    voltages.insert(voltages.end(), can_voltages.begin(), can_voltages.end());
    voltages.insert(voltages.end(), dxl_voltages.begin(), dxl_voltages.end());
    hw_errors.insert(hw_errors.end(), can_hw_errors.begin(), can_hw_errors.end());
    hw_errors.insert(hw_errors.end(), dxl_hw_errors.begin(), dxl_hw_errors.end());

    *(is_connection_ok) = (can_connection_ok && dxl_connection_ok);
    *(calibration_needed) = (can_calibration_needed || dxl_calibration_needed);
    *(calibration_in_progress) = (can_calibration_in_progress || dxl_calibration_in_progress);
    error_message = "";
    error_message += can_error_message;
    if (dxl_error_message != "") {
        error_message += "\n";
    }
    error_message += dxl_error_message;
}

void NiryoOneCommunication::getFirmwareVersions(std::vector<std::string> &motor_names,
        std::vector<std::string> &firmware_versions)
{
    std::vector<std::string> can_firmware_versions;
    std::vector<std::string> can_motor_names;

    if (can_enabled) {
        canComm->getFirmwareVersions(can_motor_names, can_firmware_versions);
    }

    motor_names.clear();
    firmware_versions.clear();

    firmware_versions.insert(firmware_versions.end(), can_firmware_versions.begin(), can_firmware_versions.end());
    motor_names.insert(motor_names.end(), can_motor_names.begin(), can_motor_names.end());
}

void NiryoOneCommunication::getCurrentPosition(double pos[6])
{
    if (can_enabled) { canComm->getCurrentPositionV2(&pos[0], &pos[1], &pos[2]); }
    if (dxl_enabled) { dxlComm->getCurrentPositionV2(&pos[3], &pos[4], &pos[5]); }

    // if disabled (debug purposes)
    if (!can_enabled) {
        pos[0] = pos_can_disabled_v2[0];
        pos[1] = pos_can_disabled_v2[1];
        pos[2] = pos_can_disabled_v2[2];
    }

    if (!dxl_enabled) {
        pos[3] = pos_dxl_disabled_v2[0];
        pos[4] = pos_dxl_disabled_v2[1];
        pos[5] = pos_dxl_disabled_v2[2];
    }
}

void NiryoOneCommunication::sendPositionToRobot(const double cmd[6])
{
    bool is_calibration_in_progress = false;
    if (can_enabled) {
        is_calibration_in_progress = canComm->isCalibrationInProgress();
    }

    // don't send position command when calibrating motors
    if (!is_calibration_in_progress) {
        if (can_enabled) { canComm->setGoalPositionV2(cmd[0], cmd[1], cmd[2]); }
        if (dxl_enabled) { dxlComm->setGoalPositionV2(cmd[3], cmd[4], cmd[5]); }

        // if disabled (debug purposes)
        if (!can_enabled) {
            pos_can_disabled_v2[0] = cmd[0];
            pos_can_disabled_v2[1] = cmd[1];
            pos_can_disabled_v2[2] = cmd[2];
        }

        if (!dxl_enabled) {
            pos_dxl_disabled_v2[0] = cmd[3];
            pos_dxl_disabled_v2[1] = cmd[4];
            pos_dxl_disabled_v2[2] = cmd[5];
        }
    }
}

void NiryoOneCommunication::addCustomDxlCommand(int motor_type, uint8_t id, uint32_t value,
        uint32_t reg_address, uint32_t byte_number)
{
    if (dxl_enabled) {
        dxlComm->addCustomDxlCommand(motor_type, id, value, reg_address, byte_number);
    }
}

void NiryoOneCommunication::rebootMotors()
{
    // Only useful for Dynamixel motors
    if (dxl_enabled) { dxlComm->rebootMotors(); }
}

void NiryoOneCommunication::activateLearningMode(bool activate)
{
    if (can_enabled) { canComm->setTorqueOn(!activate); }
    if (dxl_enabled) { dxlComm->setTorqueOn(!activate); }
}

bool NiryoOneCommunication::setLeds(std::vector<int> &leds, std::string &message)
{
    if (leds.size() != 4) {
        message = "Led array must have 4 values";
        return false;
    }

    if (dxl_enabled) {
        dxlComm->setLeds(leds);
    }

    message = "Set LED ok";
    return true;
}

int NiryoOneCommunication::pingAndSetDxlTool(uint8_t id, std::string name)
{
    // 11, Gripper 1
    if (dxl_enabled) {
        return dxlComm->pingAndSetTool(id, name);
    }
    return TOOL_STATE_PING_OK;
}

int NiryoOneCommunication::openGripper(uint8_t id, uint16_t open_position, uint16_t open_speed, uint16_t open_hold_torque)
{
    // Speed: [100, 1000] - 300, gripper1, id: 11: open pos: 600, closed pos: 230, hold torque: 128, max close torque: 1023
    if (dxl_enabled) {
        return dxlComm->openGripper(id, open_position, open_speed, open_hold_torque);
    }
    return GRIPPER_STATE_OPEN;
}

int NiryoOneCommunication::closeGripper(uint8_t id, uint16_t close_position, uint16_t close_speed, uint16_t close_hold_torque, uint16_t close_max_torque)
{
    if (dxl_enabled) {
        return dxlComm->closeGripper(id, close_position, close_speed, close_hold_torque, close_max_torque);
    }
    return GRIPPER_STATE_CLOSE;
}
