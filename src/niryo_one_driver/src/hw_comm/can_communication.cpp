/*
    can_communication.cpp
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

#include "niryo_one_driver/can_communication.h"
#include "ros_replacements/status_output.h"

int32_t CanCommunication::rad_pos_to_steps(double position_rad, double gear_ratio, double direction)
{
    return (int32_t) ((200.0 * 8.0 * gear_ratio * position_rad * RADIAN_TO_DEGREE_C / 360.0) * direction);
}

double CanCommunication::steps_to_rad_pos(int32_t steps, double gear_ratio, double direction)
{
    return (double) ((double)steps * 360.0 / (200.0 * 8.0 * gear_ratio * RADIAN_TO_DEGREE_C)) * direction ;
}

CanCommunication::CanCommunication()
{

}

int CanCommunication::init()
{
    update_id = false;
    
    // TODO: get params
    // ros::param::get("~spi_channel", spi_channel);
    spi_channel = 0;
    // ros::param::get("~spi_baudrate", spi_baudrate);
    spi_baudrate = 1000000;
    // ros::param::get("~gpio_can_interrupt", gpio_can_interrupt);
    gpio_can_interrupt = 25;

    // // set frequencies for hw control loop
    // ros::param::get("~can_hardware_control_loop_frequency", hw_control_loop_frequency);
    hw_control_loop_frequency = 1500.0;
    // ros::param::get("~can_hw_write_frequency", hw_write_frequency);
    hw_write_frequency = 50.0;

    // ros::param::get("~can_hw_check_connection_frequency", hw_check_connection_frequency);
    hw_check_connection_frequency = 3.0;

    OUTPUT_INFO("Start CAN communication (%lf Hz)", hw_control_loop_frequency);
    OUTPUT_INFO("Writing data on CAN at %lf Hz", hw_write_frequency);
    OUTPUT_INFO("Checking CAN connection at %lf Hz", hw_check_connection_frequency);

    resetHardwareControlLoopRates();

    // TODO: get params
    // // set calibration timeout
    // ros::param::get("~calibration_timeout", calibration_timeout);
    calibration_timeout = 40;
    OUTPUT_INFO("NiryoStepper calibration timeout: %d seconds", calibration_timeout);
    // ROS_INFO("NiryoStepper calibration timeout: %d seconds", calibration_timeout);

    // start can driver
    can.reset(new NiryoCanDriver(spi_channel, spi_baudrate, gpio_can_interrupt));

    is_can_connection_ok = false;
    debug_error_message = "No connection with CAN motors has been made yet";

    // TODO: get params
    // // get connected motors from rosparams
    // ros::param::get("/niryo_one/motors/can_required_motors", required_steppers_ids);
    required_steppers_ids.push_back(1); required_steppers_ids.push_back(2); required_steppers_ids.push_back(3);
    // ros::param::get("/niryo_one/motors/can_authorized_motors", allowed_steppers_ids);
    allowed_steppers_ids.push_back(1); allowed_steppers_ids.push_back(2); allowed_steppers_ids.push_back(3); allowed_steppers_ids.push_back(6); allowed_steppers_ids.push_back(7);

    required_steppers_ids.insert(required_steppers_ids.end(), required_steppers_ids.begin(), required_steppers_ids.end());
    allowed_steppers_ids.insert(allowed_steppers_ids.end(), required_steppers_ids.begin(), required_steppers_ids.end());
    allowed_steppers_ids.insert(allowed_steppers_ids.end(), allowed_steppers_ids.begin(), allowed_steppers_ids.end());



    double gear_ratio_1, gear_ratio_2, gear_ratio_3, gear_ratio_4, gear_ratio_6, gear_ratio_7;
    // TODO: get params
    // ros::param::get("/niryo_one/motors/stepper_1_gear_ratio", gear_ratio_1);
    gear_ratio_1 = 6.0625;
    // ros::param::get("/niryo_one/motors/stepper_2_gear_ratio", gear_ratio_2);
    gear_ratio_2 = 8.3125;
    // ros::param::get("/niryo_one/motors/stepper_3_gear_ratio", gear_ratio_3);
    gear_ratio_3 = 7.875;
    // ros::param::get("/niryo_one/motors/stepper_4_gear_ratio", gear_ratio_4);
    gear_ratio_4 = 5.0;
    // ros::param::get("/niryo_one/motors/stepper_6_gear_ratio", gear_ratio_6);
    gear_ratio_6 = 5.0;
    // ros::param::get("/niryo_one/motors/stepper_7_gear_ratio", gear_ratio_7);
    gear_ratio_7 = 5.0;

    OUTPUT_INFO("Gear ratios : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf, 6 : %lf, 7, %lf)", gear_ratio_1, gear_ratio_2, gear_ratio_3, gear_ratio_4, gear_ratio_6, gear_ratio_7);

    double home_position_1, home_position_2, home_position_3, home_position_4;
    // TODO: get params
    // ros::param::get("/niryo_one/motors/stepper_1_home_position", home_position_1);
    home_position_1 = 0.0;
    // ros::param::get("/niryo_one/motors/stepper_2_home_position", home_position_2);
    home_position_2 = 0.640187;
    // ros::param::get("/niryo_one/motors/stepper_3_home_position", home_position_3);
    home_position_3 = -1.397485;
    // ros::param::get("/niryo_one/motors/stepper_4_home_position", home_position_4);
    home_position_4 = 0.0;

    OUTPUT_INFO("Home positions : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", home_position_1, home_position_2, home_position_3, home_position_4);

    double offset_position_1, offset_position_2, offset_position_3, offset_position_4;
    // TODO: get params
    // ros::param::get("/niryo_one/motors/stepper_1_offset_position", offset_position_1);
    offset_position_1 = 3.05433;
    // ros::param::get("/niryo_one/motors/stepper_2_offset_position", offset_position_2);
    offset_position_2 = 0.640187;
    // ros::param::get("/niryo_one/motors/stepper_3_offset_position", offset_position_3);
    offset_position_3 = -1.397485;
    // ros::param::get("/niryo_one/motors/stepper_4_offset_position", offset_position_4);
    offset_position_4 = 2.791;
    OUTPUT_INFO("Angle offsets : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", offset_position_1, offset_position_2, offset_position_3, offset_position_4);

    double direction_1, direction_2, direction_3;
    // TODO: get params
    // ros::param::get("/niryo_one/motors/stepper_1_direction", direction_1);
    direction_1 = -1.0;
    // ros::param::get("/niryo_one/motors/stepper_2_direction", direction_2);
    direction_2 = -1.0;
    // ros::param::get("/niryo_one/motors/stepper_3_direction", direction_3);
    direction_3 = 1.0;

    int max_effort_1, max_effort_2, max_effort_3;
    // TODO: get params
    // ros::param::get("/niryo_one/motors/stepper_1_max_effort", max_effort_1);
    max_effort_1 = 90;
    // ros::param::get("/niryo_one/motors/stepper_2_max_effort", max_effort_2);
    max_effort_2 = 130;
    // ros::param::get("/niryo_one/motors/stepper_3_max_effort", max_effort_3);
    max_effort_3 = 120;

    // Create motors with previous params
    m1 = StepperMotorState("Stepper Axis 1", CAN_MOTOR_1_ID, gear_ratio_1, direction_1,
            rad_pos_to_steps(home_position_1, gear_ratio_1, direction_1),            // home position
            rad_pos_to_steps(offset_position_1, gear_ratio_1, direction_1),          // offset position
            8, max_effort_1);
    m2 = StepperMotorState("Stepper Axis 2", CAN_MOTOR_2_ID, gear_ratio_2, direction_2,
            rad_pos_to_steps(home_position_2, gear_ratio_2, direction_2),
            rad_pos_to_steps(offset_position_2, gear_ratio_2, direction_2),
            8, max_effort_2);
    m3 = StepperMotorState("Stepper Axis 3", CAN_MOTOR_3_ID, gear_ratio_3, direction_3,
            rad_pos_to_steps(home_position_3, gear_ratio_3, direction_3),
            rad_pos_to_steps(offset_position_3, gear_ratio_3, direction_3),
            8, max_effort_3);

    for (uint8_t i = 0 ; i < required_steppers_ids.size() ; ++i) {
        if      (required_steppers_ids.at(i) == m1.getId()) { m1.enable(); }
        else if (required_steppers_ids.at(i) == m2.getId()) { m2.enable(); }
        else if (required_steppers_ids.at(i) == m3.getId()) { m3.enable(); }
        else {
            debug_error_message = "Incorrect configuration : Wrong ID (" + std::to_string(required_steppers_ids.at(i))
                + ") given in Ros Param /niryo_one_motors/can_required_motors. You need to fix this !";
            OUTPUT_ERROR(debug_error_message);
            return -1;
        }
    }
    if (required_steppers_ids.size() == 0) {
        debug_error_message = "Incorrect configuration : Ros Param /niryo_one_motors/can_required_motors "
        "should contain a list with at least one motor. You need to fix this !";
        OUTPUT_ERROR(debug_error_message);
        return -1;
    }

    OUTPUT_INFO("Number of connected motors: %d", required_steppers_ids.size());

    // fill motors array (to avoid redundant code later)
    motors.push_back(&m1);
    motors.push_back(&m2);
    motors.push_back(&m3);

    allowed_motors.push_back(&m1);
    allowed_motors.push_back(&m2);
    allowed_motors.push_back(&m3);

    // set hw control init state
    torque_on = 0;

    hw_is_busy = false;
    hw_limited_mode = true;

    write_position_enable = true;
    write_torque_enable = false;
    write_torque_on_enable = true;

    write_synchronize_enable = false;
    write_micro_steps_enable =  true;
    write_max_effort_enable = true;

    waiting_for_user_trigger_calibration = false;
    steppers_calibration_mode = CAN_STEPPERS_CALIBRATION_MODE_AUTO; // default
    write_synchronize_begin_traj = true;
    calibration_in_progress = false;
    
 

    return setupCommunication();
}

int CanCommunication::setupCommunication()
{
    int gpio_result = can->setupInterruptGpio();
    if (gpio_result != CAN_OK) {
        OUTPUT_ERROR("Error setting up GPIO");
        return gpio_result;
    }

    int spi_result = can->setupSpi();
    if (spi_result != CAN_OK) {
        OUTPUT_ERROR("Error setting up SPI");
        return spi_result;
    }

    // will return 0 on success
    return can->init();
}

bool CanCommunication::isOnLimitedMode()
{
    return hw_limited_mode;
}

void CanCommunication::resetHardwareControlLoopRates()
{
    auto now = repl::time_now();
    time_hw_last_write = now;
    time_hw_last_check_connection = now;
}

void CanCommunication::startHardwareControlLoop(bool limited_mode)
{
    OUTPUT_INFO("Start hardware control loop");
    write_torque_on_enable = true;
    write_micro_steps_enable = true;
    write_max_effort_enable = true;
    resetHardwareControlLoopRates();

    // depends on limited_mode flag
    write_position_enable = !limited_mode;
    write_synchronize_enable = !limited_mode;
    write_torque_on_enable = true;

    hw_limited_mode = limited_mode;
    hw_control_loop_keep_alive = true;

    if (!hardware_control_loop_thread) {
        OUTPUT_WARNING("START control loop thread");
        hardware_control_loop_thread.reset(new std::thread(std::bind(&CanCommunication::hardwareControlLoop, this)));
    }
}

void CanCommunication::stopHardwareControlLoop()
{
    OUTPUT_INFO("Stop hardware control loop");
    for (unsigned int i = 0; i < motors.size(); i++) {
        motors.at(i)->resetState();
    }
    hw_control_loop_keep_alive = false;
}

void CanCommunication::hardwareControlRead()
{
    if (can->canReadData()) {
        long unsigned int rxId;
        unsigned char len;
        unsigned char rxBuf[8];

        can->readMsgBuf(&rxId, &len, rxBuf);

        // 0. This functionality will come later, to allow user to plug other CAN devices to RPI
        // Developement to do here : check if id >= 0x20
        // Ids between 0x00 and 0x1F are reserved for Niryo One core communication
        // Those are lower ids with higher priority, to ensure connection with motors is always up.
	    if (rxId >= 0x20) {
            // send frame to another place and return
            //ROS_INFO("frame  will be send somewhere else ");
            //return ;
        }

        // 1. Validate motor id
        int motor_id = rxId & 0x0F; // 0x11 for id 1, 0x12 for id 2, ...
        
        // treat niryo one steppers
        bool motor_found = false;
        for (unsigned int i = 0; i < allowed_motors.size(); i++) {
            if (motor_id == allowed_motors.at(i)->getId()){
                allowed_motors.at(i)->setLastTimeRead(repl::time_now());
                motor_found = true;
                break;
            }
        }

        if (!motor_found) {
            OUTPUT_ERROR("Received can frame with wrong id : %d", motor_id);
            debug_error_message = "Unallowed connected motor : ";
            debug_error_message += std::to_string(motor_id);
            is_can_connection_ok = false;
            return;
        }

        // 1.1 Check buffer is not empty
        if (len < 1) {
            OUTPUT_ERROR("Received can frame with empty data");
            return;
        }

        // 2. If id ok, check control byte and fill data
        int control_byte = rxBuf[0];

        if (control_byte == CAN_DATA_POSITION) {
            // check length
            if (len != 4) {
                OUTPUT_ERROR("Position can frame should contain 4 data bytes");
                return;
            }

            int32_t pos = (rxBuf[1] << 16) + (rxBuf[2] << 8) + rxBuf[3];
            if (pos & (1 << 15)) {
            	pos = -1 * ((~pos + 1) & 0xFFFF);
          	}

            // fill data
            for (unsigned int i = 0; i < motors.size() ; i++) {
                if (motor_id == motors.at(i)->getId() && motors.at(i)->isEnabled()) {
                    motors.at(i)->setPositionState(pos);
                    break;
                }
            }
        }
        else if (control_byte == CAN_DATA_DIAGNOSTICS) {
            // check data length
            if (len != 4) {
                OUTPUT_ERROR("Diagnostic can frame should contain 4 data bytes");
                return;
            }
            int driver_temp_raw = (rxBuf[2] << 8) + rxBuf[3];
            double a = -0.00316;
            double b = -12.924;
            double c = 2367.7;
            double v_temp = driver_temp_raw * 3.3 / 1024.0 * 1000.0;
            int driver_temp = int((-b - std::sqrt(b*b - 4*a*(c - v_temp)))/(2*a)+30);

            // fill data
            for (unsigned int i = 0; i < motors.size() ; i++) {
                if (motor_id == motors.at(i)->getId() && motors.at(i)->isEnabled()) {
                    motors.at(i)->setTemperatureState(driver_temp);
                    break;
                }
            }
        }
        else if (control_byte == CAN_DATA_FIRMWARE_VERSION) {
            if (len != 4) {
                OUTPUT_ERROR("Firmware version frame should contain 4 bytes");
                return;
            }
            int v_major = rxBuf[1];
            int v_minor = rxBuf[2];
            int v_patch = rxBuf[3];
            std::string version = "";
            version += std::to_string(v_major); version += ".";
            version += std::to_string(v_minor); version += ".";
            version += std::to_string(v_patch);

            // fill data
            for (unsigned int i = 0; i < motors.size(); i++) {
                if (motor_id == motors.at(i)->getId() && allowed_motors.at(i)->isEnabled()) {
                    motors.at(i)->setFirmwareVersion(version);
                    return;
                }
            }
        }
        else {
            OUTPUT_ERROR("Received can frame with unknown control byte");
            return;
        }
    }
}

/*
 * Sends a CAN frame per motor (id + control byte + data)
 */
void CanCommunication::hardwareControlWrite()
{
    if (repl::time_now() - time_hw_last_write > repl::sec_to_usec(1.0/hw_write_frequency)) {
        time_hw_last_write += repl::sec_to_usec(1.0/hw_write_frequency);

        // write torque ON/OFF
        if (write_torque_on_enable) {
            if (can->sendTorqueOnCommand(CAN_BROADCAST_ID, torque_on) != CAN_OK) {
                OUTPUT_ERROR("Failed to send torque on");
            }
            else {
                write_torque_on_enable = false; // disable writing on success
            }

        }

        // write synchronize position
        if (write_synchronize_enable) {
            bool synchronize_write_success = true;

            for (unsigned int i = 0 ; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    if (can->sendSynchronizePositionCommand(motors.at(i)->getId(), write_synchronize_begin_traj) != CAN_OK) {
                        synchronize_write_success = false;
                    }
                }
            }

            if (synchronize_write_success) {
                write_synchronize_enable = false; // disable writing after success
            }
            else {
                OUTPUT_ERROR("Failed to send synchronize position command");
            }
        }

        // write position
        if (write_position_enable) {


            for (unsigned int i = 0 ; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {

                    if (can->sendPositionCommand(motors.at(i)->getId(), motors.at(i)->getPositionCommand()) != CAN_OK) {
                        //ROS_ERROR("Failed to send position");

                    }
                }
            }
        }

        // write micro steps
        if (write_micro_steps_enable) {
            bool micro_steps_write_success = true;
            for (unsigned int i = 0 ; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    if (can->sendMicroStepsCommand(motors.at(i)->getId(), motors.at(i)->getMicroStepsCommand()) != CAN_OK) {
                        micro_steps_write_success = false;
                    }
                }
            }

            if (micro_steps_write_success) {
                write_micro_steps_enable = false; // disable writing after success
            }
            else {
                OUTPUT_ERROR("Failed to send Micro Steps");
            }
        }

        // write max effort
        if (write_max_effort_enable) {
            bool max_effort_write_success = true;

            for (unsigned int i = 0; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    if (can->sendMaxEffortCommand(motors.at(i)->getId(), motors.at(i)->getMaxEffortCommand()) != CAN_OK) {
                        max_effort_write_success = false;
                    }
                }
            }

            if (max_effort_write_success) {
                write_max_effort_enable = false; // disable writing on success
            }
            else {
                OUTPUT_ERROR("Failed to send Max Effort");
            }
        }

    }
}

void CanCommunication::hardwareControlCheckConnection()
{
    if (repl::time_now() - time_hw_last_check_connection > repl::sec_to_usec(1.0/hw_check_connection_frequency)) {
        time_hw_last_check_connection += repl::sec_to_usec(1.0/hw_check_connection_frequency);

        if (!is_can_connection_ok) {
            return; // don't check if connection is already lost --> need to call scanAndCheck()
        }

        auto time_now = repl::time_now();
        if (hw_check_connection_frequency > 5.0) { // if we check MCP_2515 too fast it will not work
            hw_check_connection_frequency = 5.0;
        }
        auto timeout_read = repl::sec_to_usec(1.0/hw_check_connection_frequency);
        int max_fail_counter = (int) (hw_check_connection_frequency + 0.5); // connection error will be detected after 1 sec

        for (unsigned int i = 0; i < motors.size(); i++) {
            if (motors.at(i)->isEnabled()) {
                if (time_now - motors.at(i)->getLastTimeRead() > timeout_read * (motors.at(i)->getHwFailCounter() + 1)) {
                    OUTPUT_ERROR("CAN connection problem with motor %d, hw fail counter : %d", motors.at(i)->getId(), motors.at(i)->getHwFailCounter());
                    if (motors.at(i)->getHwFailCounter() >= max_fail_counter) {
                        is_can_connection_ok = false;
                        debug_error_message = "Connection problem with CAN bus. Motor ";
                        debug_error_message += motors.at(i)->getName();
                        debug_error_message += " is not connected";
                        return;
                    }

                    // reset MCP_2515
                    //int result = can->init();
                    //ROS_INFO("Can init : %d", result);
                    motors.at(i)->setHwFailCounter(motors.at(i)->getHwFailCounter() + 1);
                }
                else {
                    motors.at(i)->setHwFailCounter(0);
                }
            }
        }
    }
}

void CanCommunication::hardwareControlLoop()
{
    // ros::Rate hw_control_loop_rate = ros::Rate(hw_control_loop_frequency);
    bool ok = true;

    repl::Rate rate(hw_control_loop_frequency);
    while (ok) {
        if (!hw_is_busy && hw_control_loop_keep_alive) {
            hw_is_busy = true;

            hardwareControlRead();
            hardwareControlWrite();
            hardwareControlCheckConnection();

            hw_is_busy = false;
            rate.sleep();
        }
        else {
            repl::sleep(TIME_TO_WAIT_IF_BUSY);
            resetHardwareControlLoopRates();
           // ROS_INFO("HW control loop, wait because is busy");
        }
    }
    // while (ros::ok()) {
    //     if (!hw_is_busy && hw_control_loop_keep_alive) {
    //         hw_is_busy = true;

    //         hardwareControlRead();
    //         hardwareControlWrite();
    //         hardwareControlCheckConnection();

    //         hw_is_busy = false;
    //         hw_control_loop_rate.sleep();
    //     }
    //     else {
    //         ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();
    //         resetHardwareControlLoopRates();
    //        // ROS_INFO("HW control loop, wait because is busy");
    //     }
    // }
}

void CanCommunication::synchronizeSteppers(bool begin_traj)
{
    write_synchronize_enable = true;
    write_synchronize_begin_traj = begin_traj;
}

void CanCommunication::setTorqueOn(bool on)
{
    if (!on && is_can_connection_ok && waiting_for_user_trigger_calibration && !calibration_in_progress) {
        can->sendTorqueOnCommand(CAN_BROADCAST_ID, false); // only to deactivate motors when waiting for calibration
    }
    else if (hw_limited_mode) {
        torque_on = false;
        write_torque_on_enable = true;
    }
    else {
        torque_on = on;
        write_torque_on_enable = true;
    }
}

bool CanCommunication::canProcessManualCalibration(std::string &result_message)
{
    // 1. Check if motors firmware version is ok
    for (unsigned int i = 0; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            std::string firmware_version = motors.at(i)->getFirmwareVersion();
            if (firmware_version.length() == 0) {
                result_message = "No firmware version available for motor " + std::to_string(motors.at(i)->getId())
                    + ". Make sure all motors are connected";
                OUTPUT_WARNING("Can't process manual calibration : %s", result_message.c_str());
                return false;
            }
            if (std::stoi(firmware_version.substr(0,1)) < 2) {
                result_message = "You need to upgrade stepper firmware for motor " + std::to_string(motors.at(i)->getId());
                OUTPUT_WARNING("Can't process manual calibration : %s", result_message.c_str());
                return false;
            }
        }
    }

    // 2. Check if motor offset values have been previously saved (with auto calibration)
    std::vector<int> motor_id_list;
    std::vector<int> steps_list;
    if (!get_motors_calibration_offsets(motor_id_list, steps_list)) {
        result_message = "You need to make one auto calibration before using the manual calibration";
        OUTPUT_WARNING("Can't process manual calibration : %s", result_message.c_str());
        return false;
    }

    // 3. Check if all connected motors have a motor offset value
    for (unsigned int i = 0; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            for (unsigned int j = 0; j < motor_id_list.size(); j++) {
                if (motor_id_list.at(j) == motors.at(i)->getId()) {
                    break;
                }
                if (j == motor_id_list.size() - 1) {
                    result_message = "Motor " + std::to_string(motors.at(i)->getId()) + " does not have a saved offset value, "
                        + "you need to do one auto calibration";
                    OUTPUT_WARNING("Can't process manual calibration : %s", result_message.c_str());
                    return false;
                }
            }
        }
    }

    return true;
}

/*
 * User input to clear the calibration flag
 * - also choose a calibration mode (manual|auto)
 */
void CanCommunication::validateMotorsCalibrationFromUserInput(int mode)
{
    waiting_for_user_trigger_calibration = false;
    if (mode == CAN_STEPPERS_CALIBRATION_MODE_MANUAL || mode == CAN_STEPPERS_CALIBRATION_MODE_AUTO) {
        steppers_calibration_mode = mode;
    }
}

/*
 * This flag should be cleared by calling the service /niryo_one_control/calibrate_motors
 */
void CanCommunication::setCalibrationFlag(bool flag)
{
    waiting_for_user_trigger_calibration = flag;
}

/*
 * 1. If mode is manual, it will just send an offset to all the motors, it means the user has to place the robot
 * to home position before using this mode
 * 2. If mode is automatic, it will send a calibration command to all motors, and wait until it receives a confirmation
 * from all motors (success, timeout, bad params)
 *  Also, the auto calibration is done with multiple steps,
 */
int CanCommunication::calibrateMotors(int calibration_step)
{
    if (!is_can_connection_ok) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // this flag should be cleared by a user action to continue
    if (waiting_for_user_trigger_calibration) {
        return CAN_STEPPERS_CALIBRATION_WAITING_USER_INPUT;
    }

    OUTPUT_INFO("START Calibrating stepper motors, step number %d", calibration_step);
    stopHardwareControlLoop();
    repl::sleep(0.1);

    // If user wants to do a manual calibration, just send offset to current position
    if (steppers_calibration_mode == CAN_STEPPERS_CALIBRATION_MODE_MANUAL) {
        if (calibration_step > 1) {
            return CAN_STEPPERS_CALIBRATION_OK; // do manual calibration only once
        }
        calibration_in_progress = true;
        int result = manualCalibration();
        calibration_in_progress = false;
        return result;
    }
    else if (steppers_calibration_mode == CAN_STEPPERS_CALIBRATION_MODE_AUTO) {
        if (calibration_step == 1) {
            calibration_in_progress = true;
            int result = autoCalibrationStep1();
            return result;
        }
        else if (calibration_step == 2) {
            int result = autoCalibrationStep2();
            calibration_in_progress = false;
            return result;
        } else {
            return CAN_STEPPERS_CALIBRATION_FAIL;
        }
    }
    else {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }
}

int CanCommunication::getCalibrationMode()
{
    return steppers_calibration_mode;
}

bool CanCommunication::isCalibrationInProgress()
{
    return calibration_in_progress;
}

/*
 * Set to home position instead of offset position (more intuitive for user)
 */
int CanCommunication::manualCalibration()
{
    std::vector<int> motor_id_list;
    std::vector<int> steps_list;
    if (!get_motors_calibration_offsets(motor_id_list, steps_list)) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    for (unsigned int i = 0; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            // compute step offset to send
            int offset_to_send = 0;
            int sensor_offset_steps = 0;
            int absolute_steps_at_offset_position = 0;
            int offset_steps = motors.at(i)->getOffsetPosition();
            for (unsigned int j = 0; j < motor_id_list.size(); j++) {
                if (motors.at(i)->getId() == motor_id_list.at(j)) {
                    sensor_offset_steps = steps_list.at(j);
                    break;
                }
            }
            if (motors.at(i)->getId() == 1 || motors.at(i)->getId() == 2 || motors.at(i)->getId() == 4) { // position 0.0
                offset_to_send = (sensor_offset_steps - offset_steps) % 1600;
                if (offset_to_send < 0) { offset_to_send += 1600; }
                absolute_steps_at_offset_position = offset_to_send;
            }
            else if (motors.at(i)->getId() == 3) { // max position
                offset_to_send = sensor_offset_steps - offset_steps;
                absolute_steps_at_offset_position = sensor_offset_steps;
            }

            OUTPUT_INFO("Motor %d - sending offset : %d", motors.at(i)->getId(), offset_to_send);
            if (can->sendPositionOffsetCommand(motors.at(i)->getId(), offset_to_send, absolute_steps_at_offset_position) != CAN_OK) {
                return CAN_STEPPERS_CALIBRATION_FAIL;
            }
        }
    }

    return CAN_STEPPERS_CALIBRATION_OK;
}

int CanCommunication::sendCalibrationCommandForOneMotor(StepperMotorState* motor, int delay_between_steps,
        int calibration_direction, int calibr_timeout)
{
    if (!motor->isEnabled()) {
        return CAN_OK;
    }

    if (can->sendCalibrationCommand(motor->getId(), motor->getOffsetPosition(), delay_between_steps,
                (int)motor->getDirection() * calibration_direction, calibr_timeout) != CAN_OK) {
        OUTPUT_ERROR("Failed to send calibration command for motor : %d", motor->getId());
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    return CAN_OK;
}

int CanCommunication::getCalibrationResults(std::vector<StepperMotorState*> steppers, int calibr_timeout,
        std::vector<int> &sensor_offset_ids, std::vector<int> &sensor_offset_steps)
{
    std::vector<int> motors_ids;
    std::vector<bool> calibration_results;

    for (unsigned int i = 0 ; i < steppers.size() ; i++) {
        if (steppers.at(i)->isEnabled()) {
            motors_ids.push_back(steppers.at(i)->getId());
            calibration_results.push_back(false);
        }
    }

    auto time_begin_calibration = repl::time_now();
    auto timeout = time_begin_calibration + std::chrono::seconds(calibr_timeout);
    OUTPUT_INFO("Waiting for motor calibration");
    //ROS_INFO("Waiting for motor %d calibration response...", motor->getId());

    while (repl::time_now() < timeout) { 
        repl::sleep(0.0005); // check at 2000 Hz

        // check if success
        bool success = true;
        for (unsigned int i = 0 ; i < calibration_results.size() ; i++) {
            if (calibration_results.at(i) == false) {
                success = false;
            }
        }
        if (success) {
            return CAN_STEPPERS_CALIBRATION_OK;
        }

        if (can->canReadData()) {
            long unsigned int rxId;
            unsigned char len;
            unsigned char rxBuf[8];

            can->readMsgBuf(&rxId, &len, rxBuf);

            // 1. Get motor id
            int motor_id = rxId & 0x00F; // 0x101 for id 1, 0x102 for id 2, ...

            // 2. Check if motor id is in array

            for (unsigned int i = 0 ; i < motors_ids.size() ; i++) {
                if (motors_ids.at(i) == motor_id) {
                    if (len == 2) {
                        // 3. Check control byte
                        int control_byte = rxBuf[0];

                        if (control_byte == CAN_DATA_CALIBRATION_RESULT) { // only check this frame
                            int result = rxBuf[1];

                            if (result == CAN_STEPPERS_CALIBRATION_TIMEOUT) {
                                OUTPUT_ERROR("Motor %d had calibration timeout", motor_id);
                                return result;
                            }
                            else if (result == CAN_STEPPERS_CALIBRATION_BAD_PARAM) {
                                OUTPUT_ERROR("Bad params given to motor %d", motor_id);
                                return result;
                            }
                            else if (result == CAN_STEPPERS_CALIBRATION_OK) {
                                OUTPUT_INFO("Motor %d calibration OK", motor_id);
                                calibration_results.at(i) = true;
                            }
                        }
                    }
                    else if (len == 4) { // new firmware version -> get result + absolute sensor steps at offset position
                        // 3. Check control byte
                        int control_byte = rxBuf[0];

                        if (control_byte == CAN_DATA_CALIBRATION_RESULT) { // only check this frame
                            int result = rxBuf[1];

                            if (result == CAN_STEPPERS_CALIBRATION_TIMEOUT) {
                                OUTPUT_ERROR("Motor %d had calibration timeout", motor_id);
                                return result;
                            }
                            else if (result == CAN_STEPPERS_CALIBRATION_BAD_PARAM) {
                                OUTPUT_ERROR("Bad params given to motor %d", motor_id);
                                return result;
                            }
                            else if (result == CAN_STEPPERS_CALIBRATION_OK) {
                                OUTPUT_INFO("Motor %d - Calibration OK", motor_id);
                                int steps_at_offset_pos = (rxBuf[2] << 8) + rxBuf[3];
                                OUTPUT_INFO("Motor %d - Absolute steps at offset position : %d", motor_id, steps_at_offset_pos);
                                sensor_offset_ids.push_back(motor_id);
                                sensor_offset_steps.push_back(steps_at_offset_pos);
                                calibration_results.at(i) = true;

                                // keep torque ON for axis 2
                                // (if torsion spring is too strong the axis might move too much for the following calibration steps)
                                if (motor_id == m2.getId()) {
                                    can->sendTorqueOnCommand(m2.getId(), true);
                                }
                                // keep torque ON for axis 1
                                if (motor_id == m1.getId()) {
                                    can->sendTorqueOnCommand(m1.getId(), true);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return CAN_STEPPERS_CALIBRATION_TIMEOUT;
}

/*
 * To use only during calibration phase, or for debug purposes
 * - Move motor from whatever current position to current_steps + steps
 */
int CanCommunication::relativeMoveMotor(StepperMotorState* motor, int steps, int delay, bool wait)
{
    if (!motor->isEnabled()) {
        return CAN_OK;
    }

    if (can->sendTorqueOnCommand(motor->getId(), true) != CAN_OK) {
        OUTPUT_ERROR("Failed to send torque ON to motor %d", motor->getId());
        return CAN_FAIL;
    }

    if (can->sendRelativeMoveCommand(motor->getId(), steps, delay) != CAN_OK) {
        OUTPUT_ERROR("Relative Move motor failed for motor %d", motor->getId());
        return CAN_FAIL;
    }
    if (wait) {
        repl::sleep(steps*delay/1000000 + 0.5); // wait for 0.5 sec more to finish
    }

    return CAN_OK;
}

int CanCommunication::autoCalibrationStep1()
{
    // 0. Torque ON for motor 2
    if (can->sendTorqueOnCommand(m2.getId(), true) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 1. Move axis 3 up
    if (relativeMoveMotor(&m3, rad_pos_to_steps(0.5, m3.getGearRatio(), m3.getDirection()), 1500, true) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    return CAN_STEPPERS_CALIBRATION_OK;
}

int CanCommunication::autoCalibrationStep2()
{
    std::vector<int> sensor_offset_ids;
    std::vector<int> sensor_offset_steps; // absolute steps at offset position

    // 2. Send calibration cmd 1 + 2 + 4 (+3 if hw version == 2)
    if (sendCalibrationCommandForOneMotor(&m1, 800, 1, calibration_timeout) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    if (sendCalibrationCommandForOneMotor(&m2, 1100, 1, calibration_timeout) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    if (sendCalibrationCommandForOneMotor(&m3, 1100, -1, calibration_timeout) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 2.1 Wait calibration result
    std::vector<StepperMotorState*> steppers = { &m1, &m2, &m3 };
    
    if (getCalibrationResults(steppers, calibration_timeout, sensor_offset_ids, sensor_offset_steps) != CAN_STEPPERS_CALIBRATION_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 3. Move motor 1,2 to 0.0
    if (relativeMoveMotor(&m1, -m1.getOffsetPosition(), 1300, false) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 3.1 Wait for motors to finish moving
    repl::sleep(abs(m1.getOffsetPosition()) * 1300 / 1000000);

    // 3.2 Move axis 2 to home position after axis 1
    // --> in case a gripper is attached, so it won't collide with the base while moving
    if (relativeMoveMotor(&m2, -m2.getOffsetPosition(), 3000, false) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }
    repl::sleep(abs(m2.getOffsetPosition()) * 3000 / 1000000 + 0.5);

    // 6. Write sensor_offset_steps to file
    set_motors_calibration_offsets(sensor_offset_ids, sensor_offset_steps);

    return CAN_STEPPERS_CALIBRATION_OK;
}

void CanCommunication::setGoalPositionV2(double axis_1_pos_goal, double axis_2_pos_goal, double axis_3_pos_goal)
{
    m1.setPositionCommand(rad_pos_to_steps(axis_1_pos_goal, m1.getGearRatio(), m1.getDirection()));
    m2.setPositionCommand(rad_pos_to_steps(axis_2_pos_goal, m2.getGearRatio(), m2.getDirection()));
    m3.setPositionCommand(rad_pos_to_steps(axis_3_pos_goal, m3.getGearRatio(), m3.getDirection()));

    // if motor disabled, pos_state = pos_cmd (echo position)
    for (unsigned int i = 0 ; i < motors.size(); i++) {
        if (!motors.at(i)->isEnabled()) {
            motors.at(i)->setPositionState(motors.at(i)->getPositionCommand());
        }
    }
}

void CanCommunication::getCurrentPositionV2(double *axis_1_pos, double *axis_2_pos, double *axis_3_pos)
{
    *axis_1_pos = steps_to_rad_pos(m1.getPositionState(), m1.getGearRatio(), m1.getDirection());
    *axis_2_pos = steps_to_rad_pos(m2.getPositionState(), m2.getGearRatio(), m2.getDirection());
    *axis_3_pos = steps_to_rad_pos(m3.getPositionState(), m3.getGearRatio(), m3.getDirection());
}

void CanCommunication::setMicroSteps(std::vector<uint8_t> micro_steps_list)
{
    if (micro_steps_list.size() != 4) {
        OUTPUT_WARNING("Micro steps array must have 4 values");
        return;
    }

    for (unsigned int i = 0; i < micro_steps_list.size(); i++) {
        motors.at(i)->setMicroStepsCommand(micro_steps_list.at(i));
    }

    write_micro_steps_enable = true;
}

void CanCommunication::setMaxEffort(std::vector<uint8_t> max_effort_list)
{
    if (max_effort_list.size() != 4) {
        OUTPUT_WARNING("Max effort array must have 4 values");
        return;
    }

    for (unsigned int i = 0; i < max_effort_list.size(); i++) {
        motors.at(i)->setMaxEffortCommand(max_effort_list.at(i));
    }

    write_max_effort_enable = true;
}

void CanCommunication::getHardwareStatus(bool *is_connection_ok, std::string &error_message,
        int *calibration_needed, bool *calibration_in_progress,
        std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
        std::vector<int32_t> &temperatures, std::vector<double> &voltages,
        std::vector<int32_t> &hw_errors)
{
    *(is_connection_ok) = is_can_connection_ok;
    *(calibration_needed) = (waiting_for_user_trigger_calibration && is_can_connection_ok);
    *(calibration_in_progress) = this->calibration_in_progress;
    error_message = debug_error_message;

    motor_names.clear();
    motor_types.clear();
    temperatures.clear();
    voltages.clear();
    hw_errors.clear();

    for (unsigned int i = 0 ; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            motor_names.push_back(motors.at(i)->getName());
            motor_types.push_back("Niryo Stepper");
            temperatures.push_back(motors.at(i)->getTemperatureState());
            voltages.push_back(0.0);
            hw_errors.push_back(motors.at(i)->getHardwareErrorState());
        }
    }
}

void CanCommunication::getFirmwareVersions(std::vector<std::string> &motor_names,
        std::vector<std::string> &firmware_versions)
{
    motor_names.clear();
    firmware_versions.clear();

    for (unsigned int i = 0; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            motor_names.push_back(motors.at(i)->getName());
            firmware_versions.push_back(motors.at(i)->getFirmwareVersion());
        }
    }
}

bool CanCommunication::isConnectionOk()
{
    return is_can_connection_ok;
}

/*
 * Will wait for all required motors to send one can frame
 * --> error when
 *  - a motor id is not allowed
 *  - a required motor is missing
 *  - can't get access to CAN bus
 */
int CanCommunication::scanAndCheck()
{
    int counter = 0;

    while (hw_is_busy && counter < 100) {
        repl::sleep(TIME_TO_WAIT_IF_BUSY);
        counter++;
    }

    if (counter == 100) {
        debug_error_message = "Failed to scan motors, CAN bus is too busy. Will retry...";
        OUTPUT_WARNING("Failed to scan motors, CAN bus is too busy (counter max : %d)", counter);
        return CAN_SCAN_BUSY;
    }

    hw_is_busy = true;

    // if some motors are disabled, just declare them as connected
    bool m1_ok = !m1.isEnabled();
    bool m2_ok = !m2.isEnabled();
    bool m3_ok = !m3.isEnabled();
    bool m4_ok = !m4.isEnabled();

    auto time_begin_scan = repl::time_now();
    auto min_time_to_wait = repl::sec_to_usec(0.25);
    auto timeout = repl::sec_to_usec(0.5);

    while (!m1_ok || !m2_ok || !m3_ok || !m4_ok || (repl::time_now() - time_begin_scan < min_time_to_wait)) {
        repl::sleep(0.001); // check at 1000 Hz

        if (can->canReadData()) {
            long unsigned int rxId;
            unsigned char len;
            unsigned char rxBuf[8];

            can->readMsgBuf(&rxId, &len, rxBuf);
            // Validate id
            int motor_id = rxId & 0x00F; // 0x101 for id 1, 0x102 for id 2, ...
            if (motor_id == m1.getId()) {
                m1_ok = true;
            }
            else if (motor_id == m2.getId()) {
                m2_ok = true;
            }
            else if (motor_id == m3.getId()) {
                m3_ok = true;
            }
            else { // detect unallowed motor
                OUTPUT_ERROR("Scan CAN bus : Received can frame with wrong id : %d", motor_id);
                hw_is_busy = false;
                debug_error_message = "Unallowed connected motor : ";
                debug_error_message += std::to_string(motor_id);
                OUTPUT_ERROR("%s", debug_error_message.c_str());
                return CAN_SCAN_NOT_ALLOWED;
            }
        }

        if (repl::time_now() - time_begin_scan > timeout) {
            OUTPUT_ERROR("CAN SCAN Timeout");
            debug_error_message = "CAN bus scan failed : motors ";
            if (!m1_ok) { debug_error_message += m1.getName(); debug_error_message += ", "; }
            if (!m2_ok) { debug_error_message += m2.getName(); debug_error_message += ", "; }
            if (!m3_ok) { debug_error_message += m3.getName(); debug_error_message += ", "; }
            if (!m4_ok) { debug_error_message += m4.getName(); debug_error_message += ", "; }
            debug_error_message += "are not connected";
            is_can_connection_ok = false;
            hw_is_busy = false;
            OUTPUT_ERROR("%s", debug_error_message.c_str());
            return CAN_SCAN_TIMEOUT;
        }
    }

    //ROS_INFO("CAN Connection ok");
    hw_is_busy = false;
    is_can_connection_ok = true;
    debug_error_message = "";
    return CAN_SCAN_OK;
}