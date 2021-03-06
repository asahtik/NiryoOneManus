/*
    stepper_motor_state.h
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

#include <string>

#include "ros_replacements/ros_time_repl.h"

#ifndef NIRYO_STEPPER_MOTOR_STATE_H
#define NIRYO_STEPPER_MOTOR_STATE_H

class StepperMotorState {

    public:

        StepperMotorState() { }
        StepperMotorState(const std::string name, int id, double gear_ratio, double direction, 
                int32_t home_position, int32_t offset_position, uint8_t micro_steps, uint8_t max_effort) {
            this->name = name;
            this->id = id;
            this->gear_ratio = gear_ratio;
            this->direction = direction;
            this->home_position = home_position;
            this->offset_position = offset_position;
            
            is_enabled = false;
            
            cmd_micro_steps = micro_steps;
            cmd_max_effort = max_effort;

            time_last_read = *new repl::Time();

            firmware_version = "0.0.0";
            resetState();
            resetCommand();
        }
        
        void resetState() {
            state_pos = home_position;
            state_vel = 0;
            state_torque = 0;
            state_temperature = 0;
            state_hw_error = 0;
            hw_fail_counter = 0;
        }

        void resetCommand() {
            cmd_pos = home_position;
            cmd_vel = 0;
            cmd_torque = 0;
        }

        // motor properties
        int getId()                     { return id; }
        std::string getFirmwareVersion(){ return firmware_version; }
        double getGearRatio()           { return gear_ratio; }
        double getDirection()           { return direction; }
        std::string getName()           { return name; }
        bool isEnabled()                { return is_enabled; }
        repl::Time getLastTimeRead()        { return time_last_read; }
        int getHwFailCounter()          { return hw_fail_counter; }
        int32_t getHomePosition()       { return home_position; }
        int32_t getOffsetPosition()     { return offset_position; } 
        
        void setFirmwareVersion(std::string v) { firmware_version = v; }
        void setGearRatio(double ratio) { gear_ratio = ratio; } 
        void setDirection(double dir)   { direction = dir; } 
        void enable()                   { is_enabled = true; }
        void disable()                  { is_enabled = false; }
        void setLastTimeRead(repl::Time t)  { time_last_read = t; }
        void setHwFailCounter(int c)    { hw_fail_counter = c; }

        // getters - state
        int32_t getPositionState()      { return state_pos; }
        int32_t getVelocityState()      { return state_vel; }
        int32_t getTorqueState()        { return state_torque; }
        int32_t getTemperatureState()   { return state_temperature; }
        int32_t getHardwareErrorState() { return state_hw_error; }

        // setters - state
        void setPositionState(int32_t pos)     { state_pos = pos; }
        void setVelocityState(int32_t vel)     { state_vel = vel; }
        void setTorqueState(int32_t torque)    { state_torque = torque; }
        void setTemperatureState(int32_t temp) { state_temperature = temp; }
        void setHardwareError(int32_t error)   { state_hw_error = error; }

        // getters - command
        int32_t getPositionCommand()      { return cmd_pos; }
        int32_t getVelocityCommand()      { return cmd_vel; }
        int32_t getTorqueCommand()        { return cmd_torque; }
        uint8_t getMicroStepsCommand()    { return cmd_micro_steps; }
        uint8_t getMaxEffortCommand()     { return cmd_max_effort; }

        // setters - command
        void setPositionCommand(int32_t pos)     { cmd_pos = pos; } 
        void setVelocityCommand(int32_t vel)     { cmd_vel = vel; }
        void setTorqueCommand(int32_t torque)    { cmd_torque = torque; }
        void setMicroStepsCommand(uint8_t micro) { cmd_micro_steps = micro; }
        void setMaxEffortCommand(uint8_t max)    { cmd_max_effort = max; }
    private:
    
        std::string name;
        int id;
        std::string firmware_version;
        double gear_ratio;
        int32_t offset_position;
        int32_t home_position;
        double direction; 
        bool is_enabled;
        
        repl::Time time_last_read; // used for ping purpose
        int hw_fail_counter; // keeps consecutive ping failures

        int32_t state_pos;
        int32_t state_vel;
        int32_t state_torque;
        int32_t state_temperature;
        int32_t state_hw_error;

        int32_t cmd_pos;
        int32_t cmd_vel;
        int32_t cmd_torque;

        uint8_t cmd_micro_steps;
        uint8_t cmd_max_effort;
};

#endif
