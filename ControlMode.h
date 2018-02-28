#pragma once

#ifndef CONTROLMODE_H
#define CONTROLMODE_H

#include "stdafx.h"

#define MAX_CONTROL_VALUE 2000
#define MIN_CONTROL_VALUE 1000
#define THROTTLE_DEF 1000
#define ROLL_DEF 1500
#define PITCH_DEF 1500
#define YAW_DEF 1500

// Enumeration to store operating modes
enum mode
{
	manual = 0,
	automatic = 1,
};
// Enumeration to store regulators
enum regulator
{
	regoff = 0,
	pid = 1,
	mpc = 2
};
// Enumeration to store filters
enum filter
{
	filteroff = 0,
	kalman = 1
};

/*
Class to initiate, store and handle control modes
*/
class ControlMode
{
	public:
		/*
		@default operating mode
		@default operating regulator
		@default operating filter
		@default setpoint cv::Vec3d(double, double, double)
		Constructor of the class (initiate default values)
		*/
		ControlMode(mode, regulator, filter, cv::Vec3d);
		void printControlState(); // Prints control mode state
		mode getOperatingMode() { return this->operating_mode;  } // Returns current operating mode

		/*
		@operating mdoe enum value
		Sets value for current operating mode
		*/
		void setOperatingMode(mode mode) { this->operating_mode = mode; }
		regulator getOperatingReg() { return this->operating_reg;  } // Returns current operating regulator

		/*
		@regulator enum value
		Sets value for current operating regulator
		*/
		void setOperatingReg(regulator reg) { this->operating_reg = reg;  }
		filter getOperatingFilter() { return this->operating_filt; } // Returns current operating filter

		/*
		@filter enum value
		Sets value for current operating filter
		*/
		void setOperatingFilter(filter filt) { this->operating_filt = filt;  }
		void resetAllControlValues(); // Resets all control values

		/*
		@name of control value "throttle", "roll", "pitch" or "yaw"
		Resets a specific control value
		*/
		void resetControlValue(string);

		/*
		@name of control value "throttle", "roll", "pitch" or "yaw"
		Returns a specific control value
		*/
		int getControlValue(string);

		/*
		@name of control value "throttle", "roll", "pitch" or "yaw"
		@value to assign
		Sets a specific control value
		*/
		void setControlValue(string, int);

		/*
		@throttle
		@roll
		@pitch
		@yaw
		Sets all control values to given int values
		*/
		void setAllControlValues(int, int, int, int);
		cv::Vec3d getSetPoint() { return this->setpoint;  } // Returns current setpoint

		/*
		@setpoint cv::Vec3d(double, double, double)
		Sets new setpoint to given values
		*/
		void setSetPoint(cv::Vec3d newSetPoint) { this->setpoint = newSetPoint;  }
		string convertControlValuesToStr(); // Routine to convert curent control values to string
	protected:
		string droneStop; // String to stop drone
		string oldData, newData; // Strings to update commands to send to the drone

	private:
		mode operating_mode; // Operating mode (manual/automatic)
		regulator operating_reg; // Operating regulator (none/PID/MPC)
		filter operating_filt; // Operating filter (none/Kalman)
		
		int throttle, roll, pitch, yaw; // Controller values
		cv::Vec3d setpoint; // vector to store the setpoint (objective)
};

#endif // CONTROLMODE_H