#include "stdafx.h"
#include "ControlMode.h"

// Control mode
ControlMode::ControlMode(mode mode, regulator reg, filter filt, cv::Vec3d sp)
{
	this->operating_mode = mode;
	this->operating_reg = reg;
	this->operating_filt = filt;

	ControlMode::resetAllControlValues(); // throttle, roll, pitch and yaw set to default

	this->droneStop = ControlMode::convertControlValuesToStr(); // Create a standard string to stop drone
	this->newData = this->droneStop;
	this->oldData = this->newData;
	ControlMode::setSetPoint(sp); // Set default setpoint
}
void ControlMode::printControlState()
{
	cout << "Control parameters:" << endl;
	cout << "\t- t,r,p,y: " << this->newData << endl;
	cout << "\t- Operating mode: " << ((ControlMode::getOperatingMode() == mode::manual) ? "manual" : "automatic") << endl;
	cout << "\t- Regulator" << ((ControlMode::getOperatingMode() == mode::manual) ? " (unactive: manual mode): " : ": ");
	switch (ControlMode::getOperatingReg())
	{
	case regulator::regoff:
		cout << "OFF" << endl;
		break;
	case regulator::pid:
		cout << "PID" << endl;
		break;
	case regulator::mpc:
		cout << "MPC" << endl;
		break;
	default:
		cout << endl;
		break;
	}
	cout << "\t- Filter" << ((ControlMode::getOperatingMode() == mode::manual) ? " (unactive: manual mode): " : ": ");
	switch (ControlMode::getOperatingFilter())
	{
	case filter::filteroff:
		cout << "OFF" << endl;
		break;
	case filter::kalman:
		cout << "Kalman filter" << endl;
		break;
	default:
		cout << endl;
		break;
	}
}
void ControlMode::resetAllControlValues()
{
	this->throttle = THROTTLE_DEF;
	this->roll = ROLL_DEF;
	this->pitch = PITCH_DEF;
	this->yaw = YAW_DEF;
}
void ControlMode::resetControlValue(string controlValue)
{
	// Reset value
	if (controlValue == "throttle")
		this->throttle = THROTTLE_DEF;
	else if (controlValue == "roll")
		this->roll = ROLL_DEF;
	else if (controlValue == "pitch")
		this->pitch = PITCH_DEF;
	else if (controlValue == "yaw")
		this->yaw = YAW_DEF;
}
int ControlMode::getControlValue(string controlValue)
{
	if (controlValue == "throttle")
		return this->throttle;
	else if (controlValue == "roll")
		return this->roll;
	else if (controlValue == "pitch")
		return this->pitch;
	else if (controlValue == "yaw")
		return this->yaw;
}
void ControlMode::setControlValue(string controlValue, int value)
{
	// Control limit values
	if (value > MAX_CONTROL_VALUE) value = MAX_CONTROL_VALUE;
	else if (value < MIN_CONTROL_VALUE) value = MIN_CONTROL_VALUE;
	// Assign value
	if (controlValue == "throttle")
		this->throttle = value;
	else if (controlValue == "roll")
		this->roll = value;
	else if (controlValue == "pitch")
		this->pitch = value;
	else if (controlValue == "yaw")
		this->yaw = value;
}
void ControlMode::setAllControlValues(int t, int r, int p, int y)
{
	// Control limit values
	if (t > MAX_CONTROL_VALUE) t = MAX_CONTROL_VALUE;
	else if (t < MIN_CONTROL_VALUE) t = MIN_CONTROL_VALUE;
	if (r > MAX_CONTROL_VALUE) t = MAX_CONTROL_VALUE;
	else if (r < MIN_CONTROL_VALUE) t = MIN_CONTROL_VALUE;
	if (p > MAX_CONTROL_VALUE) t = MAX_CONTROL_VALUE;
	else if (p < MIN_CONTROL_VALUE) t = MIN_CONTROL_VALUE;
	if (y > MAX_CONTROL_VALUE) t = MAX_CONTROL_VALUE;
	else if (y < MIN_CONTROL_VALUE) t = MIN_CONTROL_VALUE;

	this->throttle = t;
	this->roll = r;
	this->pitch = p;
	this->yaw = y;
}
string ControlMode::convertControlValuesToStr()
{
	std::stringstream convert;
	convert << this->throttle << "," << this->roll << "," << this->pitch << "," << this->yaw;
	return convert.str();
	convert.str("");
}