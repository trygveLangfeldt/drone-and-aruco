/*
* Author: Manash Kumar Mandal
* Modified Library introduced in Arduino Playground which does not work
* This works perfectly
* LICENSE: MIT
*/
#pragma once

#ifndef SERIALPORT_H
#define SERIALPORT_H

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#include "stdafx.h"

class SerialPort
{
	public:
		/*
			@pointer to char var for port name of type COM<X> or \\\\.\\COM<XX>
			Constructor of the class, initiate communication with Arduino on a given port
		*/
		SerialPort(char *portName);
		~SerialPort(); // destructor of the class

		/*
			@pointer to char var to send - string to char*: &string[0u]
			@buffer size, use MAX_DATA_LENGTH defined here
			Function to read from the serial port
		*/
		int readSerialPort(char *buffer, unsigned int buf_size);

		/*
			@pointer to char var to send - string to char*: &string[0u]
			@buffer size, use MAX_DATA_LENGTH defined here
			Function to write to the serial port
		*/
		bool writeSerialPort(char *buffer, unsigned int buf_size);

		/*
			Boolean function, returns true if arduino is connected
		*/
		bool isConnected();

	private:
		HANDLE handler;
		bool connected;
		COMSTAT status;
		DWORD errors;
		unsigned int toRead;
};

#endif // SERIALPORT_H