#pragma once

#ifndef PROCESS_H
#define PROCESS_H

#include "VideoParameters.h"
#include "ControlMode.h"
#include "SerialPort.h"

#define MARKER_TIMEOUT 2000.f
#define DELAY_BETWEEN_DATA 30.f
#define POSE_FILE "pose.csv"
#define LOG_FILE "drone_log"
#define TRPY_FILE "trpy.csv"

// Enumeration to store system states
enum systemState
{
	stop = 0,
	start = 1,
	pause = 2,
	idle = 3,
};

/*
	General class to process data and control drone
	Inherites from both ControlMode and VideoParameters
*/
class Process : private ControlMode, private VideoParameters
{
	public:
		/*
		@ default operating mode from mode enum
		@ default operating regulator from regulator enum
		@ default operating filter from filter enum
		@ default setpoint
		@ default video parameter from parameter enum
		@ default markers parameter from parameter enum
		@ default axes parameter from parameter enum
		Constructor of the class
		*/
		Process(mode, regulator, filter, cv::Vec3d, parameter, parameter, parameter);
		// destructor of the class, Routine to free dynamic memory and close the program
		~Process();
		// Starts and joins threads
		void initiateThreads();
		// Routine to read from console/terminal and update global variables
		void consoleInput(); 

		/*
		@string input to verify if is digit
		Returns true if input is digit
		*/
		bool isInputDigit(string);

		/*
		@command to assess
		Returns true if command is valid
		*/
		bool isCommandValid(string);
		// Routine to read ArUco markers from camera and update var
		void videoProcessing();

		/*
			@ pose fil to write to
			@ log file to write to
			@ trpy file to read from
			Controller function
		*/
		void controller(std::fstream&, std::fstream&, std::ifstream&);
		// Routine to open connection to arduino (and drone)
		void connectToArduino();
		// Returns true if last command gotten by the drone is > 1000
		bool isDroneFlying();
		/*
			@ file to write to
			@ name of file
			@ list of bool, true to write:
				@ clock
				@ state (run, mode, reg & filt)
				@ throttle
				@ RPY
				@ endl
				@ open and close file
			Writes process variables to a file
		*/
		void writeToFile(std::fstream&, string, bool, bool, bool, bool, bool, bool);
		/*
			@ clock time
			@ delay in ms
			Returns true if clock time is bigger or equal delay time
		*/
		bool isReady(clock_t, float);
		// Routine to print system state
		void printSystemState();
		// Returns system state	
		systemState getSystemState() { return this->system_state; }
		/*
		@ state to set system in
		Sets system state
		*/
		void setSystemState(systemState state) { this->system_state = state;  }
		// Displays help message.
		void displayHelp();

	private:
		systemState system_state; // Variable to store current system state

		SerialPort* arduino; // Arduino port to communicate with
		std::mutex mu; // Variable to reserve the access of ressources between threads
		
		// Position/orientation var
		vector<cv::Vec3d> lastRotationVector, lastTranslationVector; // vectors to store last valid translation and rotation		

		vector<string> valid_command_str; // String that contains all valid commands
		vector<string> command_description; // String that contains description of the commands
		
		bool logData; // Bool, true to register data

		clock_t markerTimer; // Clock to register time when marker detected
		clock_t dataTimer; // Clock to store time between sent data to arduino
		clock_t loopTimer; // Clock to time videoProcessing loop and write time to log file
};
#endif // PROCESS_H