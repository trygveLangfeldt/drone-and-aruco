#include "stdafx.h"
#include "Process.h"

// Process
Process::Process(mode mode, regulator reg, filter filt, cv::Vec3d sp, parameter vid, parameter mark, parameter axes) : 
	ControlMode(mode, reg, filt, sp),
	VideoParameters(vid, mark, axes)
{
	cout << "INITIALIZING PROGRAM." << endl;
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl << endl;
	Process::setSystemState(systemState::idle); // Standard system state upon program start

	// Initialize communication with Arduino
	Process::connectToArduino();

	// Initializing position/orientation vectors
	this->lastTranslationVector.push_back(cv::Vec3d(0, 0, 0));
	this->lastRotationVector.push_back(cv::Vec3d(0, 0, 0));

	cout << "-----------------------------------------------------------------------------------------------------------------" << endl << endl;
	cout << "Calibration routine..." << endl;
	std::ifstream calib_file(CALIBDATA_FILE);
	// check for existence of calibration file
	if (calib_file)
	{
		cout << CALIBDATA_FILE << " already exists.\nDo you want to use this calibration file (Y), or restart calibration (N)? (Y/n) ";
		string input;
		while (true)
		{
			std::getline(cin, input);
			if (input == "Y" || input == "y" || input.empty())
				break;
			else if (input == "N" || input == "n")
			{
				VideoParameters::startCalibration();
				break;
			}
			else
				cout << "Beg your pardon? Answer (Y/n) ";
		}
	}
	else
		VideoParameters::startCalibration();

	// Load calibration file
	VideoParameters::loadCameraCalibration();
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl << endl;

	// Commands
	this->valid_command_str = { "start", "stop", "help", "pause", "resume", "state", "vid", "markers", "axes", "webcam",  "pose", "pc",
						  "print sp", "set sp", "mode", "reg off", "pid", "mpc", "filter off", "kalman", "log" };
	this->command_description = {"Starts program.",
						   "Stops drone and halt program.",
						   "Displays this help ('h' can also be used).",
						   "Stops drone and pause program ('p' can also be used).",
						   "Resumes program after 'pause' ('r' can also be used).",
						   "Prints system state.",
						   "Turns on video if off, turn off vid if on (default on).",
						   "Turns on markers if off, turn off markers if on (default off).",
						   "Turns on axes on video if off, turn off axes on video if on (default on).",
						   "Changes camera location turns to extern if local, to local if extern. (default local)",
						   "Prints last position/orientation matrix ('P' can also be used).",
						   "Prints pose continously. Press any key to exit this mode.",
						   "Prints current setpoint",
						   "Lets the user enter the setpoint.",
						   "Goes to automatic mode if manual, to manual if automatic (default manual, 'm' can also be used).\n\tIn manual mode, use:\n\t\tx/z to increase/decrease throttle,\n\t\td/a to roll right/left,\n\t\tw/s to pitch towrds/backwards and\n\t\tq/e to yaw right/left.\n\t\tIn automatic mode, only roll pitch and yaw can be modified.\n\t\tWrite e.g. x=1500 to give step input.",
						   "Disable regulator.",
						   "Sets PID as regulator (default regulator). TODO: change PID settings.",
						   "Sets MPC as regulator.",
						   "Disables filtering (default filter).",
						   "Enables Kalman filtering.",
						   "Starts or stops data registration"};

	// Data registration
	this->logData = false;

	// Shared time-related variables
	this->markerTimer = clock();
	this->dataTimer = clock();

	cout << "PROGRAM INITIALIZED." << endl;
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl << endl;

	initiateThreads();
}
Process::~Process()
{
	// Print stop message
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl
		<< "STOPPING PROCEDURE . . ." << endl
		<< "-----------------------------------------------------------------------------------------------------------------" << endl << endl;
	// stop drone
	this->arduino->writeSerialPort(&this->droneStop[0u], MAX_DATA_LENGTH);
	// close communication
	delete this->arduino;
}
void Process::initiateThreads()
{
	// Initialize threads
	std::thread process_thread(&Process::videoProcessing, this);
	std::thread input_thread(&Process::consoleInput, this);

	// Synchronize threads
	process_thread.join();
	input_thread.join();
}
void Process::consoleInput()
{
	string input = "";
	string value_str = "";
	bool started, paused, startedOrPaused;

	// vectors for printing translation and rotation that haven't being printed yet
	vector<cv::Vec3d> lastPrintedTranslationVec, lastPrintedRotationVec;
	lastPrintedTranslationVec.push_back(this->lastTranslationVector.at(0));
	lastPrintedRotationVec.push_back(this->lastRotationVector.at(0));

	cout << "Type in 'start' to launch program, and 'help' to get help on possible commands." << endl << endl;

	// Read from terminal while system is running
	do
	{
		// Assign values to local booleans for system state
		started = (Process::getSystemState() == systemState::start);
		paused = (Process::getSystemState() == systemState::pause);
		startedOrPaused = ((Process::getSystemState() == systemState::start) || (Process::getSystemState() == systemState::pause));

		cout << "$ystem command: ";
		// Read user input
		std::getline(cin, input);

		// Execute command if valid OR if command is just one letter (com shortcuts and drone manual control)
		// OR if the input isn't empty and contains a '=' sign at place w/ index 1 (step input to drone)
		if (Process::isCommandValid(input) || (input.size() == 1) || (!input.empty() && (input[1] == '=')))
		{
			// Start
			if (input == Process::valid_command_str[0])
			{
				if (!started)
				{
					cout << ((Process::getSystemState() == systemState::pause)? "Program resumed." : "Program started.") << endl;
					Process::setSystemState(systemState::start);
				}
				else
					cout << "The program is already running." << endl;
			}
			// Stop
			else if (input == Process::valid_command_str[1])
			{
				cout << "Stop program? (Y/n) ";
				std::getline(cin, input);
				if (input.empty() || (input == "y") || (input == "Y"))
					Process::setSystemState(systemState::stop);
			}
			// Help
			else if ((input == Process::valid_command_str[2]) || (input == "h"))
				Process::displayHelp();
			// Pause
			else if (((input == Process::valid_command_str[3]) || (input == "p")) && started)
			{
				Process::setSystemState(systemState::pause);
				cout << "\tSystem paused." << endl;
			}
			// Resume
			else if (((input == Process::valid_command_str[4]) || (input == "r")) && paused)
			{
				Process::setSystemState(systemState::start);
				cout << "\tProgram resumed." << endl;
			}
			// State | command may be given on start or paused state
			else if ((input == Process::valid_command_str[5]) && startedOrPaused)
				Process::printSystemState();
			// Video on/off
			else if ((input == Process::valid_command_str[6]) && started)
			{
				if (VideoParameters::getParameter("video") == parameter::on) VideoParameters::setParameter("video", parameter::off);
				else VideoParameters::setParameter("video", parameter::on);
				cout << "\tVideo: " << ((VideoParameters::getParameter("video") == parameter::on) ? "on." : "off.") << endl;
			}
			// Markers on/off
			else if ((input == this->valid_command_str[7]) && started)
			{
				if (VideoParameters::getParameter("markers") == parameter::on) VideoParameters::setParameter("markers", parameter::off);
				else VideoParameters::setParameter("markers", parameter::on);
				cout << "\tMarkers: " << ((VideoParameters::getParameter("markers") == parameter::on) ? "on." : "off.") << endl;
			}
			// Axes on/off
			else if ((input == this->valid_command_str[8]) && started)
			{
				if (VideoParameters::getParameter("axes") == parameter::on) VideoParameters::setParameter("axes", parameter::off);
				else VideoParameters::setParameter("axes", parameter::on);
				cout << "\tAxes: " << ((VideoParameters::getParameter("axes") == parameter::on) ? "on." : "off.") << endl;
			}
			// Webcam
			else if ((input == this->valid_command_str[9]) && started)
			{
				if (VideoParameters::getCurrentWebcam() == webcam::local) { VideoParameters::setNewWebcam(webcam::external); }
				else VideoParameters::setNewWebcam(webcam::local);
				cout << "\tNew webcam: " << ((VideoParameters::getNewWebcam() == webcam::local) ? "local." : "external.") << endl;
			}
			// Pose
			else if (((input == this->valid_command_str[10]) || (input == "P")) && startedOrPaused)
			{
				// Print last saved translation and rotation vectors
				if (this->lastTranslationVector.size() > 0)
				{
					// use value of translation and rotation using lastTranslationVector.at(index)[x] f.ex.
					cout << "Position\t\t\tOrientation:" << endl;
					for (size_t i = 0; i < this->lastTranslationVector.size(); i++)
					{
						for (size_t j = 0; j < 3; j++)
						{
							cout << ((j == 0) ? "x: " : ((j == 1) ? "y: " : "z: ")) << this->lastTranslationVector.at(i).row(j)
								<< "  \ttheta " << ((j == 0) ? "x: " : ((j == 1) ? "y: " : "z: ")) << this->lastRotationVector.at(i).row(j) << endl;
						}
					}
				}
				else
					cout << "No pose to print." << endl;
			}
			// Display pose continously
			else if ((input == this->valid_command_str[11]) && started)
			{
				cout << "\tPress any key to come out of this mode." << endl;
				while (!_kbhit())
				{
					if ((lastPrintedTranslationVec.at(0) != this->lastTranslationVector.at(0)) && this->droneDetected)
					{
						cout << "t,r,p,y: " << this->newData << "\txyz: " << this->lastTranslationVector.at(0) << endl;
						for (size_t i = 0; i < (9 + this->newData.size()); i++)
							cout << " ";
						cout << "\terror_xyz: " << this->lastTranslationVector.at(0) - ControlMode::getSetPoint() << endl;
						for (size_t i = 0; i < (9 + this->newData.size()); i++)
							cout << " ";
						cout << "\ttheta_xyz: " << this->lastRotationVector.at(0) << endl << endl;
						lastPrintedTranslationVec.at(0) = this->lastTranslationVector.at(0);
						lastPrintedRotationVec.at(0) = this->lastRotationVector.at(0);
					}
				}
			}
			// Display setpoint
			else if ((input == this->valid_command_str[12]) && startedOrPaused)
				cout << "\tSetpoint vector: " << ControlMode::getSetPoint() << endl;
			// Set new setpoint
			else if ((input == this->valid_command_str[13]) && startedOrPaused)
			{
				double* pose = new double[3]; // temp var to store entered values allocated with dynamic memory, to be freed after operation
				// Give standard values to pose from setpoint
				for (size_t i = 0; i < 3; i++)
					pose[i] = ControlMode::getSetPoint()[i];
				cout << "Enter new setpoint (empty field and 'ENTER' keeps old value): " << endl;
				
				// Do it once for x, y and z
				for (size_t i = 0; i < 3; i++)
				{
					cout << "\t" << ((i == 0) ? "x" : ((i == 1) ? "y" : "z")) <<"-value: ";
					std::getline(cin, value_str);

					// Input is correct i.e. not alph
					if (Process::isInputDigit(value_str))
					{
						// If input is ENTER
						if (value_str.empty())
						{
							; // keep old value
						}
						// Else if input is a number
						else
						{
							// get new value from user input
							pose[i] = std::stod(value_str, 0);
						}
					}
					// at least one input was wrong, retrieve old set point values and break
					else
					{
						for (size_t j = 0; j < 3; j++)
							pose[j] = ControlMode::getSetPoint()[j];
						break;
					}
				}
				// If the input was wrong i.e. includes alpha
				if (!Process::isInputDigit(value_str))
				{
					cout << "ERROR: Wrong Input (maybe alph), try again." << endl;
				}
				// If the input was correct, i.e. only numbers
				else
				{
					cout << "New setpoint: ";
					// Give setpoint the values of pose, and print new sp
					ControlMode::setSetPoint(cv::Vec3d(pose[0], pose[1], pose[2]));
					cout << ControlMode::getSetPoint() << endl;
				}
				delete[] pose;
			}
			// Switch mode
			else if (((input == this->valid_command_str[14]) || (input == "m")) && startedOrPaused)
			{
				if (ControlMode::getOperatingMode() == mode::manual)
					ControlMode::setOperatingMode(mode::automatic);
				else ControlMode::setOperatingMode(mode::manual);
				cout << "\tOperating mode: " << ((ControlMode::getOperatingMode() == mode::manual) ? "manual." : "automatic.\n\tPose file is being updated, you can start Matlab controll script.") << endl;
			}
			// Regulator off
			else if ((input == this->valid_command_str[15]) && startedOrPaused)
			{
				ControlMode::setOperatingReg(regulator::regoff);
				cout << "\tRegulator off."<< endl;
			}
			// PID
			else if ((input == this->valid_command_str[16]) && startedOrPaused)
			{
				// TODO: program PID values in here
				ControlMode::setOperatingReg(regulator::pid);
				cout << "\tRegulator: PID." << endl;
			}
			// MPC
			else if ((input == this->valid_command_str[17]) && startedOrPaused)
			{
				ControlMode::setOperatingReg(regulator::mpc);
				cout << "\tRegulator: MPC." << endl;
			}
			// Filter off
			else if ((input == this->valid_command_str[18]) && startedOrPaused)
			{
				ControlMode::setOperatingFilter(filter::filteroff);
				cout << "\tFilter off." << endl;
			}
			// Kalman
			else if ((input == this->valid_command_str[19]) && startedOrPaused)
			{
				ControlMode::setOperatingFilter(filter::kalman);
				cout << "\tFilter: Kalman." << endl;
			}
			// Log/Don't log data
			else if ((input == this->valid_command_str[20]) && started)
			{
				if (this->logData) this->logData = false;
				else this->logData = true;
				cout << ((this->logData)? "\tLogging data." : "\tNot logging data.") << endl;
			}
			// When the input is a type 'x=1500', register the '=', the first letter (info on which input to step), and the value
			else if ((input[1] == '=') && started)
			{
				string cmd;
				cmd.insert(0, input, 2, input.size() - 2); // copy the rest of the input after '=' to cmd string
				if (((input[0] == 'x') || (input[0] == 'z')) && (ControlMode::getOperatingMode() == mode::manual))
					ControlMode::setControlValue("throttle", std::stoi(cmd));
				else if ((input[0] == 'd') || (input[0] == 'a'))
					ControlMode::setControlValue("roll", std::stoi(cmd));
				else if ((input[0] == 'w') || (input[0] == 's'))
					ControlMode::setControlValue("pitch", std::stoi(cmd));
				else if ((input[0] == 'e') || (input[0] == 'q'))
					ControlMode::setControlValue("yaw", std::stoi(cmd));

				// in this case can newData be accessed and written by both this thread and the controller thread, we want to reserve this ressurse for the writing periode
				// exclusive access to newData signaled by locking mu
				Process::mu.lock();
				this->newData = ControlMode::convertControlValuesToStr();
				Process::mu.unlock();
				cout << "\tData sent: " << this->newData << endl;
			}
			// Commands to control drone manually with x/z, w/s, q/e and a/d
			else if (started)
			{
				if ((input == "x") && (ControlMode::getOperatingMode() == mode::manual)) ControlMode::setControlValue("throttle", ControlMode::getControlValue("throttle") + 25);
				else if ((input == "z") && (ControlMode::getOperatingMode() == mode::manual)) ControlMode::setControlValue("throttle", ControlMode::getControlValue("throttle") - 25);
				else if (input == "d") ControlMode::setControlValue("roll", ControlMode::getControlValue("roll") + 25);
				else if (input == "a") ControlMode::setControlValue("roll", ControlMode::getControlValue("roll") - 25);
				else if (input == "w") ControlMode::setControlValue("pitch", ControlMode::getControlValue("pitch") + 25);
				else if (input == "s") ControlMode::setControlValue("pitch", ControlMode::getControlValue("pitch") - 25);
				else if (input == "e") ControlMode::setControlValue("yaw", ControlMode::getControlValue("yaw") + 25);
				else if (input == "q") ControlMode::setControlValue("yaw", ControlMode::getControlValue("yaw") - 25);
				// reset throttle
				else if (input == "c") ControlMode::resetControlValue("throttle");
				// reset values
				else if (input == "r") ControlMode::resetAllControlValues();

				// in this case can newData be accessed and written by both this thread and the controller thread, we want to reserve this ressurse for the writing periode
				// exclusive access to newData signaled by locking mu
				Process::mu.lock();
				this->newData = ControlMode::convertControlValuesToStr();
				Process::mu.unlock();
				cout << "\tData sent: " << this->newData << endl;
			}
			// Unvalid command while system is paused
			else if (paused)
			{
				cout << "System paused, general commands are disabled. Type in" << endl
					<< "\t- 'resume' to resume process," << endl
					<< "\t- 'help' to get help," << endl
					<< "\t- 'state' to get system state," << endl
					<< "\t- 'stop' to halt program." << endl;
			}
			// Unvalid command while system hasn't started yet
			else if (Process::getSystemState() == systemState::idle)
				cout << "\tWaiting for 'start' command." << endl;
			// Just in case
			else
				cout << "\tUnrecognized command. Type in 'help' to get a list of available commands." << endl;
		}
		// ENTER was pressed without an input
		else if (input.empty()) { ; }
		// Print error message if not
		else
			cout << "\tUnrecognized command. Type in 'help' to get a list of available commands." << endl;
	} while (Process::getSystemState() != systemState::stop);
}
bool Process::isInputDigit(string input)
{
	std::locale loc;
	bool inputIsDigit = true;
	// Check for unvalid alpha inputs
	for (std::string::iterator it = input.begin(); it != input.end(); ++it)
		if (!std::isdigit(*it, loc) && *it != '.')
			inputIsDigit = false;
	return inputIsDigit;
}
bool Process::isCommandValid(string command)
{
	bool valid_cmd = false;
	// Check for valid command
	for (vector<string>::iterator it = this->valid_command_str.begin(); it != this->valid_command_str.end(); ++it)
		if (command == *it)
		{
			valid_cmd = true;
			break;
		}
	return valid_cmd;
}
void Process::videoProcessing()
{
	// Wait until the program has started
	while (Process::getSystemState() == systemState::idle) { ; }
	
	// Initialize all variables if user doesn't request to stop the program even before staring video
	if (Process::getSystemState() == systemState::start)
	{
		this->loopTimer = clock(); // clock to measure loop time
		std::fstream logFile; // File used to log input, pose, setpoint and error
		std::fstream poseFile; // File use to communicate to matlab run, reg and pose
		std::ifstream trpyFile;// file containing throttle, roll, pitch and yaw calculated in matlab

		Process::writeToFile(poseFile, POSE_FILE, false, true, false, true, false, true);

		cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
		vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::aruco::DetectorParameters parameters;
		vector<int> markerIds;

		vector<cv::Vec3d> rotationVector, translationVector; // vectors for continuous detection of translation and rotation

		cv::Mat frame;
		cv::VideoCapture vid(static_cast<int>(VideoParameters::getCurrentWebcam()));
		cv::namedWindow(WEBCAM_WINDOW, CV_WINDOW_AUTOSIZE);
		if (!vid.isOpened()) return;

		while (Process::getSystemState() != systemState::stop)
		{
			if (VideoParameters::getNewWebcam() != VideoParameters::getCurrentWebcam())
			{
				VideoParameters::setCurrentWebcam(VideoParameters::getNewWebcam());
				vid.open(VideoParameters::getCurrentWebcam());
				if (!vid.isOpened()) return;
			}

			// Go through video processing if system isn't paused (or stopped)
			if (Process::getSystemState() == systemState::start)
			{
				if (!vid.read(frame)) return;

				//detectMarkers detects all possible markers
				cv::aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds, cv::aruco::DetectorParameters::create(), rejectedCandidates);

				// Start web cam if vid is on
				if (VideoParameters::getParameter("video") == parameter::on)
				{
					if (!vid.isOpened()) vid.open(VideoParameters::getCurrentWebcam());

					// Draws all attempts to detect markers
					if (VideoParameters::getParameter("markers") == parameter::on)
						cv::aruco::drawDetectedMarkers(frame, rejectedCandidates);

					// If detected marker is the drone marker
					for (size_t i = 0; i < markerIds.size(); i++)
					{
						if (markerIds.at(i) == this->droneMarker)
						{
							this->markerTimer = clock(); // update timer
							this->droneDetected = true; // set bool for detected drone
							// Calculates rotation and translation vectors
							cv::aruco::estimatePoseSingleMarkers(markerCorners, QR_CODE_SIZE, cameraMatrix, distanceCoeff, rotationVector, translationVector);

							// Draws the axis on the marker
							if (VideoParameters::getParameter("axes") == parameter::on)
								cv::aruco::drawAxis(frame, this->cameraMatrix, this->distanceCoeff, rotationVector.at(0), translationVector.at(0), QR_CODE_SIZE);
						}
						else this->droneDetected = false; // set bool for detected drone
					}
					cv::imshow(WEBCAM_WINDOW, frame);
					cv::waitKey(1);
				}
				// Keep estimating pose while vid id off
				else
				{
					if (vid.isOpened()) cv::destroyWindow(WEBCAM_WINDOW);

					// Calculates rotation and translation vectors if dected marker is drone marker
					for (size_t i = 0; i < markerIds.size(); i++)
					{
						if (markerIds.at(i) == this->droneMarker)
						{
							this->markerTimer = clock(); // update timer
							this->droneDetected = true; // set bool for detected drone
							cv::aruco::estimatePoseSingleMarkers(markerCorners, QR_CODE_SIZE, this->cameraMatrix, this->distanceCoeff, rotationVector, translationVector);
						}
						else this->droneDetected = false; // set bool for detected drone
					}
				}
			}
			// Update position variable
			if ((translationVector != this->lastTranslationVector) & (translationVector.size() > 0))
				this->lastTranslationVector = translationVector;
			// Update rotation variable
			if ((rotationVector != this->lastRotationVector) & (rotationVector.size() > 0))
				this->lastRotationVector = rotationVector;

			Process::controller(poseFile, logFile, trpyFile);
		}
		// close log file
		logFile.close();
		// Procedure to send stop signal to matlab (run = 0 because system_state = stop)
		Process::writeToFile(poseFile, POSE_FILE, false, true, false, true, false, true);

		// Little routine to get rid of possible unwanted empty lines in the log file
		std::ofstream tempFile;
		string line;
		logFile.open(LOG_FILE, std::ifstream::in);
		tempFile.open("temp.csv", std::ofstream::out);
		int* i = new int;
		*i = 0;
		while (std::getline(logFile, line))
		{
			if (!line.empty())
			{
				if (i == 0)
					tempFile << line; // the first line of the new file is just the first line of the old one, wo endl
				else tempFile << endl << line; // by setting the endl before printing the line, we avoid getting a final empty line
			}
			*i += 1;
		}
		delete i;
		tempFile.close();
		logFile.close();
		remove(LOG_FILE);
		rename("temp.csv", LOG_FILE);
	}
}
void Process::controller(std::fstream& pose, std::fstream& logfile, std::ifstream& trpyfile)
{
	// Open log file if it is closed when the command to log data has been given
	if (this->logData && !logfile.is_open())
	{
		// file containing log information on all registered position, orientation
		logfile.open("drone_log.csv", std::ofstream::out);
	}
	// When system is paused, stop drone
	if ((Process::getSystemState() == systemState::pause) && Process::isReady(this->dataTimer, DELAY_BETWEEN_DATA) && Process::isDroneFlying())
	{
		// Write data to be used in matlab
		Process::writeToFile(pose, POSE_FILE, false, true, false, true, false, true);
		this->arduino->writeSerialPort(&this->droneStop[0u], MAX_DATA_LENGTH);
		this->dataTimer = clock();
	}
	// When system is running
	else if (Process::getSystemState() == systemState::start)
	{
		// Assess if the video process detects the drone, and if we are in automatic mode
		if (ControlMode::getOperatingMode() == mode::automatic)
		{
			// Read from trpy file to send t,r,p and y to drone
			if (this->droneDetected)
			{
				// variable to read from csv file
				string line;
				trpyfile.open(TRPY_FILE, std::ifstream::in); // Open file and read content
				while (std::getline(trpyfile, line))
				{
					Process::mu.lock();
					this->newData = line;
					Process::mu.unlock();
				}
				trpyfile.close();
			}
			// Send stop command to drone if out of reach
			else if (!this->droneDetected && Process::isReady(this->markerTimer, MARKER_TIMEOUT) && Process::isDroneFlying())
			{
				Process::mu.lock();
				this->newData = ControlMode::droneStop;
				Process::mu.unlock();
			}
		}
		if (Process::isReady(this->dataTimer, DELAY_BETWEEN_DATA))
		{
			// Write in the log file
			if (this->logData)
				Process::writeToFile(logfile, LOG_FILE, true, false, true, true, true, false);

			// Write data to be used in matlab
			Process::writeToFile(pose, POSE_FILE, false, true, false, true, false, true);
				
			// Check for valid data to send
			if (this->oldData != this->newData)
			{
				// Send to arduino & update oldData variable
				this->arduino->writeSerialPort(&this->newData[0u], MAX_DATA_LENGTH);
				this->oldData = this->newData;
			}
			// Reset data Timer clock (lets us know when we can send to arduino)
			this->dataTimer = clock();
		}
	}
}
void Process::connectToArduino()
{
	cout << "Welcome to the PC-to-Arduino interface." << endl;
	string port, input = "";

	while (true)
	{
		port = "COM";
		cout << "\nEnter COM port number your arduino is connected to: " << port;
		std::getline(cin, input);

		if (Process::isInputDigit(input))
		{
			int* int_input; // int input is only used here, allocate using dynamic memory
			int_input = new int;
			*int_input = std::stoi(input, 0); // convert input to int

			if (*int_input > 9) { port = "\\\\.\\COM"; } // If port number is bigger than 9, the backslashes have to be included
			port += input; // gives COM<X> or COM<XX>

			cout << "Waiting for connection with arduino . . ." << endl;
			this->arduino = new SerialPort(&port[0u]); // creates connection with arduino at given port 
			if (this->arduino->isConnected())
			{
				cout << "Arduino connected at port " << port << "." << endl;
				break;
			}
			else
			{
				cout << "Arduino not found . . ." << endl;
				delete this->arduino;
			}
			delete int_input; // free dynamic memory
		}
		else
			cout << "ERROR: Input is not digit! COM port has to be a number" << endl;
	}
	cout << "Waiting for connection with drone . . ." << endl;
	// Waiting for feedback from Arduino
	while (true)
	{
		char feedback[MAX_DATA_LENGTH];
		int bytes_in = this->arduino->readSerialPort(feedback, MAX_DATA_LENGTH);
		if (bytes_in > 0) break;
	}
	cout << "The drone should now be connected to arduino." << endl
		<< "At any time, if the drone is disconnected, restart the drone, and press the restart button on arduino." << endl
		<< "This will not affect the process. Wait for the drone to be connected before sending data through Serial port." << endl << endl;
}
bool Process::isDroneFlying()
{
	if (ControlMode::getControlValue("throttle") > 1000) return true;
	else return false;
}
void Process::writeToFile(std::fstream& file, string fileName, bool pClock,
	bool pState, bool pThrottle, bool pRPY, bool pEndl, bool openCloseFile)
{
	if (openCloseFile)
		file.open(fileName, std::fstream::out);
	if (pClock)
		file << ((float)(clock() - this->loopTimer)) / CLOCKS_PER_SEC << ",";
	if (pState)
		file << static_cast<int>(Process::getSystemState()) << "," << static_cast<int>(ControlMode::getOperatingMode()) << "," << static_cast<int>(ControlMode::getOperatingReg()) << "," << static_cast<int>(ControlMode::getOperatingFilter()) << ",";
	if (pThrottle)
		file << ControlMode::getControlValue("throttle") << ",";
	if (pRPY)
		file << ControlMode::getControlValue("roll") << "," << ControlMode::getControlValue("pitch") << "," << ControlMode::getControlValue("yaw") << ",";
	for (size_t i = 0; i < 3; i++)
		file << this->lastTranslationVector.at(0)[i] << ",";
	for (size_t i = 0; i < 3; i++)
		file << this->lastRotationVector.at(0)[i] << ",";
	for (size_t i = 0; i < 3; i++)
		file << ControlMode::getSetPoint()[i] << ((i == 2) ? "" : ",");
	if (pEndl)
		file << endl;
	if (openCloseFile)
		file.close();
}
bool Process::isReady(clock_t currentTime, float delay)
{
	if (((float)(clock() - currentTime) * 1000.f / CLOCKS_PER_SEC) >= delay) return true;
	else return false;
}
void Process::printSystemState()
{
	cout << endl << "-----------------------------------------------------------------------------------------------------------------" << endl 
		<< "System state:" << endl;
	switch (this->system_state)
	{
		case systemState::idle:
			cout << "\tIdle - waiting for start command." << endl;
			break;
		case systemState::start:
			cout << "\tProgram running." << endl;
			cout << ((logData) ? "\tLogging data." : "\tNot logging data.") << endl;
			ControlMode::printControlState();
			VideoParameters::printVideoState();
			break;
		case systemState::stop:
			cout << "\tStop sequence initiated...\n\n" << endl;
			break;
		case systemState::pause:
			cout << "\tPaused - type in 'resume' to resume process." << endl;
			ControlMode::printControlState();
			VideoParameters::printVideoState();
			break;
	}
	cout << "-----------------------------------------------------------------------------------------------------------------\n" << endl;
}
void Process::displayHelp()
{
	// This first little loop is just a trick to get a clean console output
	size_t max_size = 0;
	for (size_t i = 0; i < this->valid_command_str.size(); i++)
	{
		if (this->valid_command_str.at(i).size() > max_size)
			max_size = this->valid_command_str.at(i).size();
	}

	cout << endl << "-----------------------------------------------------------------------------------------------------------------" << endl;
	cout << "HELP\nList of valid commands:" << endl;
	// Check integrity of data. If a problem comes from this part, check
	// valid_command_str and command_description from Process::Process()
	if (this->valid_command_str.size() != this->command_description.size())
		cout << "ERROR! valid_command_str and command_description_str have different sizes!" << endl;
	// Print Help from variables initialized in Process::Process()
	else
		for (size_t i = 0; i < this->valid_command_str.size(); ++i)
		{
			if (i == 0) cout << "System commands:" << endl;
			else if (i == 6) cout << "Video commands:" << endl;
			else if (i == 10) cout << "Pose commands:" << endl;
			else if (i == 14) cout << "Control mode and regulator commands:" << endl;
			cout << "\t- '" << this->valid_command_str[i] << "' ";
			for (size_t j = 0; j < (max_size - this->valid_command_str.at(i).size()); j++)
			{
				cout << " ";
			}
			cout << " " << this->command_description[i] << endl;
		}

	cout << "-----------------------------------------------------------------------------------------------------------------" << endl << endl;
}