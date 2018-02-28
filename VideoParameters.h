#pragma once

#ifndef VIDEOPARAMETERS_H
#define VIDEOPARAMETERS_H

#include "stdafx.h"

#define CALIBRATION_SQUARE_DIM 0.026f // length of a square on the chess board used for calibration
#define CHESS_BOARD_DIM cv::Size(6, 9) // number of squares on the chessboard
#define QR_CODE_SIZE 0.066f // size of the side of the QR code
#define CALIBDATA_FILE "calibdata.txt" // name of the calibration file
#define WEBCAM_WINDOW "Webcam feed" // name of webcam window

// Enumeration to store parameter state
enum parameter
{
	on,
	off
};
enum webcam
{
	local = 0,
	external = 1
};

/*
Class to initiate, store and handle video parameters using Open CV
*/
class VideoParameters
{
	public:
		/*
		@default video parameter
		@default marker parameter
		@default axes parameter
		Constructor of the class (sets default values)
		*/
		VideoParameters(parameter, parameter, parameter);
		void printVideoState(); // Prints video param state

		/*
		@name of parameter "video", "markers" or "axes"
		Returns the parameter enum value
		*/
		parameter getParameter(string);

		/*
		@name of parameter "video", "markers" or "axes"
		@enum value to set parameter to
		Sets the parameter enum value
		*/
		void setParameter(string, parameter);	
		int startCalibration(); // Creates a calibration textfile with parameters for the distance coefficients.
		bool loadCameraCalibration(); // Loads the calibration textfile with parameters for the distance coefficients.
		bool saveCameraCalibration(); // Saves calibration to file

		/*
		@images vector
		@vector of all corner found
		@show results (false)
		Finds the corner of the board
		*/
		void getBoardCorners(vector<cv::Mat>, vector<vector<cv::Point2f>>&, bool);

		/*
		@marker number
		Creates ArUco marker
		*/
		void createArucoMarkers(int num);

		/*
		@corners
		Creates the pose of the board
		*/
		void createKnownBoardPosition(vector<cv::Point3f>&);

		/*
		@calibration images matrix
		Computes camera calibration
		*/
		void cameraCalibration(vector<cv::Mat>);
		// Sets new webcam value
		void setNewWebcam(webcam webcam) { this->newWebcam = webcam; }
		// Returns new webcam value
		webcam getNewWebcam() { return this->newWebcam; }
		// Sets current webcam value
		void setCurrentWebcam(webcam webcam) { this->currentWebcam = webcam; }
		// Returns current webcam value
		webcam getCurrentWebcam() { return this->currentWebcam; }

	protected:
		cv::Mat distanceCoeff; // Distance coefficient matrix,
		cv::Mat cameraMatrix; // Camera matrix

		int droneMarker; // #ID of the specific marker used on the drone
		bool droneDetected; // boolean true if drone is detected

	private:
		parameter video, markers, axes; // video parameters
		webcam currentWebcam, newWebcam; // webcam parameter
};

#endif // VIDEOPARAMETERS_H