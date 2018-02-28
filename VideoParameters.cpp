#include "stdafx.h"
#include "VideoParameters.h"

// Video parameters
VideoParameters::VideoParameters(parameter vid, parameter mark, parameter axes)
{
	this->video = vid;
	this->markers = mark;
	this->axes = axes;

	VideoParameters::setCurrentWebcam(webcam::external);
	VideoParameters::setNewWebcam(VideoParameters::getCurrentWebcam());

	this->droneMarker = 2;
	this->droneDetected = false;
}
void VideoParameters::printVideoState()
{
	parameter vidState = VideoParameters::getParameter("video");
	cout << "Video parameters:" << endl;
	cout << "\t- Video: " << ((vidState == parameter::on) ? "ON" : "OFF") << endl;
	cout << "\t- Markers" << ((vidState == parameter::off) ? " (video OFF): " : ": ") << ((VideoParameters::getParameter("markers") == parameter::on) ? "ON" : "OFF") << endl;
	cout << "\t- Axes:" << ((vidState == parameter::off) ? " (video OFF): " : ": ") << ((VideoParameters::getParameter("axes") == parameter::on) ? "ON" : "OFF") << endl;
}
parameter VideoParameters::getParameter(string param)
{
	if (param == "video")
		return this->video;
	else if (param == "markers")
		return this->markers;
	else if (param == "axes")
		return this->axes;
}
void VideoParameters::setParameter(string param, parameter state)
{
	if (param == "video")
		this->video = state;
	else if (param == "markers")
		this->markers = state;
	else if (param == "axes")
		this->axes = state;
}
int VideoParameters::startCalibration()
{
	cout << "Initiating calibration . . . Please wait . . ." << endl;

	cv::Mat frame;
	cv::Mat draw2Frame;

	vector<cv::Mat> savedImages;
	vector<vector<cv::Point2f>> markerCorners, rejectedCanditate;

	string user_input;
	cout << "Calibrate " << ((VideoParameters::getCurrentWebcam() == webcam::external) ? "external" : "local") << " camera? (Y/n) ";
	std::getline(cin, user_input);
	if ((user_input == "N") || (user_input == "n"))
	{
		VideoParameters::setCurrentWebcam(webcam::local);
		cout << "Using " << ((VideoParameters::getCurrentWebcam() == webcam::external) ? "external" : "local") << " camera." << endl;
	}

	cv::VideoCapture vid(VideoParameters::getCurrentWebcam());

	if (!vid.isOpened()) return -1;

	int fps = 20;
	int pic = 0;

	cv::namedWindow(WEBCAM_WINDOW, CV_WINDOW_AUTOSIZE);

	cout << "Keep focus on video window and press SPACE to take calibration pictures." << endl
		<< "Press ENTER when you took at least 15." << endl
		<< "# taken picture:" << endl;
	while (true)
	{
		if (!vid.read(frame)) break;

		vector<cv::Vec2f> foundPoints;
		bool found = false;

		found = cv::findChessboardCorners(frame, CHESS_BOARD_DIM, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		frame.copyTo(draw2Frame);
		cv::drawChessboardCorners(draw2Frame, CHESS_BOARD_DIM, foundPoints, found);

		if (found)
			cv::imshow(WEBCAM_WINDOW, draw2Frame);
		else
			cv::imshow(WEBCAM_WINDOW, frame);

		char character = cv::waitKey(1000 / fps);
		switch (character)
		{
			case ' ': // SPACE pressed
				if (found)
				{
					++pic;
					cout << pic << endl;
					cv::Mat temp;
					frame.copyTo(temp);
					savedImages.push_back(temp);//const ref
				}
				break;

			case 13: // ENTER pressed Start calibration
				if (savedImages.size()>=15)
				{
					cv::destroyWindow(WEBCAM_WINDOW);
					VideoParameters::cameraCalibration(savedImages);
					VideoParameters::saveCameraCalibration();
					return 1;
					break;
				}
				else cout << "Take more pictures!" << endl;
				break;

			case 27: // ESC pressed, exit
				return 1;
				break;

			default:
				break;
		}
	}
	cout << "Calibration done." << endl;
	return 1;
}
bool VideoParameters::loadCameraCalibration()
{
	cout << "Loading calibration file . . ." << endl;
	//cv::Size chessBoard = cv::Size(6, 9);

	std::ifstream inStream(CALIBDATA_FILE);
	if (inStream)
	{
		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		this->cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);

		for (size_t r = 0; r < rows; r++)
		{
			for (size_t c = 0; c < columns; c++)
			{
				double read = 0.0f;
				inStream >> read;
				this->cameraMatrix.at<double>(r, c) = read;
			}
		}

		//distancecoeff
		inStream >> rows;
		inStream >> columns;
		this->distanceCoeff = cv::Mat::zeros(rows, columns, CV_64F);

		for (size_t r = 0; r < rows; r++)
		{
			for (size_t c = 0; c < columns; c++)
			{
				double read = 0.0f;
				inStream >> read;
				this->distanceCoeff.at<double>(r, c) = read;
			}
		}
		inStream.close();
		cout << "Calibration file loaded." << endl;
		return true;
	}
	else return false;
}
bool VideoParameters::saveCameraCalibration()
{
	cout << "Saving calibration . . ." << endl;
	std::ofstream outStream(CALIBDATA_FILE);

	if (outStream)
	{
		uint16_t rows = this->cameraMatrix.rows;
		uint16_t columns = this->cameraMatrix.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (size_t i = 0; i < rows; i++)
		{
			for (size_t j = 0; j < columns; j++)
			{
				double value = this->cameraMatrix.at<double>(i, j);
				outStream << value << endl;
			}
		}

		rows = this->distanceCoeff.rows;
		columns = this->distanceCoeff.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (size_t r = 0; r < rows; r++)
		{
			for (size_t c = 0; c < columns; c++)
			{
				double value = this->distanceCoeff.at<double>(r, c);
				outStream << value << endl;
			}
		}
		outStream.close();
		return true;
		cout << "Calibration saved." << endl;
	}
	else return false;
}
void VideoParameters::getBoardCorners(vector<cv::Mat> images, vector<vector<cv::Point2f>>& allFoundCorners, bool showResults)
{
	cv::Size chessBoard = cv::Size(6, 9);

	for (vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<cv::Point2f> PointBuf;
		bool found = cv::findChessboardCorners(*iter, CHESS_BOARD_DIM, PointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found)
		{
			allFoundCorners.push_back(PointBuf);
		}
		if (showResults)
		{
			cv::drawChessboardCorners(*iter, CHESS_BOARD_DIM, PointBuf, found);
			cv::imshow("Looking for corners", *iter);
			cv::waitKey(0);
		}
	}
}
void VideoParameters::createArucoMarkers(int num)
{
	cv::Mat outputMarker;

	cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
	for (int i = 0; i < num; i++)
	{
		cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		std::ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		cv::imwrite(convert.str(), outputMarker);
	}
}
void VideoParameters::createKnownBoardPosition(vector<cv::Point3f>& corners)
{
	for (int i = 0; i < CHESS_BOARD_DIM.height; i++)
	{
		for (int j = 0; j < CHESS_BOARD_DIM.width; j++)
		{
			corners.push_back(cv::Point3f(j*CALIBRATION_SQUARE_DIM, i*CALIBRATION_SQUARE_DIM, 0.0f));
		}
	}
}
void VideoParameters::cameraCalibration(vector<cv::Mat> calibrationImages)
{
	cout << "Calibrating . . . This may take a while, please wait . . . " << endl;
	vector<vector<cv::Point2f>> checkerboardImgSpace;
	VideoParameters::getBoardCorners(calibrationImages, checkerboardImgSpace, false);
	vector<vector<cv::Point3f>> worldSpaceCornerpoints(1);

	VideoParameters::createKnownBoardPosition(worldSpaceCornerpoints[0]);
	worldSpaceCornerpoints.resize(checkerboardImgSpace.size(), worldSpaceCornerpoints[0]);

	vector<cv::Mat> rVectors, tVectors;
	this->distanceCoeff = cv::Mat::zeros(8, 1, CV_64F);

	cv::calibrateCamera(worldSpaceCornerpoints, checkerboardImgSpace, CHESS_BOARD_DIM, this->cameraMatrix, this->distanceCoeff, rVectors, tVectors);
	cout << "Calibration done." << endl;
}