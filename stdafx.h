/*
	stdafx.h : include file for standard system include files,
	or project specific include files that are used frequently, but
	are changed infrequently
*/

#pragma once
// Required headers throughout the project

/*
GENERAL HEADERS
*/
#include "targetver.h"
#include <stdlib.h> // for std::atoi(), std::stoi(), std::rand()
#include <windows.h> // for DWORD, HANDLE, COMSTAT, DCB, Windows OS specific header
#include <time.h> // for clock(), clock_t
#include <locale> // for std::isalpha()

/*
DATA STREAM RELATED HEADERS
*/
#include <iostream>  // for std::fstream, std::cout, cin, endl...
#include <sstream> // for std::stringstream
#include <fstream> // for std::ofstream, std::ifstream

// OpenCV specific headers
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\aruco.hpp>
#include <opencv2\ccalib.hpp>
//#include <opencv2\calib3d.hpp> // used?
//#include <opencv2\imgcodecs.hpp> // used?

// Including OpenCV to any project in Visual Studio:
/*
Solution explorer -> Properites -> C/C++ -> Additional Include Directories -> write "$(OPENCV3_DIR)\include"
Solution explorer -> Properites -> Linker -> Additional Library Directories -> write "$(OPENCV3_DIR)\x86\vc15\lib"
Solution explorer -> Properties -> Linker -> Input -> Additional Dependencies -> add
"opencv_aruco330d.lib
opencv_core330d.lib
opencv_calib3d330d.lib
opencv_ccalib330d.lib
opencv_highgui330d.lib
opencv_imgcodecs330d.lib
opencv_imgproc330d.lib
opencv_videoio330d.lib
opencv_video330d.lib" For Debug
For Release, remove the 'd' suffixe
Add any other opencv_xxx(d).lib you might need
*/

/*
DATA TYPE HEADERS
*/
#include <string> // for std::string
#include <vector> // for std::vector
#include <conio.h> // for getline() and _getch()

/*
THREAD RELATED HEADERS
*/
#include <thread> // to handle std::threads
#include <mutex> // to handle std::mutex

/*
FREQUENTLY USED FUNCTION CALLS FROM STD NAMESPACE
additional calls from std namespace should be written using std:: to avoid ambiguous calls
DO NOT "#using namespace std" (e.g.) or any other namespace such as "#using namespace cv"
this may cause problems with ambiguous calls
call functions using namespacename::classname::func or namespacename::func such as std::getline()
*/
using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;