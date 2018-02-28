/*
	This program is written by
	R.Bjorkli, S.S.Haraldsen, T.S.Langfeldt & C.D.L.Plaissy
	08-11.2017 for Hogskolen i Oslo og Akershus
	as part of the DRACO project in Robotteknikk & Kybernetikk II
	--------------------------------------------------------------
	main.cpp : Defines the entry point for the console application.
*/

// TODO: Version program!!!
// TODO: ADAPT CODE TO C++ STANDARDS AND CONVENTIONAL DESIGN PATTERNS

#include "stdafx.h"
#include "Process.h"

// main function
int main()
{
	// Set console title to DRACO
	SetConsoleTitle(TEXT("DRACO"));
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl;
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl;
	cout << "\t\t\t\tDRACO\tDrone Regulation with AruCO" << endl;
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl;
	cout << "-----------------------------------------------------------------------------------------------------------------" << endl << endl;
	// Initialize process
	Process* process_control;
	// Default values are passsed to the constructor of the class, and threads are initiated
	process_control = new Process(mode::manual, regulator::regoff, filter::filteroff,
								  cv::Vec3d(0,0,0.8), parameter::on, parameter::off, parameter::off);
	// the system runs until the threads are terminated
	// Free dynamic memory when process is over
	delete process_control;

	return 0;
}