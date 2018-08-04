/**
* Robomaster Vision program of Summer camp.
* Copyright (c) 2018, Xidian University Robotics Vision group.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
* documentation files(the "Software"), to deal in the Software without restriction.
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include "cube.h"

using namespace cv;

int main(int, char**)
{
	Mat frame;
	VideoCapture cap;
	// open the default camera using default API
		cap.open(0);
	//cap.open("example2.mp4");
	// OR advance usage: select any API backend
	int deviceID = 0;             // 0 = open default camera
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API
	//open selected camera using selected API
	cap.open(deviceID + apiID);
	if (!cap.isOpened()) {
		std::cerr << "ERROR! Unable to open camera\n";
		return -1;
	}
	Mat gray;

	cubedetector cube;
	while (true)
	{
		cap >> frame;
		if (frame.empty()) {
			std::cerr << "ERROR! blank frame grabbed\n";
			break;
		}

		cube.getcube(frame);

		if (waitKey(1) == 27)
			break;
	}
	return 0;
}