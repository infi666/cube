/**
* Robomaster Vision program of Summer camp.
* Copyright (c) 2018, Xidian University Robotics Vision group.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
* documentation files(the "Software"), to deal in the Software without restriction.
*/
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

class cubedetector
{
public:
	explicit cubedetector() {}
	
	void getcube(const Mat& src);
protected:

	bool checkcube(vector<RotatedRect> & cube_rects);
	
	RotatedRect adjustRRect(const RotatedRect & rect);
private:

	Mat frame;  
	Mat gray;   
};
