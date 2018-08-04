/**
* Robomaster Vision program of Summer camp.
* Copyright (c) 2018, Xidian University Robotics Vision group.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
* documentation files(the "Software"), to deal in the Software without restriction.
*/

#include"cube.h"

RotatedRect cubedetector::adjustRRect(const RotatedRect & rect) 
{
	const Size2f & s = rect.size;
	if (s.width > s.height)
		return rect;
	return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
}

void cubedetector::getcube(const Mat& src)
{
	int center_x = 0, center_y = 0;
	static int count = 0;
	double step_width;
	src.copyTo(frame);

	cvtColor(frame, gray, CV_RGB2GRAY);
	GaussianBlur(gray, gray, Size(5, 5), 0, 0);

	//threshold(gray, gray, 90, 255, THRESH_OTSU);
	adaptiveThreshold(gray, gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 1);

	medianBlur(gray, gray, 3);
	vector<vector<Point2i> > contours;
	vector<Vec4i> hierarchy;
	findContours(gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<RotatedRect> cube_rects;
	for (size_t i = 0; i < contours.size(); ++i)
	{
		RotatedRect rect = minAreaRect(contours[i]);
		rect = adjustRRect(rect);
		const Size2f & s = rect.size;
		float ratio_cur = s.width / s.height;

		if (0.8 < ratio_cur && ratio_cur < 1.2 &&
			20 < rect.size.width && rect.size.width < frame.cols / 5)
		{
			cube_rects.push_back(rect);
		}
	}

	if (checkcube(cube_rects))
	{
		for (int i = 0; i < cube_rects.size(); ++i)
		{
			Point2f vertices[4];
			cube_rects[i].points(vertices);
			for (int i = 0; i < 4; i++)
				line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(50, 255, 50), 2);//先画矩形线
		}
		for (int i = 0; i < cube_rects.size(); ++i) //九个矩形方块的x坐标的平均值
		{
			center_x += cube_rects[i].center.x;
		}
		center_x /= 9;
		for (int i = 0; i < cube_rects.size(); ++i) // 九个矩形方块的y坐标的平均值
		{
			center_y += cube_rects[i].center.y;
		}
		center_y /= 9;

		step_width = cube_rects[0].size.width * 1.5;//画矩阵外框
		rectangle(frame, Point(center_x - step_width, center_y - step_width), Point(center_x + step_width, center_y + step_width), Scalar(0, 0, 255), 4);

		Mat hsvImage = frame.clone();

		vector<int> colors;

		cvtColor(hsvImage, hsvImage, COLOR_BGR2HSV);//转为hsv的颜色空间

		for (size_t i = 0; i < cube_rects.size(); ++i) 
		{	
			colors.push_back(hsvImage.at<Vec3b>(cube_rects[i].center)[0]);//Vec3b
		}

		std::sort(colors.begin(), colors.end());
		vector<int>::iterator it = colors.begin();//迭代器

		it = colors.erase(it);
		while (it != colors.end())
			++it;
		colors.erase(it - 1);//删除

		int ave = 0, result = 0;

		for (size_t i = 0; i < colors.size(); ++i) //均值
		{
			ave = ave + colors[i];
		}
		ave = ave / colors.size();

		for (size_t i = 0; i < colors.size(); ++i) //方差
		{
			result = result + (colors[i] - ave) * (colors[i] - ave);
		}
		result = result / colors.size();
		printf("%d\t\t\t", result);

		for (size_t i = 0; i < colors.size(); ++i)
		{
			printf("%d\t", colors[i]);
		}
		printf("\n");

		if (result < 50) //当方差<50时计数一次
		{
			++count;
		}

		if (count > 15)//15次后结束一轮识别.
		{
			printf("You can fetch the cube\n");
			count = 0;
		}
	}
	imshow("Live", frame);

	imshow("gray", gray);
}

bool cubedetector::checkcube(vector<RotatedRect> & cube_rects)
{
	const int cube_size = cube_rects.size();
	if (cube_rects.size() > 15 || cube_rects.size() <= 9) return false;
	if (cube_rects.size() > 9)
	{
		float dist_map[15][15] = { 0 };
		// calculate distance of each cell center
		for (int i = 0; i < cube_size; ++i)
		{
			for (int j = i + 1; j < cube_size; ++j)
			{
				float d = (cube_rects[i].center.x - cube_rects[j].center.x)*(cube_rects[i].center.x - cube_rects[j].center.x) +
					(cube_rects[i].center.y - cube_rects[j].center.y)*(cube_rects[i].center.y - cube_rects[j].center.y); 
				dist_map[i][j] = d;
				dist_map[j][i] = d;
			}
		}

		// choose the minimun distance cell as center cell
		int center_idx = 0;
		float min_dist = 100000000;
		for (int i = 0; i < cube_size; ++i)
		{
			float cur_d = 0;
			for (int j = 0; j < cube_size; ++j) cur_d += dist_map[i][j];

			if (cur_d < min_dist) {
				min_dist = cur_d;
				center_idx = i;
			}
		}

		// sort distance between each cell and the center cell
		vector<pair<float, int>> dist_center;
		for (int i = 0; i < cube_size; ++i)
			dist_center.push_back(make_pair(dist_map[center_idx][i], i));

		std::sort(dist_center.begin(), dist_center.end(), [](const pair<float, int> & p1, const pair<float, int> & p2) { return p1.first < p2.first; });

		// choose the nearest 9 cell as suduku
		vector<RotatedRect> sudoku_rects_temp;
		for (int i = 0; i < 9; ++i)
			sudoku_rects_temp.push_back(cube_rects[dist_center[i].second]);

		sudoku_rects_temp.swap(cube_rects);
	}
	return cube_rects.size() == 9;
}