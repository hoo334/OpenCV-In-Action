#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2\imgproc\types_c.h>
using namespace std;
using namespace cv;


int main()
{
	Mat vignetting_correction(string path);

	//生成文件路径
	string directory = ".\\samples\\";
	char picname[20];
	sprintf_s(picname, "%d.jpg", 3);
	string path = directory + picname;

	// 判断文件是否存在
	fstream file;
	file.open(path, ios::in);

	if (!file) {
		cout << path << "不存在!" << endl;
		return -1;
	}

	//显示原始图像和矫正图像
	Mat origin = imread(path);
	namedWindow("origin", CV_WINDOW_NORMAL);
	imshow("origin", origin);

	Mat correction = vignetting_correction(path);
	namedWindow("vignetting correction", CV_WINDOW_NORMAL);
	imshow("vignetting correction", correction);


	waitKey(0);
	system("pause");
	return 0;
}


/// <summary>
/// 去暗角算法 Revisiting Image Vignetting Correction by Constrained Minimization of Log-Intensity Entropy 
/// </summary>
/// <param name="path">图片路径</param>
/// <returns>去除暗角后的彩图</returns>
Mat vignetting_correction(string path) {
	Mat process(Mat img);
	Mat img = imread(path);
	int channelsNum = img.channels();

	Mat* channel = new Mat[channelsNum];
	Mat dst;
	const int rows = img.rows;
	const int cols = img.cols;

	split(img, channel);


	for (int c = 0; c < channelsNum; c++) {
		channel[c] = process(channel[c]);
	}

	merge(channel, size_t(3), dst);

	return dst;
}

/// <summary>
/// 对灰度图像进行矫正（或者图像的一个分量）
/// </summary>
/// <param name="img">灰度图像</param>
/// <returns>矫正图像</returns>
Mat process(Mat img) {
	bool check_abc(float a, float b, float c);
	float calculateH(float a, float b, float c, Mat img);
	float cal_g(float a, float b, float c, Point2f opticalPoint, Point2f pixelPoint);

	Mat dst(img.size(), CV_8UC1);
	const int rows = img.rows;
	const int cols = img.cols;

	float values[3], minValues[3];
	float delta = 8;
	memset(values, 0, sizeof(float) * 3);
	memset(minValues, 0, sizeof(float) * 3);
	float minH = calculateH(values[0], values[1], values[2], img);

	//计算 a，b，c
	while (delta > 1 / 256.0) {
		for (int i = 0; i < 6; i++) {
			if ((i & 1) == 0) {
				values[i / 2] += delta;
			}
			if ((i & 1) == 1) {
				values[i / 2] -= 2 * delta;
			}
			//std::cout << values[0] << "|" << values[1] << "|" << values[2] << endl;
			if (check_abc(values[0], values[1], values[2])) {
				float H = calculateH(values[0], values[1], values[2], img);
				if (H < minH) {
					minH = H;
					memcpy(minValues, values, sizeof(float) * 3);
				}
			}
			if ((i & 1) == 1) {
				values[i / 2] += delta;
			}
		}
		delta = delta / 2.0;
		//std::cout << "delta: " << delta << endl;
	}
	//std::cout << minValues[0] << "|" << minValues[1] << "|" << minValues[2] << "|";

	//去暗角
	for (int row = 0; row < rows; row++) {
		uchar* originLine = img.ptr<uchar>(row);
		uchar* dstLine = dst.ptr<uchar>(row);
		for (int col = 0; col < cols; col++) {
			float g = cal_g(minValues[0], minValues[1], minValues[2], Point2f(rows / 2, cols / 2), Point2f(row, col));
			int val = round(originLine[col] * g);
			if (val > 255) {
				dstLine[col] = 255;
			}
			else if (val < 0) {
				dstLine[col] = 0;
			}
			else {
				dstLine[col] = val;
			}
		}
	}
	return dst;
}

/// <summary>
/// 计算 g 值
/// </summary>
/// <param name="opticalPoint">摄像机光学中心</param>
/// <param name="pixelPoint">当前像素点</param>
/// <returns>g 值</returns>
float cal_g(float a, float b, float c, Point2f opticalPoint, Point2f pixelPoint) {
	float r = sqrt(pow(pixelPoint.x - opticalPoint.x, 2) + pow(pixelPoint.y - opticalPoint.y, 2)) / sqrt(pow(opticalPoint.x, 2) + pow(opticalPoint.y, 2));
	float g = 1 + a * pow(r, 2) + b * pow(r, 4) + c * pow(r, 6);
	return g;
}

/// <summary>
/// 判断参数 a，b，c 是否满足要求
/// </summary>
bool check_abc(float a, float b, float c) {
	float qPositive = (sqrt(4 * b * b - 12 * a * c) - 2 * b) / (6 * c);
	float qNegative = (-sqrt(4 * b * b - 12 * a * c) - 2 * b) / (6 * c);

	if (a > 0 && b == 0 && c == 0 ||
		a >= 0 && b > 0 && c == 0 ||
		c == 0 && b < 0 && -a <= 2 * b ||
		c > 0 && b * b < 3 * a * c ||
		c > 0 && b * b == 3 * a * c && b >= 0 ||
		c > 0 && b * b == 3 * a * c && -b >= 3 * c ||
		c > 0 && b * b > 3 * a * c && qPositive <= 0 ||
		c > 0 && b * b > 3 * a * c && qNegative >= 1 ||
		c < 0 && b * b > 3 * a * c && qPositive >= 1 && qNegative <= 0
		) {
		return true;
	}
	else {
		return false;
	}
}

/// <summary>
/// 计算灰度图像的熵
/// </summary>
/// <param name="img">灰度图像或图像的一个通道</param>
/// <returns>灰度图的熵</returns>
float calculateH(float a, float b, float c, Mat img) {
	Mat floatImg(img.size(), CV_32FC1);
	const int rows = floatImg.rows;
	const int cols = floatImg.cols;

	/*
	* 使用 a，b，c 校正原始图像
	*/
	for (int i = 0; i < rows; i++) {
		uchar* origin = img.ptr<uchar>(i);
		float* correct = floatImg.ptr<float>(i);
		for (int j = 0; j < cols; j++) {
			float g = cal_g(a, b, c, Point2f(rows / 2, cols / 2), Point2f(i, j));
			correct[j] = origin[j] * g;
		}
	}

	/*
	* 计算对数熵
	*/
	for (int i = 0; i < rows; i++) {
		float* line = floatImg.ptr<float>(i);
		for (int j = 0; j < cols; j++) {
			line[j] = 255 * log(1 + line[j]) / 8;
		}
	}

	/*
	* 计算直方图每个 bin 中的像素数目
	*/
	float histogram[256];
	memset(histogram, 0, sizeof(float) * 256);
	for (int row = 0; row < rows; ++row)
	{
		float* value = floatImg.ptr<float>(row);
		for (int col = 0; col < cols; ++col)
		{
			int k_floor = floor(value[col]);
			int k_ceil = ceil(value[col]);
			histogram[k_floor] += (1 + k_floor - value[col]);
			histogram[k_ceil] += (k_ceil - value[col]);
		}
	}

	/*
	* 对直方图进行线性平滑  SmoothRadius = 4
	*/
	float tempHist[256 + 2 * 4];
	for (int i = 0; i < 4; i++) {
		tempHist[i] = histogram[4 - i];
		tempHist[260 + i] = histogram[254 - i];
	}

	memcpy(tempHist + 4, histogram, 256 * sizeof(float));

	for (int X = 0; X < 256; X++) {
		histogram[X] = (tempHist[X] + 2 * tempHist[X + 1] + 3 * tempHist[X + 2] + 4 * tempHist[X + 3] +
			5 * tempHist[X + 4] + 4 * tempHist[X + 5] + 3 * tempHist[X + 6] + 2 * tempHist[X + 7]) + tempHist[X + 8] / 25.0f;
	}

	/*
	* 计算熵
	*/
	float sumNk = 0;
	for (int i = 0; i < 256; ++i)
	{
		sumNk += histogram[i];
	}
	float H = 0, pk;
	for (int i = 0; i < 256; ++i)
	{
		pk = histogram[i] / sumNk;
		if (pk != 0)
			H += pk * log(pk);
	}
	return -H;
}
