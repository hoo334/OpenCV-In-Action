#include <iostream>
#include <fstream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2\imgproc\types_c.h>
using namespace std;
using namespace cv;

//ͼƬ����
int IMGCOUNT;

//��������ڲ�������
Mat cameraMatrixL, cameraMatrixR;

//���̷���ʵ�ʳ���mm��
Size squareSize;

//��������������
Mat distCoeffsL, distCoeffsR;

//���������ȡ�Ķ��ͼ��Ľǵ���������
vector<vector<Point2f>> imagePointsLeft, imagePointsRight;

//����ͼ���С(����ͼ���С����һ��)
Size imageSize;

//�ǵ����������
vector<vector<Point3f>>objectPoints;

//���̸�ǵ�n*n  ����ͼƬʱ�ı�
Size boardSize;

//�����ͼ��·��
String leftImagePath;
//�����ͼ��·��
String rightImagePath;
//�������ʾͼƬ��
String leftImageName;
//�������ʾͼƬ��
String rightImageName;

//R ��תʸ�� Tƽ��ʸ�� E�������� F��������  
Mat R, T, E, F;

//У����ת����R��ͶӰ����P ��ͶӰ����Q 
Mat Rl, Rr, Pl, Pr, Q;

//ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������  
Rect validROIL, validROIR;

//ӳ��� 
Mat mapLx, mapLy, mapRx, mapRy;

//����������ͼ������
Mat rgbImageL, rgbImageR, grayImageL, grayImageR;

//SGBM ����
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

//��ά����
Mat xyz;

//��갴�µ���ʼ��
Point origin;

//�������ѡ��
Rect selection;

//�Ƿ�ѡ�����
bool selectObject = false;

//������������������
vector<Point2f> two2DPoints;

//��ʾ��ȵĵ�
vector<Point3f> two3DPoints;


void cameraMesurement() {

	void cameraCalibrate();
	void cameraRectify();
	void match();
	void saveConfig();
	void loadConfig();
	void loadData();
	void saveData();

	int select = 0;
	loadConfig();
	loadData();

	while (select != 1 && select != 2) {
		cout << "Calibrate Camera ???" << endl;
		cout << "1) yes  2) No " << endl;
		cin >> select;
		if (select == 1) {
			cameraCalibrate(); cameraRectify();
		}
		else if (select == 2) {
			break;
		}
		else {
			cout << "Invalid Input." << endl;
		}
	}

	match();
	saveConfig();
	saveData();
	waitKey(0);
}

/**
* ��������������
*/
int main() {
	cameraMesurement();
	system("pause");
	return 0;
}

/*
SingleCameraCalibrate(...)
������궨������ÿ��ͼ�ǵ��������꣬�ǵ��������꣬ͼ���С
*/
void singleCameraCalibrate(Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>>& imagePoints, Size& imageSize, string filename, Size boardSize, vector<vector<Point3f>>& objectPoints, Size squareSize)
{
	void calObjectPoints(vector<vector<Point3f>> & objectPoints, Size boardSize, Size squareSize);

	//ÿ��ͼ�����ת����
	vector<Mat> rvecsMat;
	vector<Mat> tvecsMat;

	//��ȡ�ǵ�
	for (int i = 1; i <= IMGCOUNT; i++)
	{
		//����ÿ��ͼ���ϼ�⵽�Ľǵ�
		vector<Point2f> imagePointsBuf;
		char picname[20];
		sprintf_s(picname, "%02d.jpg", i);
		String tempname = filename + picname; cout << tempname << endl;
		Mat imageInput = imread(tempname);
		if (!findChessboardCorners(imageInput, boardSize, imagePointsBuf)) {
			//�Ҳ����ǵ�
			assert("can not find chessboard corners!\n");
			return;
		}
		else {
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			//�����ؾ�ȷ��
			//�Դ���ȡ�Ľǵ���о�ȷ��
			find4QuadCornerSubpix(view_gray, imagePointsBuf, Size(5, 5));

			//������ͼƬ�б�ǽǵ�		
			drawChessboardCorners(view_gray, boardSize, imagePointsBuf, true);

			//���������ؽǵ�
			imagePoints.push_back(imagePointsBuf);

			//��ʾͼƬ
			imshow("Camera Calibration", view_gray);
			waitKey(50);
		}
		imageSize.width = imageInput.cols;
		imageSize.height = imageInput.rows;
		imageInput.release();
	}

	calObjectPoints(objectPoints, boardSize, squareSize);

	//cout << object_points[0] << endl<<endl; cout << image_points_seq[0] << endl;

	//����궨
	calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	objectPoints.clear();
	cout << cameraMatrix << endl << endl << distCoeffs << endl << endl;
}

/*
calObjectPoints(...)
����ǵ���������
*/
void calObjectPoints(vector<vector<Point3f>>& objectPoints, Size boardSize, Size squareSize)
{
	for (int t = 0; t < IMGCOUNT; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < boardSize.height; i++)
		{
			for (int j = 0; j < boardSize.width; j++)
			{
				Point3f realPoint;
				//����궨�������������ϵ��z=0��ƽ����
				realPoint.x = (float)(j * squareSize.width);
				realPoint.y = (float)(i * squareSize.height);
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		objectPoints.push_back(tempPointSet);
	}
}

/*�����ͼ��ɫ*/
void GenerateFalseMap(cv::Mat& src, cv::Mat& disp)
{
	// color map  
	float max_val = 255.0f;
	float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
	{ 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
	float sum = 0;
	for (int i = 0; i < 8; i++)
		sum += map[i][3];

	float weights[8]; // relative   weights  
	float cumsum[8];  // cumulative weights  
	cumsum[0] = 0;
	for (int i = 0; i < 7; i++) {
		weights[i] = sum / map[i][3];
		cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
	}

	int height_ = src.rows;
	int width_ = src.cols;
	// for all pixels do  
	for (int v = 0; v < height_; v++) {
		for (int u = 0; u < width_; u++) {

			// get normalized value  
			float val = std::min(std::max(src.data[v * width_ + u] / max_val, 0.0f), 1.0f);

			// find bin  
			int i;
			for (i = 0; i < 7; i++)
				if (val < cumsum[i + 1])
					break;

			// compute red/green/blue values  
			float   w = 1.0 - (val - cumsum[i]) * weights[i];
			uchar r = (uchar)((w * map[i][0] + (1.0 - w) * map[i + 1][0]) * 255.0);
			uchar g = (uchar)((w * map[i][1] + (1.0 - w) * map[i + 1][1]) * 255.0);
			uchar b = (uchar)((w * map[i][2] + (1.0 - w) * map[i + 1][2]) * 255.0);
			//rgb�ڴ��������  
			disp.data[v * width_ * 3 + 3 * u + 0] = b;
			disp.data[v * width_ * 3 + 3 * u + 1] = g;
			disp.data[v * width_ * 3 + 3 * u + 2] = r;
		}
	}
}

/*
����ƥ��
*/
void stereoMatch(int, void*, Ptr<StereoSGBM>& sgbm, Mat& rectifyImageL, Mat& rectifyImageR, Mat& xyz, Mat& Q, Size imgSize)
{
	void clickChessBoardImage(int event, int x, int y, int flags, void* ustc);
	void clickDepthImage(int event, int x, int y, int, void*);
	void insertDepth32f(cv::Mat & depth);

	sgbm->setPreFilterCap(32);
	int SADWindowSize = 9;
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);;//����ʵ������Լ��趨
	int NumDisparities = ((imgSize.width / 8) + 15) & -16;;//�������ȥ��С���졣��ֵʼ�մ����㡣�ڵ�ǰʵ���У��˲�������ɱ�16������x1-x2=B-dis
	int UniquenessRatio = 10;//����ʵ������Լ��趨
	sgbm->setBlockSize(sgbmWinSize);
	int cn = rectifyImageL.channels();

	sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
	sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(NumDisparities);
	sgbm->setUniquenessRatio(UniquenessRatio);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(10);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(StereoSGBM::MODE_SGBM);
	Mat disp, dispf, disp8;
	sgbm->compute(rectifyImageL, rectifyImageR, disp);//�����Ӳ�

	//ȥ�ڱ�
	Mat img1p, img2p;
	copyMakeBorder(rectifyImageL, img1p, 0, 0, NumDisparities, 0, IPL_BORDER_REPLICATE);//��ȡ���������ͼ��
	copyMakeBorder(rectifyImageR, img2p, 0, 0, NumDisparities, 0, IPL_BORDER_REPLICATE);//��ȡ���������ͼ��
	dispf = disp.colRange(NumDisparities, img2p.cols - NumDisparities);

	dispf.convertTo(disp8, CV_8U, 255 / (NumDisparities * 16.));

	//��ʵ�������ʱ��ReprojectTo3D������X / W, Y / W, Z / W��Ҫ����16(Ҳ����W����16)�����ܵõ���ȷ����ά������Ϣ��
	reprojectImageTo3D(dispf, xyz, Q, true);
	xyz = xyz * 16;
	//imshow("disparity", disp8);
	Mat color(dispf.size(), CV_8UC3);
	//GenerateFalseMap(disp8, color);//ת�ɲ�ͼ
	namedWindow("disparity", CV_WINDOW_NORMAL);
	//setMouseCallback(��������, ���ص�����, �����ص������Ĳ�����һ��ȡ0)
	setMouseCallback("disparity", &clickDepthImage, 0);//disparity
	imshow("disparity", disp8);
	//saveXYZ("xyz.xls", xyz);
}

void cameraCalibrate()
{

	//��������������궨
	singleCameraCalibrate(cameraMatrixL, distCoeffsL, imagePointsLeft, imageSize, leftImagePath, boardSize, objectPoints, squareSize);
	singleCameraCalibrate(cameraMatrixR, distCoeffsR, imagePointsRight, imageSize, rightImagePath, boardSize, objectPoints, squareSize);

	//����ǵ����������
	calObjectPoints(objectPoints, boardSize, squareSize);

	//˫Ŀ�궨
	double rms = stereoCalibrate(objectPoints, imagePointsLeft, imagePointsRight,
		cameraMatrixL, distCoeffsL,
		cameraMatrixR, distCoeffsR,
		imageSize, R, T, E, F, CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "rms:" << rms << endl << endl;
	cout << "R:" << R << "T:" << T << endl << endl;
}

void cameraRectify()
{
	//���������
	stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, R, T, Rl, Rr, Pl, Pr, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validROIL, &validROIR);
	cout << "Q:" << Q << endl << endl;

	//�����У��ӳ��
	initUndistortRectifyMap(cameraMatrixL, distCoeffsL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(cameraMatrixR, distCoeffsR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

	cout << "Pr:" << Pr << endl << endl;
}

void match()
{
	void drawLines(Mat & img);
	void clickChessBoardImage(int event, int x, int y, int flags, void* ustc);
	void clickDepthImage(int event, int x, int y, int, void*);
	//��ȡ�����ͼƬ
	string filename = leftImagePath + leftImageName;
	rgbImageL = imread(filename); //imshow("beforel", rgbImageL); 
	cvtColor(rgbImageL, grayImageL, CV_BGR2GRAY);

	//��ȡ�����ͼƬ
	filename = rightImagePath + rightImageName;
	rgbImageR = imread(filename);
	cvtColor(rgbImageR, grayImageR, CV_BGR2GRAY);

	//����ӳ����������ͼ��
	Mat rectifyImageL2, rectifyImageR2;
	remap(rgbImageL, rectifyImageL2, mapLx, mapLy, INTER_LINEAR);
	remap(rgbImageR, rectifyImageR2, mapRx, mapRy, INTER_LINEAR);

	//namedWindow("originImageL", CV_WINDOW_NORMAL);
	//imshow("originImageL", rgbImageL);

	//namedWindow("originImageR", CV_WINDOW_NORMAL);
	//imshow("originImageR", rgbImageR);
	//

	namedWindow("rectifyImageL", CV_WINDOW_NORMAL);
	setMouseCallback("rectifyImageL", &clickChessBoardImage, 0);
	drawLines(rectifyImageL2);
	imshow("rectifyImageL", rectifyImageL2);

	namedWindow("rectifyImageR", CV_WINDOW_NORMAL);
	setMouseCallback("rectifyImageR", &clickChessBoardImage, 0);
	drawLines(rectifyImageR2);
	imshow("rectifyImageR", rectifyImageR2);

	stereoMatch(0, 0, sgbm, rectifyImageL2, rectifyImageR2, xyz, Q, imageSize);
}

void drawLines(Mat& img) {
	for (int i = 0; i < img.rows; i += 32) {
		line(img, Point(0, i), Point(img.cols, i), Scalar(0, 255, 0));
	}
}

//���������ʵ�ʾ���
void calDistance()
{
	if (two2DPoints.size() == 2)
	{
		double x, y, z; double A, B;
		double ul, vl, ur, vr;
		double axl, ayl, uol, vol, axr, ayr, uor, vor;
		double r1, r2, r3, r4, r5, r6, r7, r8, r9, t1, t2, t3;

		ul = two2DPoints[0].x; vl = two2DPoints[0].y;
		ur = two2DPoints[1].x; vr = two2DPoints[1].y;

		axl = cameraMatrixL.at<double>(0, 0);
		uol = cameraMatrixL.at<double>(0, 2);
		ayl = cameraMatrixL.at<double>(1, 1);
		vol = cameraMatrixL.at<double>(1, 2);

		axr = cameraMatrixL.at<double>(0, 0);
		uor = cameraMatrixL.at<double>(0, 2);
		ayr = cameraMatrixL.at<double>(1, 1);
		vor = cameraMatrixL.at<double>(1, 2);

		r1 = R.at<double>(0, 0); r2 = R.at<double>(0, 1); r3 = R.at<double>(0, 2);
		r4 = R.at<double>(1, 0); r5 = R.at<double>(1, 1); r6 = R.at<double>(1, 2);
		r7 = R.at<double>(2, 0); r8 = R.at<double>(2, 1); r9 = R.at<double>(2, 2);
		t1 = T.at<double>(0); t2 = T.at<double>(1); t3 = T.at<double>(2);

		A = (ur - uor) * (r7 * ayl * (ul - uol) + r8 * axl * (vl - vol) + r9 * axl * ayl) - (r1 * ayl * axr * (ul - uol) + r2 * axl * axr * (vl - vol) + r3 * axl * ayl * axr);
		B = (vr - vor) * (r7 * ayl * (ul - uol) + r8 * axl * (vl - vol) + r9 * axl * ayl) - (r4 * ayl * ayr * (ul - uol) + r5 * axl * ayr * (vl - vol) + r6 * axl * ayl * ayr);

		z = (axl * ayl * axr * t1 - axl * ayl * t3 * (ur - uor)) / A;
		x = z * (ul - uol) / axl;
		y = z * (vl - vol) / ayl;

		cout << "x: " << x << "y: " << y << "z: " << z << endl;
		Point3f temp; temp.x = x; temp.y = y; temp.z = z;
		two3DPoints.push_back(temp);
		if (two3DPoints.size() == 2)
		{
			double dis = sqrt((two3DPoints[0].x - two3DPoints[1].x) * (two3DPoints[0].x - two3DPoints[1].x) + (two3DPoints[0].y - two3DPoints[1].y) * (two3DPoints[0].y - two3DPoints[1].y) + (two3DPoints[0].z - two3DPoints[1].z) * (two3DPoints[0].z - two3DPoints[1].z));
			cout << "dis:   " << dis << endl;
			two3DPoints.clear();
		}

	}
	else
		return;
}

void clickChessBoardImage(int event, int x, int y, int flags, void* ustc)
{
	void calDistance();
	if ((event == EVENT_LBUTTONDOWN))//�����������¼�����  
	{
		Point2f pt = cvPoint(x, y);//��ȡ��ǰ��ĺ�������ֵ 
		two2DPoints.push_back(pt); cout << pt << endl;
		calDistance();
		if (two2DPoints.size() == 2)two2DPoints.clear();

	}

}

void clickDepthImage(int event, int x, int y, int, void*)
{

	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:   //�����ť���µ��¼�
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
		cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;

		break;
	case EVENT_LBUTTONUP:    //�����ť�ͷŵ��¼�
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			break;
	}

}



void saveConfig() {
	FileStorage fs("config.txt", FileStorage::WRITE);

	fs << "IMGCOUNT" << IMGCOUNT;
	fs << "squareSizeW" << squareSize.width << "squareSizeH" << squareSize.height;
	fs << "boardSizeW" << boardSize.width << "boardSizeH" << boardSize.height;
	fs << "leftImagePath" << leftImagePath;
	fs << "leftImageName" << leftImageName;
	fs << "rightImagePath" << rightImagePath;
	fs << "rightImageName" << rightImageName;
	fs.release();
}

void loadConfig() {
	FileStorage fs("config.txt", FileStorage::READ);
	IMGCOUNT = fs["IMGCOUNT"].real();
	squareSize = Size(fs["squareSizeW"].real(), fs["squareSizeH"].real());
	boardSize = Size(fs["boardSizeW"].real(), fs["boardSizeH"].real());
	leftImagePath = fs["leftImagePath"].string();
	leftImageName = fs["leftImageName"].string();
	rightImagePath = fs["rightImagePath"].string();
	rightImageName = fs["rightImageName"].string();

	cout << "IMGCOUNT: " << IMGCOUNT << endl;
	cout << "squareSize: " << squareSize << endl;
	cout << "boardSize: " << boardSize << endl;
	cout << "leftImagePath: " << leftImagePath << ", leftImageName: " << leftImageName << endl;
	cout << "rightImagePath: " << rightImagePath << ", rightImageName: " << rightImageName << endl;

	fs.release();

}

void saveData() {
	FileStorage fs("data.txt", FileStorage::WRITE);

	fs << "cameraMatrixL" << cameraMatrixL << "cameraMatrixR" << cameraMatrixR;
	fs << "imageSizeW" << imageSize.width << "imageSizeH" << imageSize.height;
	fs << "R" << R << "T" << T << "Q" << Q;
	fs << "mapLx" << mapLx << "mapLy" << mapLy << "mapRx" << mapRx << "mapRy" << mapRy;

	fs.release();
}

void loadData() {
	FileStorage fs("data.txt", FileStorage::READ);
	cameraMatrixL = fs["cameraMatrixL"].mat();
	cameraMatrixR = fs["cameraMatrixR"].mat();
	imageSize = Size(fs["imageSizeW"].real(), fs["imageSizeH"].real());
	R = fs["R"].mat(); T = fs["T"].mat(); Q = fs["Q"].mat();
	mapLx = fs["mapLx"].mat(); mapLy = fs["mapLy"].mat(); mapRx = fs["mapRx"].mat(); mapRy = fs["mapRy"].mat();

	cout << "cameraMatrixL: " << endl << cameraMatrixL << endl << "cameraMatrixR" << endl << cameraMatrixR << endl;
	fs.release();
}


