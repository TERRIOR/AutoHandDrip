#include "Vibe.h"
#include <cstdio>

using namespace cv;
using namespace std;
#define MAX_GRAY_VALUE 256
#define MIN_GRAY_VALUE 0
const int g_cannyLowThreshold =3;
const int c_rectr = 20;
int getarea(Mat img);
int otsu(cv::Mat&dst);
int otsu(cv::Mat&dst, cv::Mat&mask);
bool isclose(vector<Point> contour);
int main(int argc, char* argv[])
{
	Mat frame, gray, mask ,f1;
	VideoCapture capture;
	RotatedRect ellisperectmax,rrect;
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	capture.open("3.avi");
	if (!capture.isOpened())
	{
		cout << "No camera or video input!\n" << endl;
		return -1;
	}

	ViBe_BGS Vibe_Bgs;
	bool count = true;
	capture >> f1;
	capture >> frame;
	////////////////////////////////取出最外层的滤杯圈//////////////////////////////
	Mat frame_gray,framecp;
	Mat cmask;
	Rect re;
	int c_size;
	frame.copyTo(framecp);
	cvtColor(frame, frame_gray, CV_RGB2GRAY);
	GaussianBlur(frame_gray, frame_gray, Size(35, 35), 0, 0);//高斯必须为奇数
	threshold(frame_gray, frame_gray, 0, 255, THRESH_BINARY | THRESH_OTSU);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(12, 12));
	//进行形态学操作  
	morphologyEx(frame_gray, frame_gray, MORPH_OPEN, element);
	Canny(frame_gray, frame_gray, g_cannyLowThreshold, g_cannyLowThreshold * 3, 3);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(frame_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//////////////////////////////////////////////////////
	unsigned int cmin = 200;
	unsigned int cmax = 2000;
	vector<vector<Point> >::iterator itc = contours.begin();
	while (itc != contours.end())
	{
		if (itc->size()<cmin || itc->size()>cmax  )
			itc = contours.erase(itc);
		else
		{
			++itc;
			
		}
		
	}
	//cout << "Contours: " << contours.size() << "  " << ends;
	//cout << endl;
	/////////////////////////////////////
	if (contours.size() > 0){
		itc = contours.begin();
		int i = 0;
		int j = 0;
		unsigned int max = itc->size();

		while (itc != contours.end())
		{
			Scalar color(40 * i, 255, 30 * i);
			drawContours(framecp, contours, i, color, 2);
			if (itc->size() > max){
				max = itc->size();
				j = i;
			}
			itc++;
			i++;
		}
		//cout << "max:" << max << "第" << j << endl;
		c_size = max;
		ellisperectmax = fitEllipse(Mat(contours[j]));
		cout << "height:" << ellisperectmax.size.height << "width:" << ellisperectmax.size.width << "area:" << ellisperectmax.size.area() << endl;
		re = boundingRect(contours[j]);//roi感兴趣区域，其他去除
		ellipse(framecp, ellisperectmax, Scalar(rand() & 255, rand() & 255, rand() & 255), 8);
		imshow("framecp", framecp);
		cmask = Mat::zeros(frame.size(), CV_8UC1);
		cmask.setTo(0);
		ellipse(cmask, ellisperectmax, Scalar(255), -1);
		cmask = cmask(Rect(re.x + 15, re.y + 15, re.size().width - 30, re.size().height - 30));
		ellisperectmax.center.x = ellisperectmax.center.x - re.x-15;//消除切除后圆心的偏移
		ellisperectmax.center.y = ellisperectmax.center.y - re.y-15;//同上
		//Mat cmask2 = Mat::zeros(frame.size(), CV_8UC1);
		//circle(cmask2, c_point, c_rectr, Scalar(255));
		//imshow("ellipse", cmask);
		//imshow("ellipse", cmask2);
	}
	while (1)
	{
		capture >> frame;
		if (frame.empty())
			continue;
		frame = frame( Rect(re.x+15,re.y+15,re.size().width-30,re.size().height-30));//只取出中心部分，其余不要，可减少遍历时运算，加快速度
		cvtColor(frame, gray, CV_RGB2GRAY);
		if (count)
		{
			
			cout << "rows:"<<frame.rows<<"col:"<<frame.cols<< endl;
			Vibe_Bgs.init(gray);
			Vibe_Bgs.processFirstFrame(gray);
			cout << " Training ViBe complete!" << endl;
			count = false;
		}
		else
		{
			Vibe_Bgs.testAndUpdate(gray);
			mask = Vibe_Bgs.getMask();
			
			Mat element = getStructuringElement(MORPH_ELLIPSE, Size(7,7));
			morphologyEx(mask, mask, MORPH_OPEN, element);
			
			int mask_area = 0;
			mask_area=getarea(mask);//这里的mask需要改成只有注水杆区域（目前还包括粉层的扩散）
			cout << "area:"<<mask_area<< endl;

			if (mask_area<1000)
			{
				frame.copyTo(f1);
				cout << "f1update"<< endl;
			}
			//imshow("f1", f1);
			//imshow("mask", mask);
			Mat frame2(Scalar(255));
			frame2.setTo(255);
			Mat frame_gs;
			Mat frame3;
			frame.copyTo(frame3, mask);
			//imshow("frame3", frame3);
	
			f1.copyTo(frame2, mask);
			mask = 255 - mask;//mask取反 此时mask为没运动区域
			frame.copyTo(frame2, mask);
			Mat frame2_gray;
			cvtColor(frame2, frame2_gray, CV_RGB2GRAY);
			//GaussianBlur(frame2_gray, frame2_gray, Size(3, 3), 0, 0);
			element = getStructuringElement(MORPH_ELLIPSE, Size(12, 12));
			//进行形态学操作  
			//morphologyEx(frame2_gray, frame2_gray, MORPH_OPEN, element);
			Mat dst1;
			//mask = cmask&mask;//此时mask为椭圆内没运动区域
			GaussianBlur(gray, gray, Size(13, 13), 0, 0);
			morphologyEx(gray, gray, MORPH_CLOSE, element);
			int a = otsu(gray, cmask);//只对cmask二值化（避免四个角剩余的蓝黑色部分影响）
			cout << "a:" << a<<endl;
			//Mat gray_roi;//设置roi
			//gray_roi = gray(re);//把那个区域拿出来；
			//imshow("roi", gray_roi);
			gray = gray > a;
			Mat c_gray;
			gray.copyTo(c_gray, cmask);
			
			//erode(c_gray, c_gray, element);//腐蚀操作
			//morphologyEx(c_gray, c_gray, MORPH_CLOSE, element);
			imshow("dst", c_gray);
			Canny(c_gray, c_gray, g_cannyLowThreshold, g_cannyLowThreshold * 3, 3);
			findContours(c_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			vector<vector<Point> >::iterator itContours = contours.begin();
			for (; itContours != contours.end(); ++itContours) {

				cout << "Size: " << itContours->size() << ends;//每个轮廓包含的点数
				/*vector<Point>::iterator it1 = itContours->begin();
				for (; it1 != (*itContours).end(); ++it1) {
					cout << "x:" << it1->x<<"y:"<<it1->y << endl;
				}*/

			}
			cout << "Contours: " << contours.size() << endl;
			unsigned int cmin = 200;
			unsigned int cmax =500;
			vector<vector<Point> >::iterator itc = contours.begin();
			while (itc != contours.end())
			{
				if ((!isclose(*itc))||itc->size()<cmin || itc->size()>cmax )
					itc = contours.erase(itc);
				else
				{
					++itc;
				}
			}
			cout << "Contours: " << contours.size() << "  " << ends;
			cout << endl;
			////////////////////////////////////////////
			if (contours.size() > 0){
				itc = contours.begin();
				int i = 0;
				int j = 0;
				unsigned int max = itc->size();
				frame.copyTo(framecp);
				while (itc != contours.end())
				{
					Scalar color(40 * i, 255, 30 * i);
					drawContours(framecp, contours, i, color, 2);
					if (itc->size() > max){
						max = itc->size();
						j = i;
					}
					itc++;
					i++;
				}
			
				RotatedRect rrect = fitEllipse(Mat(contours[j]));
				cout << "height:" << rrect.size.height << "width:" << rrect.size.width << "area:"<<rrect.size.area()<<endl;
				if (abs(rrect.center.x - ellisperectmax.center.x)<c_rectr&&abs(rrect.center.y - ellisperectmax.center.y)<c_rectr 
					&& (rrect.size.height + rrect.size.width)<(ellisperectmax.size.height + ellisperectmax.size.width)
					&& rrect.size.area()<ellisperectmax.size.area())
				{
					
					ellipse(framecp, rrect, Scalar(rand() & 255, rand() & 255, rand() & 255), 8);
				}
				imshow("framecp", framecp);//绘制椭圆轮廓后的图像
			}
			imshow("f2", frame2_gray);//消除遮挡后mat
		}
		//imshow("input", frame);

		if (cvWaitKey(10) == 27)
			break;
	}

	return 0;
}

bool isclose(vector<Point> contour){//判断轮廓是否连续
	Point firstp;
	Point endp;
	firstp = contour[contour.size() / 4*3];
	endp = contour[contour.size()/4];
	//所有findcontour出来的轮廓都是闭合的，如果轮廓为曲线/直线（看上去），则是原路返回
	//所以取1/4 与3/4的两个点进行对比，如果两个点位置为同一个点则，可以判断该轮廓不是闭合曲线（视觉）
	if (abs(firstp.x - endp.x) <=50 && abs(firstp.y - endp.y) <= 50)
	{
		return false;
	}
	else
	{
		return true;
	}
}
int getarea(Mat img){
	int i = 0,j=0;
	int height = img.cols;
	int width = img.rows;
	int count=0;
	for (i = 0; i<width; i++){
		for (j = 0; j<height; j++){
			
			if (img.at<char>(i,j)!=0)
			{
				count++;
			}
		}
	}
	return count;
}
////////////////////////////////灰度直方图增强对比度///////////////////////////////////////////////////
int ImgStrong(Mat img, Mat result)
{
	//***************  
	//p[]各个灰度级出现的概率  
	//p1[]各个灰度级之前的概率和  
	//各个灰度级出现的次数  
	//*****************  
	assert((img.cols == result.cols) && (img.rows == result.rows));
	double p[256], p1[256], num[256];
	int nheight = img.rows;
	int nwidth = img.cols;
	int total = nheight*nwidth;
	memset(p, 0, sizeof(p));
	memset(p1, 0, sizeof(p1));
	memset(num, 0, sizeof(num));
	//各个灰度级出现的次数  
	for (int i = 0; i < nheight; i++)
	{
		uchar *data = img.ptr<uchar>(i);
		for (int j = 0; j < nwidth; j++)
		{
			num[data[j]]++;
		}
	}

	//各个灰度级出现的概率  
	for (int i = 0; i < 256; i++)
	{
		p[i] = num[i] / total;
	}
	//各个灰度级之前的概率和  
	for (int i = 0; i < 256; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			p1[i] += p[j];
		}
	}

	//直方图变换  
	for (int i = 0; i < nheight; i++)
	{
		uchar *data = img.ptr<uchar>(i);
		uchar *data0 = result.ptr<uchar>(i);
		for (int j = 0; j < nwidth; j++)
		{
			data0[j] = p1[data[j]] * 255 + 0.5;
		}
	}
	return 0;
}

////////////////////////////otsu法阈值确定///////////////////////////////////////////
int otsu(cv::Mat&dst){

	int i, j;
	int tmp;

	double u0, u1, w0, w1, u, uk;

	double cov;
	double maxcov = 0.0;
	int maxthread = 0;

	int hst[MAX_GRAY_VALUE] = { 0 };
	double pro_hst[MAX_GRAY_VALUE] = { 0.0 };

	int height = dst.cols;
	int width = dst.rows;




	//统计每个灰度的数量
	for (i = 0; i<width; i++){
		for (j = 0; j<height; j++){
			tmp = dst.at<uchar>(i, j);
			hst[tmp]++;
		}
	}

	//计算每个灰度级占图像中的概率
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		pro_hst[i] = (double)hst[i] / (double)(width*height);


	//计算平均灰度值
	u = 0.0;
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		u += i*pro_hst[i];

	double det = 0.0;
	for (i = MIN_GRAY_VALUE; i< MAX_GRAY_VALUE; i++)
		det += (i - u)*(i - u)*pro_hst[i];

	//统计前景和背景的平均灰度值，并计算类间方差

	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++){

		w0 = 0.0; w1 = 0.0; u0 = 0.0; u1 = 0.0; uk = 0.0;

		for (j = MIN_GRAY_VALUE; j < i; j++){

			uk += j*pro_hst[j];
			w0 += pro_hst[j];

		}
		u0 = uk / w0;


		w1 = 1 - w0;
		u1 = (u - uk) / (1 - w0);


		//计算类间方差
		cov = w0*w1*(u1 - u0)*(u1 - u0);




		if (cov > maxcov)
		{
			maxcov = cov;
			maxthread = i;
		}
	}

	std::cout << maxthread << std::endl;
	return maxthread;


}

int otsu(cv::Mat&dst, cv::Mat&mask){

	int i, j;
	int tmp;

	double u0, u1, w0, w1, u, uk;

	double cov;
	double maxcov = 0.0;
	int maxthread = 0;

	int hst[MAX_GRAY_VALUE] = { 0 };
	double pro_hst[MAX_GRAY_VALUE] = { 0.0 };

	int height = dst.cols;
	int width = dst.rows;
	int are = 0;



	//统计每个灰度的数量
	for (i = 0; i<width; i++){
		for (j = 0; j<height; j++){
			if (mask.at<uchar>(i, j) != 0)
			{
				are++;
				tmp = dst.at<uchar>(i, j);
				hst[tmp]++;
			}

		}
	}

	//计算每个灰度级占图像中的概率
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		pro_hst[i] = (double)hst[i] / (double)(are);


	//计算平均灰度值
	u = 0.0;
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		u += i*pro_hst[i];

	double det = 0.0;
	for (i = MIN_GRAY_VALUE; i< MAX_GRAY_VALUE; i++)
		det += (i - u)*(i - u)*pro_hst[i];

	//统计前景和背景的平均灰度值，并计算类间方差

	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++){

		w0 = 0.0; w1 = 0.0; u0 = 0.0; u1 = 0.0; uk = 0.0;

		for (j = MIN_GRAY_VALUE; j < i; j++){

			uk += j*pro_hst[j];
			w0 += pro_hst[j];

		}
		u0 = uk / w0;


		w1 = 1 - w0;
		u1 = (u - uk) / (1 - w0);


		//计算类间方差
		cov = w0*w1*(u1 - u0)*(u1 - u0);




		if (cov > maxcov)
		{
			maxcov = cov;
			maxthread = i;
		}
	}
	std::cout << maxthread << std::endl;
	return maxthread;


}