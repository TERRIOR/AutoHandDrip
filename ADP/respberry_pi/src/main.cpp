#include "Vibe.h"
#include <cstdio>

using namespace cv;
using namespace std;
#define MAX_GRAY_VALUE 256
#define MIN_GRAY_VALUE 0
const int g_cannyLowThreshold =3;
const int c_rectr = 20;
int getarea(Mat &img);
int otsu(cv::Mat&dst);
int otsu(cv::Mat&dst, cv::Mat&mask);
bool isclose(vector<Point> contour);
bool comp(const vector<Point> &a, const  vector<Point> &b);
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
	int c_size_max,c_size_min;
	Mat lastmask,lastmask2;
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
		if ((!isclose(*itc)) || itc->size()<cmin || itc->size()>cmax)
			itc = contours.erase(itc);
		else
		{
			++itc;
			
		}
		
	}
	//cout << "Contours: " << contours.size() << "  " << ends;
	//cout << endl;
	if (contours.size() > 0){
		itc = contours.begin();
		int i = 0;
		while (itc != contours.end())
		{
			Scalar color(40 * i, 255, 30 * i);
			drawContours(framecp, contours, i, color, 2);
			itc++;
			i++;
		}
		sort(contours.begin(), contours.end(), comp);//对轮廓进行排序
		c_size_max = contours[0].size();//取出最大轮廓
		c_size_min = contours[1].size();//取出第二大的轮廓
		ellisperectmax = fitEllipse(Mat(contours[0]));
		RotatedRect ellispemin = fitEllipse(Mat(contours[1]));
		cout << "height:" << ellisperectmax.size.height << "width:" << ellisperectmax.size.width << "area:" << ellisperectmax.size.area() << endl;
		re = boundingRect(contours[0]);//roi感兴趣区域，其他去除
		ellipse(framecp, ellisperectmax, Scalar(rand() & 255, rand() & 255, rand() & 255), 8);
		imshow("framecp", framecp);
		cmask = Mat::zeros(frame.size(), CV_8UC1);
		lastmask=Mat::zeros(frame.size(), CV_8UC1);
		lastmask2 = Mat::zeros(frame.size(), CV_8UC1);
		cmask.setTo(0);
		lastmask.setTo(255);
		lastmask2.setTo(255);
		ellipse(cmask, ellisperectmax, Scalar(255), -1);
		ellipse(lastmask, RotatedRect(ellispemin.center, Size(ellispemin.size.width - 100, ellispemin.size.height -100), ellispemin.angle), Scalar(0), -1);
		ellipse(lastmask2, RotatedRect(ellispemin.center, Size(ellispemin.size.width +100, ellispemin.size.height + 100), ellispemin.angle), Scalar(0), -1);
		cmask = cmask(Rect(re.x + 15, re.y + 15, re.size().width - 30, re.size().height - 30));
		lastmask = lastmask(Rect(re.x + 15, re.y + 15, re.size().width - 30, re.size().height - 30));
		lastmask2 = lastmask2(Rect(re.x + 15, re.y + 15, re.size().width - 30, re.size().height - 30));
		ellisperectmax.center.x = ellisperectmax.center.x - re.x-15;//消除切除后圆心的偏移
		ellisperectmax.center.y = ellisperectmax.center.y - re.y-15;//同上
		//Mat cmask2 = Mat::zeros(frame.size(), CV_8UC1);
		//circle(cmask2, c_point, c_rectr, Scalar(255));
		//imshow("ellipse", cmask);
		//imshow("ellipse2", lastmask);
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
			
			Mat element = getStructuringElement(MORPH_ELLIPSE, Size(15,15));
			morphologyEx(mask, mask, MORPH_OPEN, element);
			
			int mask_area = countNonZero(mask);//这里的mask需要改成只有注水杆区域（目前还包括粉层的扩散）
			//mask_area=getarea(mask);
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
			//Mat frame3;
			//frame.copyTo(frame3, mask);
			//imshow("frame3", frame3);
			
			/*vibe运动算法去除注水区域（暂时不用）
			f1.copyTo(frame2, mask);
			mask = 255 - mask;//mask取反 此时mask为没运动区域
			frame.copyTo(frame2, mask);
			Mat frame2_gray;
			cvtColor(frame2, frame2_gray, CV_RGB2GRAY);
			GaussianBlur(frame2_gray, frame2_gray, Size(3, 3), 0, 0);
			element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
			morphologyEx(frame2_gray, frame2_gray, MORPH_OPEN, element);
			imshow("f2", frame2_gray);//消除遮挡后mat
			*/
		
			
			Mat dst1;
			GaussianBlur(gray, gray, Size(21, 21), 0, 0);
			//morphologyEx(gray, gray, MORPH_CLOSE, element);
			//mask = cmask&mask&lastmask;//此时mask为椭圆内没运动区域
			//imshow("ellispe", mask);
			int a = otsu(gray, cmask);//只对cmask二值化（避免四个角剩余的蓝黑色部分影响）
			frame_gs = gray > a;
			Mat c_gray;
			frame_gs.copyTo(c_gray, cmask);
			c_gray = (c_gray&lastmask)|lastmask2;//lastmask与lastmask2限制了粉圈范围，粉圈的二值化图黑圈大于lastmask，小于lastmask2
			//dilate (c_gray, c_gray, element);//腐蚀操作
			element = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
			morphologyEx(c_gray, c_gray, MORPH_CLOSE, element);
			imshow("dst", c_gray);
			Canny(c_gray, c_gray, g_cannyLowThreshold, g_cannyLowThreshold * 3, 3);
			findContours(c_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			vector<vector<Point> >::iterator itContours = contours.begin();
			for (; itContours != contours.end(); ++itContours) {

				cout << "Size: " << itContours->size() << ends;//每个轮廓包含的点数
			}
			cout << "Contours: " << contours.size() << c_size_min << endl;
			unsigned int cmin = c_size_min-100;//把上次检测的结果作为下次检测的筛选条件
			unsigned int cmax =c_size_min+50;//
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
					c_size_min = contours[j].size();
					ellipse(framecp, rrect, Scalar(rand() & 255, rand() & 255, rand() & 255), 8);
					lastmask.setTo(255);
					lastmask2.setTo(255);
					//以这次检测出的圆去限制（补全&去除）下一个圆   范围：-10~30（待调整）
					ellipse(lastmask2, RotatedRect(rrect.center, Size(rrect.size.width +30, rrect.size.height +30), rrect.angle), Scalar(0), -1);
					ellipse(lastmask, RotatedRect(rrect.center, Size(rrect.size.width - 10, rrect.size.height - 10), rrect.angle), Scalar(0), -1);
					Rect re = boundingRect(Mat(contours[j]));//roi感兴趣区域，其他去除
					//int a = otsu(gray);
					Mat fc= gray(re);
					fc = fc > 100;//取出单独处理  fc = fc > a;
					int contarea=contourArea(contours[j]);
					int blackarea =fc.rows*fc.cols- countNonZero(fc);
					double prewhite =  (contarea - blackarea)*100/ contarea;
					cout << prewhite<<"%"<< "  "<<contarea << "  "<<blackarea<<  endl;
					imshow("quanli", fc);
					imshow("framecp", framecp);//绘制椭圆轮廓后的图像
				}
				
			}
			
		}
		//imshow("input", frame);//原图

		if (cvWaitKey(10) == 27)
			break;
	}

	return 0;
}


bool comp(const vector<Point> &a, const  vector<Point> &b)//soft排序的自定义排序函数
{
	return a.size()>b.size();
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
int getarea(Mat &img){
	int i = 0,j=0;
	int height = img.cols;
	int width = img.rows;
	int count=0;
	for (i = 0; i<width; i++){
		for (j = 0; j<height; j++){
			
			if (img.at<char>(i,j)=0)
			{
				count++;
			}
		}
	}
	return count;
}
////////////////////////////////灰度直方图增强对比度///////////////////////////////////////////////////
int ImgStrong(Mat &img, Mat &result)
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