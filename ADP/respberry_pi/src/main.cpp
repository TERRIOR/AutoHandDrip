#include "Vibe.h"
#include <cstdio>

using namespace cv;
using namespace std;
#define MAX_GRAY_VALUE 256
#define MIN_GRAY_VALUE 0
const int g_cannyLowThreshold = 3;
int getarea(Mat img);
int otsu(cv::Mat&dst);
int otsu(cv::Mat&dst, cv::Mat&mask);
int main(int argc, char* argv[])
{
	Mat frame, gray, mask ,f1;
	VideoCapture capture;
	
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
	//GaussianBlur(f1, f1, Size(3, 3), 0, 0);
	////////////////////////////////ȡ���������˱�Ȧ//////////////////////////////
	Mat frame_gray,framecp;
	Mat cmask;
	frame.copyTo(framecp);
	cvtColor(frame, frame_gray, CV_RGB2GRAY);
	GaussianBlur(frame_gray, frame_gray, Size(35, 35), 0, 0);//��˹����Ϊ����
	//imshow("gaosi", frame_gray);
	//imshow("��ֵ��", dst);
	//bilateralFilter(frame_gray, dst1, 35, 35 * 2, 35 / 2);
	//imshow("˫���˲�", frame_gray);
	//threshold(dst1, dst,0, 255, THRESH_BINARY|THRESH_OTSU);
	int a = otsu(frame_gray);//�����㷨
	cout << "otsu��ֵ����" << a << endl;
	frame_gray = frame_gray > a;
	//ImgStrong(dst, dst);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(12, 12));
	//������̬ѧ����  
	morphologyEx(frame_gray, frame_gray, MORPH_OPEN, element);

	Canny(frame_gray, frame_gray, g_cannyLowThreshold, g_cannyLowThreshold * 3, 3);
	//imshow("canny", dst);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(frame_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int in = 0;

	//////////////////////////////////////////////////////

	vector<vector<Point> >::const_iterator itContours = contours.begin();
	for (; itContours != contours.end(); ++itContours) {

		cout << "Size: " << itContours->size() << ends;//ÿ�����������ĵ���
	}
	cout << endl;
	cout << "Contours: " << contours.size() << endl;
	unsigned int cmin = 300;
	unsigned int cmax = 2000;
	vector<vector<Point> >::iterator itc = contours.begin();
	int last=0;
	while (itc != contours.end())
	{
		if (itc->size()<cmin || itc->size()>cmax  )
			itc = contours.erase(itc);
		else
		{
			++itc;
			
		}
		
	}
	cout << "Contours: " << contours.size() << "  " << ends;
	cout << endl;
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
		//cout << "max:" << max << "��" << j << endl;
		RotatedRect rrect = fitEllipse(Mat(contours[j]));
		ellipse(framecp, rrect, Scalar(rand() & 255, rand() & 255, rand() & 255), 8);
		imshow("framecp", framecp);
		cmask = Mat::zeros(frame.size(), CV_8UC1);
		cmask.setTo(0);
		ellipse(cmask, rrect, Scalar(255), -1);
		imshow("ellipse", cmask);
	}
	while (1)
	{
		capture >> frame;
		if (frame.empty())
			continue;
		cout << "rows:"<<frame.rows<<"col:"<<frame.cols<< endl;
		cvtColor(frame, gray, CV_RGB2GRAY);
		if (count)
		{
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
			mask_area=getarea(mask);
			cout << "area:"<<mask_area<< endl;

			if (mask_area<4000)
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
			imshow("frame3", frame3);
			//GaussianBlur(frame, frame_gs, Size(3, 3), 0, 0);
			f1.copyTo(frame2, mask);
			mask = 255 - mask;//maskȡ�� ��ʱmaskΪû�˶�����
			frame.copyTo(frame2, mask);
			Mat frame2_gray;
			cvtColor(frame2, frame2_gray, CV_RGB2GRAY);
			//GaussianBlur(frame2_gray, frame2_gray, Size(3, 3), 0, 0);
			//element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
			//������̬ѧ����  
			//morphologyEx(frame2_gray, frame2_gray, MORPH_OPEN, element);
			Mat dst1;
			//mask = cmask&mask;//��ʱmaskΪ��Բ��û�˶�����
			int a = otsu(gray, cmask);
			gray = gray > a;
			imshow("dst", gray);
			imshow("f2", frame2_gray);
		}


		/*
				vector<Mat> rgb_planes;
		Mat frame_hsv;
		cvtColor(frame, frame_hsv, CV_BGR2HSV_FULL);
		split(frame_hsv, rgb_planes);
		/// �趨bin��Ŀ
		int histSize = 255;

		/// �趨ȡֵ��Χ ( R,G,B) )
		float range[] = { 0, 255 };
		const float* histRange = { range };

		bool uniform = true; bool accumulate = false;

		Mat r_hist, g_hist, b_hist;

		/// ����ֱ��ͼ:
		calcHist(&rgb_planes[0], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&rgb_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
		calcHist(&rgb_planes[2], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);

		// ����ֱ��ͼ����
		int hist_w = 400; int hist_h = 400;
		int bin_w = cvRound((double)hist_w / histSize);

		Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0, 0, 0));

		/// ��ֱ��ͼ��һ������Χ [ 0, histImage.rows ]
		normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
		normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

		/// ��ֱ��ͼ�����ϻ���ֱ��ͼ
		for (int i = 1; i < histSize; i++)
		{
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
				Scalar(0, 0, 255), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
				Scalar(0, 255, 0), 2, 8, 0);
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
				Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
				Scalar(255, 0, 0), 2, 8, 0);
		}

		/// ��ʾֱ��ͼ
		namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE);
		imshow("calcHist Demo", histImage);
		*/


		imshow("input", frame);

		if (cvWaitKey(10) == 27)
			break;
	}

	return 0;
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
////////////////////////////////�Ҷ�ֱ��ͼ��ǿ�Աȶ�///////////////////////////////////////////////////
int ImgStrong(Mat img, Mat result)
{
	//***************  
	//p[]�����Ҷȼ����ֵĸ���  
	//p1[]�����Ҷȼ�֮ǰ�ĸ��ʺ�  
	//�����Ҷȼ����ֵĴ���  
	//*****************  
	assert((img.cols == result.cols) && (img.rows == result.rows));
	double p[256], p1[256], num[256];
	int nheight = img.rows;
	int nwidth = img.cols;
	int total = nheight*nwidth;
	memset(p, 0, sizeof(p));
	memset(p1, 0, sizeof(p1));
	memset(num, 0, sizeof(num));
	//�����Ҷȼ����ֵĴ���  
	for (int i = 0; i < nheight; i++)
	{
		uchar *data = img.ptr<uchar>(i);
		for (int j = 0; j < nwidth; j++)
		{
			num[data[j]]++;
		}
	}

	//�����Ҷȼ����ֵĸ���  
	for (int i = 0; i < 256; i++)
	{
		p[i] = num[i] / total;
	}
	//�����Ҷȼ�֮ǰ�ĸ��ʺ�  
	for (int i = 0; i < 256; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			p1[i] += p[j];
		}
	}

	//ֱ��ͼ�任  
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

////////////////////////////otsu����ֵȷ��///////////////////////////////////////////
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




	//ͳ��ÿ���Ҷȵ�����
	for (i = 0; i<width; i++){
		for (j = 0; j<height; j++){
			tmp = dst.at<uchar>(i, j);
			hst[tmp]++;
		}
	}

	//����ÿ���Ҷȼ�ռͼ���еĸ���
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		pro_hst[i] = (double)hst[i] / (double)(width*height);


	//����ƽ���Ҷ�ֵ
	u = 0.0;
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		u += i*pro_hst[i];

	double det = 0.0;
	for (i = MIN_GRAY_VALUE; i< MAX_GRAY_VALUE; i++)
		det += (i - u)*(i - u)*pro_hst[i];

	//ͳ��ǰ���ͱ�����ƽ���Ҷ�ֵ����������䷽��

	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++){

		w0 = 0.0; w1 = 0.0; u0 = 0.0; u1 = 0.0; uk = 0.0;

		for (j = MIN_GRAY_VALUE; j < i; j++){

			uk += j*pro_hst[j];
			w0 += pro_hst[j];

		}
		u0 = uk / w0;


		w1 = 1 - w0;
		u1 = (u - uk) / (1 - w0);


		//������䷽��
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



	//ͳ��ÿ���Ҷȵ�����
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

	//����ÿ���Ҷȼ�ռͼ���еĸ���
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		pro_hst[i] = (double)hst[i] / (double)(are);


	//����ƽ���Ҷ�ֵ
	u = 0.0;
	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++)
		u += i*pro_hst[i];

	double det = 0.0;
	for (i = MIN_GRAY_VALUE; i< MAX_GRAY_VALUE; i++)
		det += (i - u)*(i - u)*pro_hst[i];

	//ͳ��ǰ���ͱ�����ƽ���Ҷ�ֵ����������䷽��

	for (i = MIN_GRAY_VALUE; i<MAX_GRAY_VALUE; i++){

		w0 = 0.0; w1 = 0.0; u0 = 0.0; u1 = 0.0; uk = 0.0;

		for (j = MIN_GRAY_VALUE; j < i; j++){

			uk += j*pro_hst[j];
			w0 += pro_hst[j];

		}
		u0 = uk / w0;


		w1 = 1 - w0;
		u1 = (u - uk) / (1 - w0);


		//������䷽��
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
