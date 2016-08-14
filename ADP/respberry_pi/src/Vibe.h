#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

#define NUM_SAMPLES 20		//每个像素点的样本个数 更新背景速度（不确定）   20
#define MIN_MATCHES 2		//#min指数                            2
#define RADIUS 10	//Sqthere半径  越大 反应越小                  20
#define SUBSAMPLE_FACTOR 16	//子采样概率                          16


class ViBe_BGS
{
public:
	ViBe_BGS(void);
	~ViBe_BGS(void);

	void init(const Mat _image);   //初始化
	void processFirstFrame(const Mat _image);
	void testAndUpdate(const Mat _image);  //更新
	Mat getMask(void){ return m_mask; };
	void deleteSamples(){ delete samples; };

private:
	unsigned char ***samples;
	//	float samples[1024][1024][NUM_SAMPLES+1];//保存每个像素点的样本值

	/*
	Mat m_samples[NUM_SAMPLES];
	Mat m_foregroundMatchCount;*/

	Mat m_mask;
};