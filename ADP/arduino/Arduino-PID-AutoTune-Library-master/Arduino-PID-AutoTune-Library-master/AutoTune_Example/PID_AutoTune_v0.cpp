#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID_AutoTune_v0.h"


PID_ATune::PID_ATune(double* Input, double* Output)
{
	input = Input;//指针的赋值 因此这里改变input 外面的input也会改变
	output = Output;
	controlType =0 ; //default to PI  默认是pi
	noiseBand = 0.5;//噪声
	running = false;
	oStep = 30;//?
	SetLookbackSec(10);//samptime 为250
	lastTime = millis();
	
}



void PID_ATune::Cancel()
{
	running = false;
} 
 
int PID_ATune::Runtime()
{
  /*
  running=false时
  还没运行过
  第一次初始化参数 主要是 setpoint 
  running=ture时
  不断选出极值
  加入峰值数值
  自整定完成条件(return 1):
  1,峰值大于9
  2,峰值大于3 且 峰峰均值小于0.05*最大差值
  */
	justevaled=false;
	if(peakCount>9 && running)//running==true 说明不是第一次运行 峰值数目>9
	{
		running = false;
		FinishUp();
		return 1;
	}
	unsigned long now = millis();
	
	if((now-lastTime)<sampleTime) return false;//return 0 不改变pid参数
	lastTime = now;
	double refVal = *input;
	justevaled=true;
	if(!running)
	{ //initialize working variables the first time around变量(variables)第一次初始化
		peakType = 0;
		peakCount=0;
		justchanged=false;
		absMax=refVal;
		absMin=refVal;
                //setpoint=
		setpoint = refVal;
		running = true;
		outputStart = *output;
		*output = outputStart+oStep;//一开始为自加  oStep= (初始化)
	}
	else
	{
		if(refVal>absMax)absMax=refVal;//absmax为最大值(不是峰值)
		if(refVal<absMin)absMin=refVal;//最小值
	}
	
	//oscillate the output base on the input's relation to the setpoint输出的震荡基于input与setpoint之间的关系
	
	if(refVal>setpoint+noiseBand) *output = outputStart-oStep;//如果input大于setpoint加误差 
	else if (refVal<setpoint-noiseBand) *output = outputStart+oStep;
	
	
  //bool isMax=true, isMin=true;
  isMax=true;isMin=true;
  //id peaks
  for(int i=nLookBack-1;i>=0;i--)
  {
    double val = lastInputs[i];
    if(isMax) isMax = refVal>val;//判断refval是否所有输入的最大或最小值
    if(isMin) isMin = refVal<val;
    lastInputs[i+1] = lastInputs[i];
  }
  lastInputs[0] = refVal;  //输入数组
  if(nLookBack<9)//注意这里是nLookBack 不是peakcount()     这段判断意义不明
  {  //we don't want to trust the maxes or mins until the inputs array has been filled我们不想相信高峰或最小值直到输入数组已经满了
	return 0;
  }
  
  if(isMax)
  {
    if(peakType==0)peakType=1;
    if(peakType==-1)
    {
      peakType = 1;
      justchanged=true;
      peak2 = peak1;
    }
    peak1 = now;
    peaks[peakCount] = refVal;
   
  }
  else if(isMin)
  {
    if(peakType==0)peakType=-1;
    if(peakType==1)//如果上次为峰顶
    {
      peakType=-1;
      peakCount++;//这次为峰谷 峰数量加一
      justchanged=true;
    }
    
    if(peakCount<10)peaks[peakCount] = refVal;//更新峰值数组
  }
  
  if(justchanged && peakCount>2)//刚更新 且 峰值有三个以上
  { //we've transitioned.  check if we can autotune based on the last peaks我们已经转换 检查是否可以根据最后山峰去自整定
    double avgSeparation = (abs(peaks[peakCount-1]-peaks[peakCount-2])+abs(peaks[peakCount-2]-peaks[peakCount-3]))/2;//peaks[peakCount-2]与它周围的两点差的平方 峰峰值平均值
    if( avgSeparation < 0.05*(absMax-absMin))//如:烧水 最高温度100度 最低温30度 当峰峰值均值小于3.5(70*0.05)时则判断为自整定完成
    {
      FinishUp();
      running = false;
      return 1;
	 
    }
  }
   justchanged=false;
	return 0;
}
void PID_ATune::FinishUp()
{
      *output = outputStart;
      //we can generate tuning parameters!我们可以生成整定参数
      Ku = 4*(2*oStep)/((absMax-absMin)*3.14159);
      Pu = (double)(peak1-peak2) / 1000;
}

double PID_ATune::GetKp()
{
	return controlType==1 ? 0.6 * Ku : 0.4 * Ku;
}

double PID_ATune::GetKi()
{
	return controlType==1? 1.2*Ku / Pu : 0.48 * Ku / Pu;  // Ki = Kc/Ti
}

double PID_ATune::GetKd()
{
	return controlType==1? 0.075 * Ku * Pu : 0;  //Kd = Kc * Td
}

void PID_ATune::SetOutputStep(double Step)
{
	oStep = Step;
}

double PID_ATune::GetOutputStep()
{
	return oStep;
}

void PID_ATune::SetControlType(int Type) //0=PI, 1=PID
{
	controlType = Type;
}
int PID_ATune::GetControlType()
{
	return controlType;
}
	
void PID_ATune::SetNoiseBand(double Band)
{
	noiseBand = Band;
}

double PID_ATune::GetNoiseBand()
{
	return noiseBand;
}

void PID_ATune::SetLookbackSec(int value)
{
    if (value<1) value = 1;
	
	if(value<25)
	{
		nLookBack = value * 4;
		sampleTime = 250;
	}
	else
	{
		nLookBack = 100;
		sampleTime = value*10;
	}
}

int PID_ATune::GetLookbackSec()
{
	return nLookBack * sampleTime / 1000;
}
