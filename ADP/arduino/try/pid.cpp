/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <[url=mailto:br3ttb@gmail.com]br3ttb@gmail.com[/url]> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/
 
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
 
#include "pid.h"
float lasttemp;
int TFILTER_N=3;
float Buffer[3];
 
/*Constructor (...)*********************************************************                                构造函数
 *    The parameters specified here are those for for which we can't set up                                 这里指定的参数是那些我们不能建立可靠的预设值，
 *    reliable defaults, so we need to have the user set them.                                                                                因此我们需要有用户来设置它们。
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
         
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
        inAuto = false;//设置对应指针
         
        PID::SetOutputLimits(0, 255);                                //default output limit corresponds to                                         默认输出限制，是由Arduino PWM限制的
                                                                                                //the arduino pwm limits
 
    SampleTime = 300;                                                        //default Controller Sample Time is 0.1 seconds                默认控制采样时间0.1s
 
    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);
 
    lastTime = millis()-SampleTime;                                
}
  
  
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 *   
 *    计算
 *   这个函数在每次执行loop的时候调用。这个函数将决定是否计算一个新的PID输出。
 *   当输出计算完成后返回真，当什么都没完成后返回假。
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/                                        // 计算所有的工作误差变量
          double input = *myInput;
      double error = *mySetpoint - input;                                                                        // error=设定点-输入值    例如控温系统中，当温度小于设定点，error为正
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
  
      /*Compute PID Output*/                                                                                                        // 计算PID输出
      double output = kp * error + ITerm- kd * dInput;
       
          if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
          *myOutput = output;
           
      /*Remember some variables for next time*/                                     // 为下次计算记录一些变量
      lastInput = input;
      lastTime = now;
          return true;
   }
   else return false;
}
 
 
/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 * 设置调节参数
 * 这个函数允许将控制器的动态性能被调整。
 * 它从构造函数中被自动地调用，但是调节也可以在正常运行期间动态地调整。
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
  
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
    
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
  
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
   
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed        
 * 设置采样时间
 * 设置计算执行的周期，以ms为单位。
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
  
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 * 设置输出限制
 * 这个函数将比SetInputLimits函数更常用。当控制器输入生成到0-1023的范围（已经是默认），
 * 输出将有一点不同。可能它们将执行一个时间窗口，或者需要0-8000等。
 * 或者它们可能想要限制在0-125. 无论如何，都可以在这里设置。
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
  
   if(inAuto)
   {
           if(*myOutput > outMax) *myOutput = outMax;
           else if(*myOutput < outMin) *myOutput = outMin;
          
           if(ITerm > outMax) ITerm= outMax;
           else if(ITerm < outMin) ITerm= outMin;
   }
}
 
/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 * 设置模式
 * 允许控制器模式设定成手动（0）和自动（非0）。
 * 当从手动转成自动，控制器自动初始化
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto     我们只是从手动到自动*/  
        PID::Initialize();
    }
    inAuto = newAuto;
}
  
/* Initialize()****************************************************************
 *        does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 * 初始化
 * 做所有需要发生的事情以保证从手动到自动无扰动转移。
 ******************************************************************************/
void PID::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInput;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}
 
/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 * 设置控制方向
 * PID可以连接到正向作用过程（正输出导致正输入）和反向作用过程（正输出导致负输入）
 * 我们需要知道是哪个，否则我们可能在应该减小时增加输出。这是从构造函数调用的。
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
          kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}
//滤波去除误差
void filter(float c,int error,double *input){
  double temp = c;
       if (temp-lasttemp<error){
           if (temp-lasttemp>-error){
            *input=temp;
            lasttemp = temp; 
           }
           else *input = temp;
        }
        else *input = temp; 
  }

  //滤波加平均值去除误差
  void average_filter(float c,int error,double *input){
       double temp = c;
       int i;
        float  filter_sum ; 
         if (temp-lasttemp<error){
           if (temp-lasttemp>-error){
            Buffer[TFILTER_N - 1] = lasttemp;
           }
           else {
            Buffer[TFILTER_N - 1] = temp;
            lasttemp = temp;
           }
        }
        else {
          Buffer[TFILTER_N - 1] = temp;
          lasttemp = temp; 
        }
        
        for(i = 0; i < TFILTER_N - 1; i++) {
          Buffer[i] =Buffer[i + 1];
          filter_sum += Buffer[i]; 
  }
    //tp->buffer[TFILTER_N - 2]=filter_sum / (TFILTER_N - 1);
    temp= filter_sum / (TFILTER_N - 1);
    *input = temp;
}

 
/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 * 状态函数
 * 仅仅因为你设置了Kp = 1并不意味着它真的发生了。
 * 函数查询PID的内部状态。它们在这里是为了显示的目的。
 * 这是PID前端使用的功能。
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}
