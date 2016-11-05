#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION        1.0.0
 extern float lasttemp;
 //extern int over_cnt;
 //extern float filter_sum;;
/*不需要其他文件看到
  extern int TFILTER_N;
  extern float Buffer[3];
*/


//void filter(float c,int error,double *input);
void average_filter(float c,double *input);



class PID
{
 
 
  public:
 
  //Constants used in some of the functions below                 在下面函数中使用的常数
  #define AUTOMATIC        1
  #define MANUAL        0
  #define DIRECT  0
  #define REVERSE  1
 
  //commonly used functions ************************************************************************  通常使用的函数
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and           构造函数。连接PID的输入、输出和设置点。
        double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here                        初始化调优参数。
         
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)                                                 设置PID为自动或者手动模式。
 
    bool Compute();                       // * performs the PID calculation.  it should be                                                                         执行PID计算。
                                          //   called every time loop() cycles. ON/OFF and                                                                                                 它可以在每次loop循环的时候调用。
                                          //   calculation frequency can be set using SetMode                                                                                         开关状态和计算频率可以用SetMode和SetSanmpleTime分别地设置。
                                          //   SetSampleTime respectively
 
    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but                限制输出在一个特定的范围。默认是0-255，
                                                                                  //it's likely the user will want to change this depending on                                                                                但是用户可以根据实际应用改变这里。
                                                                                  //the application
         
 
 
  //available but not commonly used functions ********************************************************         可以使用但通常不用的函数
    void SetTunings(double, double,       // * While most users will set the tunings once in the                                  当大多数用户在构造函数中设置一次调节参数，
                    double);                   //   constructor, this function gives the user the option                                                                 这个函数给用户一个在运行过程中改变调节参数的选择                                
                                          //   of changing tunings during runtime for Adaptive control                                                                 
        void SetControllerDirection(int);          // * Sets the Direction, or "Action" of the controller. DIRECT                 设置方向，或者说控制的动作。
                                                                                  //   means the output will increase when error is positive. REVERSE                                 Direct意味着当误差为正时，输出增加。Reverse反之。
                                                                                  //   means the opposite.  it's very unlikely that this will be needed                                         这通常只在构造函数中设置一次。
                                                                                  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which                                                          设置PID计算出的频率，单位是ms。默认是100
                                          //   the PID calculation is performed.  default is 100
                                                                                   
                                                                                   
                                                                                   
  //Display functions ****************************************************************                                                                 显示函数
        double GetKp();                                                  // These functions query the pid for interal values.                                                                 函数确定PID的区间值。
        double GetKi();                                                  //  they were created mainly for the pid front-end,                                                                 它们主要为PID前端而创建。
        double GetKd();                                                  // where it's important to know what is actually                                                                   这个前端对于了解PID内部实际工作很重要。
        int GetMode();                                                  //  inside the PID.
        int GetDirection();                                          //
 
  private:
        void Initialize();
         
        double dispKp;                                // * we'll hold on to the tuning parameters in user-entered                                                           出于显示的目的，我们将以用户输入的格式保持调节参数
        double dispKi;                                //   format for display purposes
        double dispKd;                                //
     
        double kp;                  // * (P)roportional Tuning Parameter                                                                                                                                           P参数
    double ki;                  // * (I)ntegral Tuning Parameter                                                                                                                                                           I 参数
    double kd;                  // * (D)erivative Tuning Parameter                                                                                                                                                  D参数
 
        int controllerDirection;
 
    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables                                                               输入、输出和设置点变量的指针
    double *myOutput;             //   This creates a hard link between the variables and the                                                           这生成一个硬链接，在变量和PID之间
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us                                                           从不得不持续告诉我们这些值是什么中，解放了用户，我们只需要指针
                                  //   what these values are.  with pointers we'll just know.
                           
        unsigned long lastTime;
        double ITerm, lastInput;
 
        unsigned long SampleTime;
        double outMin, outMax;
        bool inAuto;
};
#endif
