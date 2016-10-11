#include <Event.h>
#include <Timer.h>

#include <PID_v1.h>
#include <Max6675.h>
#include "PID_AutoTune_v0.h"
const int relaypin=7;
int pidstep=0;

Timer t; 
byte ATuneModeRemember=2;
double input=30, output=10, setpoint=96;
//double kp=2,ki=0.5,kd=2;
double kp=1.12,ki=0.05,kd=0;
//设置仿真参数
double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
//设置整定参数
double aTuneStep=20, aTuneNoise=1, aTuneStartValue=60;
unsigned int aTuneLookBack=20;//设置选取峰值的时间范围 例如 :小于25时 间隔时间:aTuneLookBack*4*250/1000=aTuneLookBack 
                               //大于25时 间隔时间;100*10*aTuneLookBack/1000=aTuneLookBack 不过大于25后记录的数组大小不会变 依然是100 只是每个成员的时间间隔增加了

boolean tuning = true;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false/true to connect to the real world设置是否用真实参数(是否仿真)
boolean useSimulation = true;
Max6675 ts(8, 9, 10);
void setup()
{
  pinMode(relaypin,OUTPUT);
  ts.setOffset(0);
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)//输入整定参数
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);
  t.every(200, gettemp);
  t.every(50,pidcontrol);
}

void loop()
{
  t.update();
  unsigned long now = millis();

  
  
  if(tuning)
  {
    byte val = (aTune.Runtime());//如果返回1则说明不再需要自整定了 返回0则继续
    if (val!=0)
    {
      tuning = false;//不再整定
    }
    if(!tuning)
    { //we're done, set the tuning parameters设置整定参数
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);//设置mypid模式
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)//100ms进入一次
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
     //analogWrite(0,output); 
     //Serial.println(output);
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)//500ms进入一次
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void pidcontrol(){
  if(output>0){
    if(output>100)output=100;
    if(pidstep<50){
      if(pidstep<output/2){
      digitalWrite(relaypin,HIGH);
      }else{
      digitalWrite(relaypin,LOW);
      }
      pidstep++;
    }else pidstep=0;
  }else digitalWrite(relaypin,LOW);
  
}
void gettemp(){
  if(!useSimulation)
  { //pull the input in from the real world从现实世界拉取input
    //input = analogRead(0);//0~255;
    input=ts.getCelsius();
  }
  
}

void changeAutoTune()//改变整定参数 取反tuning
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.把output设置成所需的起始频率?
    output=aTuneStartValue;//100
    aTune.SetNoiseBand(aTuneNoise);//1
    aTune.SetOutputStep(aTuneStep);//50
    aTune.SetLookbackSec((int)aTuneLookBack);//20
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

//串口接收与发送
void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } 
  else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}
//仿真更新input值
void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];//往0退一位
  }
  //compute the input计算输入
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;//0.1的随机数偏差

}
