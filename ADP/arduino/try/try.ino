#include <Servo.h>
#include "ang.h"
#include "Max6675_1.h"
#include"pid.h"
#define RelayPin 13
Max6675 ts(9, 10,11);
// 定义我们将要使用的变量
double Setpoint, Input, Output;
//指定链接和最初的调优参数
float error=3;//允许的误差范围
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;
Servo myservo1,myservo2,myservo3;  
String comdata = "";
angle ang={0,0,0};//写默认的角度 暂时先写0,0,0
float x_max=100,y_max=100,z_max=100;
void receive();
void setup() {
  Serial.begin(9600);
  //机械手部分的初始化
  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
  delay(20);
  myservo1.write(0);
  myservo2.write(0);
  myservo3.write(0);
  delay(20);
//控温初始化
  ts.setOffset(0);
  windowStartTime = millis();
  //初始化变量
  Setpoint = 35;
  //告诉PID在从0到窗口大小的范围内取值
  myPID.SetOutputLimits(0, WindowSize);
  //开启PID
  myPID.SetMode(AUTOMATIC);
  lasttemp = ts.getCelsius();

}

void loop() {  
  //接收信号,并处理
  receive();
  xyztoangle(x, y, z, &ang);
  //Serial.println(ang.angle1);
  //Serial.println(ang.angle2);
  //Serial.println(ang.angle3);
  myservo1.write(ang.angle1);
  myservo2.write(ang.angle2);
  myservo3.write(ang.angle3);
  //滤波去除误差，并输入温度

  average_filter(ts.getCelsius(),&Input);
  
  myPID.Compute();//计算是否需要重新计算
 
  /************************************************
   * turn the output pin on/off based on pid output 基于PID输出，打开或关闭端口输出
   ************************************************/
  if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window 继电器窗口时间
    windowStartTime += WindowSize;
  }
  if(Output < millis() - windowStartTime) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);
  if (ts.getCelsius())
  Serial.println(ts.getCelsius(), 2);
  Serial.print(" C / ");

  delay(300);


    
}
void receive(){
  while (Serial.available() > 0)  
  {
    comdata += char(Serial.read());
    delay(2);//为了防止数据丢失,在此设置短暂延时delay(2)
  }
    //将信号转化为1脚的坐标
  if (comdata.length() > 0)
  {
     float c;
     String a;
     int b;     
     switch( comdata[0]){
     case 'x':
       a=comdata.substring(1);
       b=a.toInt();//在此把comdata转化成INT型数值,以备后续使用
       c=(float) b;
       if(c>x_max) c=x_max;
       x=c;
     break;
     case 'y':
       a=comdata.substring(1);
       b=a.toInt();
       c=(float) b;
       if(c>y_max) c=y_max;
       y=c;
     break;
     case 'z':
       a=comdata.substring(1);
       b=a.toInt();
       c=(float) b;
       if(c>z_max) c=z_max;
       z=c;
     break;
   }
   comdata = String("");//清空字符串;
//分别计算出另外两条腿的坐标
  }
}


