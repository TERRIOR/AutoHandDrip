#include <Event.h>
#include <Timer.h>
#include <Servo.h>
#include "ang.h"
#include "Max6675_1.h"
#include"pid.h"
#define RelayPin 13
Timer t;                               //instantiate the timer object
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
char com;
angle ang={0,0,0};//写默认的角度 暂时先写0,0,0
//int x_max=100,y_max=100,z_max=100;
void sreceive();
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //机械手部分的初始化
  myservo1.attach(4);
  myservo2.attach(5);
  myservo3.attach(6);
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
  t.update();
  //滤波去除误差，并输入温度
  average_filter(ts.getCelsius(),&Input);
  //Serial.print(Input);
  //Serial.print(",");
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
  //Serial.println(ts.getCelsius(), 2);
 // Serial.println(" C / ");

  delay(300);  
}
  sreceive();
  xyztoangle(x, h, y, &ang);
  if(isdrip){
    Serial.print("  ");
    Serial.print(ang.angle1);
    Serial.print("  ");
    Serial.print(ang.angle2);
    Serial.print("  ");
    Serial.println(ang.angle3);
  }

  //myservo1.write(ang.angle1);
  //myservo2.write(ang.angle2);
  //myservo3.write(ang.angle3);
}
void sreceive(){
   while (Serial1.available() > 0 )  
    {
      com=char(Serial1.read());
      if(com==' '){
        break;
      }
      comdata += com;
      delay(6);
      
    }
    if (comdata.length() > 0)
    {
      
      int yindex=0;
      int hindex=0;
      switch(comdata[0]){
        case 'x':
            isdrip=true;
            yindex=comdata.indexOf('y',0);
            x=comdata.substring(1,yindex).toInt();
            hindex=comdata.indexOf('h',0);
            //if comdata contain the 'h' get the h 
            if(hindex!=-1){
              y=comdata.substring(yindex+1,hindex).toInt();
            }else {
              y=comdata.substring(yindex+1).toInt();
            }
        break;//change the "x"and"y" it is possible that the high change as well; 
        case 'h':
          h=comdata.substring(1).toInt()-100;
        break;//only change the "high"
        case '0':
          isdrip=false;
        break;//stop pour water
      }
      if(isdrip){
        Serial.print("x:");
        Serial.print(x);
        Serial.print("y:");
        Serial.print(y);
        Serial.print("h:");
        Serial.println(h);
      }else{
        Serial.println("not drip");
      }
      //Serial.print(comdata);
      comdata = "";
    }
}


