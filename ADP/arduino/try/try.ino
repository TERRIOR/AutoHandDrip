#include <Event.h>
#include <Timer.h>
#include <Servo.h>
#include<stdio.h>
#include<math.h>
#include "ang.h"
#include "Max6675_1.h"
#include"pid.h"
#define RelayPin 22
#include <FlexiTimer2.h>
/*
6,7,8 are used at max6675
10,11,12 are used at three servo which drive the machine hand 
*/
bool waterstate=LOW;
const int waterpin=2;
const int ledpin=13;
const int relaypump=12;
bool reached=false;
int pidstep=0;
const int relaypin=7;
Timer t;                               //instantiate the timer object
Max6675 ts(6,7,8);//原为9,10,11没必要
// 定义我们将要使用的变量
double Setpoint=90, Input, Output;
double kp=1.55,ki=0.01,kd=0;
//指定链接和最初的调优参数
float error=3;//允许的误差范围
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;
Servo myservo1,myservo2,myservo3;  
String comdata = "";
char com;
angle ang={0,0,0};//写默认的角度 暂时先写0,0,0
//int x_max=100,y_max=100,z_max=100;
void sreceive();
void movetop();
void receivesrh();
void movetop2();
void watchreach();
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  //机械手部分的初始化
  myservo1.attach(22);//10
  myservo2.attach(24);//11
  myservo3.attach(26);
  delay(20);
  myservo1.write(93);
  myservo2.write(88);
  myservo3.write(95);
  delay(20);
  //控温初始化
  ts.setOffset(0);
  windowStartTime = millis();
  //初始化变量
  
  //告诉PID在从0到窗口大小的范围内取值
  //myPID.SetOutputLimits(0, WindowSize);
  //开启PID
  myPID.SetMode(AUTOMATIC);
  lasttemp = ts.getCelsius();
  FlexiTimer2::set(20, 1.0/1000, movetop2); // call every 500 1ms "ticks"
  // FlexiTimer2::set(500, flaqsh); // MsTimer2 style is also supported
  FlexiTimer2::start();
  t.every(200, gettemp);
  t.every(50,pidcontrol);
  //t.every(100, movetop);
}

void loop() {  
 
  watchreach();
  t.update();
  //t.update();
  //movetop();
  receivesrh();
  //滤波去除误差，并输入温度
  
  //average_filter(ts.getCelsius(),&Input);
  //Serial.print(Input);
  //Serial.print(",");
  //myPID.Compute();//计算是否需要重新计算
  /************************************************
   * turn the output pin on/off based on pid output 基于PID输出，打开或关闭端口输出
   ************************************************/
  /*if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window 继电器窗口时间
    windowStartTime += WindowSize;
  }
  if(Output < millis() - windowStartTime) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);
  if (ts.getCelsius()) delay(300);*/
  
  //Serial.println(ts.getCelsius(), 2);
 // Serial.println(" C / ");
   
}
void movetop(){
  sreceive();
  if(isdrip){
    setinmax(&x,45,-45);
    setinmax(&y,45,-45);
    setinmax(&h,-70,-150);
    xyztoangle(x, h, y, &ang);
    Serial.print("x:");
    Serial.print(x);
    Serial.print("y:");
    Serial.print(y);
    Serial.print("h:");
    Serial.println(h);
    Serial.print("  ");
    Serial.print(ang.angle1);
    Serial.print("  ");
    Serial.print(ang.angle2);
    Serial.print("  ");
    Serial.println(ang.angle3);
    isdrip=false;  
    myservo1.write(initangle+ang.angle1);
    myservo2.write(initangle+ang.angle2);
    myservo3.write(initangle+ang.angle3); 
  }else{
    myservo1.write(initangle+ang.angle1);
    myservo2.write(initangle+ang.angle2);
    myservo3.write(initangle+ang.angle3); 
    //Serial.println("nodrip");
  }
  //*1.5 so that map the angle from 0~120 to 0~180
  //initangle -ang.angle: change the angle to servo angle 

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
            if(hindex>0){
              h=comdata.substring(hindex+1).toInt()-170;//h 坐标偏移100 客户端初始50 即-50
              y=comdata.substring(yindex+1,hindex).toInt();
            }else {
              y=comdata.substring(yindex+1).toInt();
            }
        break;//change the "x"and"y" it is possible that the high change as well; 
        case 'h':
          h=comdata.substring(1).toInt()-170;
        break;//only change the "high"
        case '0':
          isdrip=false;
        break;//stop pour water
      }
      //setinmax(&x,45,-45);
      //setinmax(&y,45,-45);
      //setinmax(&h,-70,-150);
      if(isdrip){
        Serial.print("x:");
        Serial.print(x);
        Serial.print("y:");
        Serial.print(y);
        Serial.print("h:");
        Serial.println(h);
      }
      //Serial.print(comdata);
      comdata = "";
    }
}

void receivesrh(){
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
      Serial.println(comdata);
      int sindex=0;
      int rindex=0;
      int hindex=0;
      switch(comdata[0]){
        case 's':
            //isdrip=true;
            //sindex=comdata.indexOf('s',0);
            if(comdata.length()>3){
              s=comdata.substring(1,3).toInt();
            }else{
              s=comdata.substring(1).toInt();
            }
            
        break;//change the "x"and"y" it is possible that the high change as well; 
        case 'r':
            isdrip=true;
            //sindex=comdata.indexOf('s',0);
            if(comdata.length()>3){
              r=comdata.substring(1,3).toInt();
            }else{
              r=comdata.substring(1).toInt();
            }
        break;
        case 'h':
            //isdrip=true;
           if(comdata.length()>3){
              h=comdata.substring(1,3).toInt()-170;
            }else{
              h=comdata.substring(1).toInt()-170;
            }
        break;//only change the "high"
        case '0':
          isdrip=false;
        break;//stop pour water
      }
      //setinmax(&x,45,-45);
      //setinmax(&y,45,-45);
      setinmaxint(&r,49,0);
      setinmaxint(&s,100,0);
      setinmax(&h,-70,-150);
      if(isdrip){
       
        Serial.print("s:");
        Serial.print(s);
        Serial.print("r:");
        Serial.print(r);
        Serial.print("h:");
        Serial.println(h);
      }else{
        Serial.println("not drip");
        comdata = "";
      }
      //Serial.print(comdata);
      comdata = "";
    }
}
void movetop2(){

  if(isdrip){
    cangle+=PI*s/2500;//2*PI/20*s/100/5
    if(cangle>6.28){
      cangle-=2*PI;
    }
    x= (float) (r*cos(cangle));
    y= (float) (r*sin(cangle));
    setinmax(&x,45,-45);
    setinmax(&y,45,-45);
    setinmax(&h,-90,-150);
    /*
    Serial.print("x:");
    Serial.print(x);
    Serial.print("y:");
    Serial.print(y);
    Serial.print("h:");
    Serial.println(h);*/
    xyztoangle(x, h, y, &ang);
    Serial.print("  ");
    Serial.print(ang.angle1);
    Serial.print("  ");
    Serial.print(ang.angle2);
    Serial.print("  ");
    Serial.println(ang.angle3);
    //isdrip=false;  
    myservo1.write(initangle+ang.angle1+3);
    myservo2.write(initangle+ang.angle2-2);
    myservo3.write(initangle+ang.angle3+5); 
  }
}
void watchreach(){
   waterstate=digitalRead(waterpin);
  if(waterstate==LOW||reached){//此处只要有一次到达就不再进水,保护(实际项目中把"||reached"删掉)
    digitalWrite(ledpin,HIGH);
    digitalWrite(relaypump,LOW);
    Serial.println("reached");
    reached=true;
    myPID.Compute();
  }else{
    Serial.println("unreached");
    digitalWrite(ledpin,LOW);
    digitalWrite(relaypump,HIGH);
    reached=false;
  }
}
void pidcontrol(){
  if(reached){
    
    if(Output>0){
      if(Output>100)Output=100;
      if(pidstep<50){
        if(pidstep<Output/2){
          digitalWrite(relaypin,HIGH);
        }else{
          digitalWrite(relaypin,LOW);
        }
        pidstep++;
      }else pidstep=0;
    }else digitalWrite(relaypin,LOW);
  }
}
void gettemp(){
    //Input=ts.getCelsius();
     average_filter(ts.getCelsius(),&Input);
}
