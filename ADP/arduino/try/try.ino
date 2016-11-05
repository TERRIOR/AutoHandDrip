#include <Event.h>
#include <Timer.h>
#include <Servo.h>
#include<stdio.h>
#include<math.h>
#include "ang.h"
#include "Max6675_1.h"
#include"pid.h"
#include <FlexiTimer2.h>
/*
6,7,8 are used at max6675
22,24,26 are used at three servo which drive the machine hand 
28加热继电器 30 input 水位机 32 水泵继电器
*/
char com;
String comdata = "";
Timer t;  //instantiate the timer object
bool reached=false;
bool waterstate=LOW;
const int relaypin=28;//加热继电器
const int waterpin=30;//水位机
const int ledpin=13;//led 没啥卵用
const int relaypump=32;//控温初始化
const int relaydrip=34;//水阀
/*******************pid参数**********************/
const int outputmax = 100;//输出的最大值
int out;//最终的输出
int pidstep=0;//脉宽
Max6675 ts(6,7,8);//原为9,10,11没必要
// 定义我们将要使用的变量
double Setpoint=90, Input, Output;//pid库计算的变量
double kp=5,ki=0.001,kd=0;//pid参数
//指定链接和最初的调优参数
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT);//初始化 传递参数指针
/*******************舵机参数******************************/
Servo myservo1,myservo2,myservo3;
angle ang={0,0,0};//写默认的角度 暂时先写0,0,0
/*******************函数声明***************************************/
void sreceive();//串口获得xyz
void movetop();//运动到xyz
void receivesrh();//串口读取极坐标
void movetop2();//mode2 极坐标定位
void watchreach();//获得滤波后的温度
void setup() {
  //引脚初始化
  pinMode(relaypin,OUTPUT);
  pinMode(waterpin,INPUT);
  pinMode(ledpin,OUTPUT);
  pinMode(relaypump,OUTPUT);
  pinMode(relaydrip,OUTPUT);
  //串口初始化
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
  myPID.SetOutputLimits(0, outputmax);//告诉PID在从0到max的范围内取值
  myPID.SetMode(AUTOMATIC); //开启PID
  lasttemp = ts.getCelsius();
  //定时器初始化
  FlexiTimer2::set(20, 1.0/1000, movetop2); // call every 500 1ms "ticks"
  FlexiTimer2::start();
  t.every(250, gettemp);
  t.every(50,pidcontrol);
  //t.every(100, movetop);
}

void loop() {  
  watchreach();
  t.update();
  receivesrh();
}
void movetop(){
  sreceive();
  if(isdrip){
    digitalWrite(relaydrip,HIGH);
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
    digitalWrite(relaydrip,LOW);
  }
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
        case '1'://开启冲煮 app开始计时
          dripstart=true;
        break;
        case '2'://关闭冲煮过程
          dripstart=false;
          reached=false;//reached置为false 关闭冲煮意味着可以进水 让水泵重新有机会进行进水
        break;
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
  if(isdrip&&dripstart){
    cangle+=PI*s/2500;//2*PI/20*s/100/5
    if(cangle>6.28){
      cangle-=2*PI;
    }
    digitalWrite(relaydrip,HIGH);
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
    Serial.println(h);
    */
    xyztoangle(x, h, y, &ang);
    Serial.print("  ");
    Serial.print(ang.angle1);
    Serial.print("  ");
    Serial.print(ang.angle2);
    Serial.print("  ");
    Serial.println(ang.angle3);
    myservo1.write(initangle+ang.angle1+3);
    myservo2.write(initangle+ang.angle2-2);
    myservo3.write(initangle+ang.angle3+5); 
  }else digitalWrite(relaydrip,LOW);
}
void watchreach(){
  
  if(!dripstart){
    waterstate=digitalRead(waterpin);
  } else waterstate=LOW;//为了注水时绝对不会进水
  if(waterstate==LOW||reached){//此处只要有一次到达就不再进水,保护(实际项目中把"||reached"删掉)
    digitalWrite(ledpin,HIGH);
    digitalWrite(relaypump,LOW);
    //Serial.println("reached");
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
      if(Output>outputmax)out=outputmax;
      else out=Output;
      if(Input<Setpoint*0.92) 
      {
        out=outputmax;
      }else if(Input>Setpoint){
        out=0;
      }
      if(pidstep<50){
        if(pidstep<out/2){
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
    average_filter(ts.getCelsius(),&Input);
}
