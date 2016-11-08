#include<Arduino.h>
#include <Event.h>
#include <Timer.h>
#include <Servo.h>
#include<stdio.h>
#include<math.h>
#include "ang.h"
#include "Max6675_1.h"
#include"pid.h"
#include <FlexiTimer2.h>
#include "HX711_A.h"
/*
6,7,8 are used at max6675
22,24,26 are used at three servo which drive the machine hand 
28加热继电器 30 input 水位机 32 水泵继电器
34,36,38,40压力传感器
*/
/************************serial*****************************/
char com;
String comdata = "";

/*************************串口0数据********************************/
//send
bool ifrecognize=false;//是否开启单目识别
//receive
int powdercircle=0;//咖啡液比例
int foam=0;//泡沫比例
int mode=0;//true:极坐标模式 false:xyz模式 双目识别 还是单目?
/************************串口2数据*********************************/
bool stereostart=false;//是否开始双目注水
int o_point[3];//双目模式的原点
/****************************插值**********************************/
float X[3]={0,0,0};
float Y[3]={0,0,0};
float H[3]={110,110,110};
float CX[11]={0};
float CY[11]={0};
float CH[11]={110};
/************************引脚*************************************/
Timer t;  //instantiate the timer object
bool reached=false;
bool waterstate=LOW;
const int relaypin=28;//加热继电器
const int waterpin=30;//水位机
const int ledpin=13;//led 没啥卵用
const int relaypump=32;//控温初始化
const int relaydrip=34;//水阀
/********************水流机**************************/
int lasttime=0;//初始化水流量/时间/流速
float waterFlow = 0;
float lastwaterFlow = 0;
const int time=1000;
float currentSpeed = 0;
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
/*********************压力**************************************/
HX711 hx_r(34, 36);//SCK,DT,AMP,CO
HX711 hx_l(38, 40);
HX711 hx_m(42,44);
int l_offset=0;
int r_offset=0;//初始化offset和CO//0.00237417
int m_offset=0;//初始化offset和CO//0.00237417
float weigh_l,weigh_r ,weigh,weigh_m;
/*******************函数声明***************************************/
void sreceive();//串口获得xyz  mode=1
void receivesrh();//串口读取极坐标 mode=2
void movetop2();//mode2 极坐标定位
void watchreach();//获得滤波后的温度
void getweight();//获得重量(两个)
void serialappout();//响app发送信息
void serialhuin();//获得手冲壶信息
void serialpiout();//发送pi信号
void serialpiin();//接收pi信号 双目在此 mode=3

void setup() {
  //中断初始化
  attachInterrupt(0, waterpulse, RISING);
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
  //压力初始化
  r_offset=hx_r.tare();
  l_offset=hx_l.read();
  m_offset=hx_m.read();//设置offset
  hx_l.set_co(0.00237417);//左压力传感初始化    
  hx_r.set_co(0.00246840);
  hx_m.set_co(0.00091604);  
  hx_l.set_offset(l_offset);//76106
  hx_r.set_offset(r_offset);//右压力传感初始化
  hx_r.set_offset(76006);
  hx_m.set_offset(77421);
   //控温初始化
  ts.setOffset(0);
  myPID.SetOutputLimits(0, outputmax);//告诉PID在从0到max的范围内取值
  myPID.SetMode(AUTOMATIC); //开启PID
  lasttemp = ts.getCelsius();
  //定时器初始化
  FlexiTimer2::set(20, 1.0/1000, movetop2); // call every 500 1ms "ticks"
  FlexiTimer2::start();
  t.every(250, gettemp);//250更新一次温度
  t.every(50,pidcontrol);//
  t.every(time,waterspeed);//每1000ms去计算一次流速
  t.every(1000,serialappout);//每1000ms去发送给app
  t.every(2000,serialpiout);//每1000ms去计算一次流速
}

void loop() {  
  watchreach();
  t.update();
  if(mode==0){
    receivesrh();
  }
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
        case 't':
          ifpid=true;//接收到新的温度可以进行pid加热
          if(comdata.length()>3){
            Setpoint=comdata.substring(1,3).toInt();
          }else{
            Setpoint=comdata.substring(1).toInt();
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
          ifpid=false;
          dripstart=false;
          reached=false;//reached置为false 关闭冲煮意味着可以进水 让水泵重新有机会进行进水
        break;
        case '0':
          isdrip=false;
        break;//stop pour water
      }
      setinmaxint(&r,49,0);
      setinmaxint(&s,100,0);
      setinmax(&h,-100,-150);
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
    if(mode==0){
      cangle+=PI*s/2500;//2*PI/20*s/100/5
      if(cangle>6.28){
        cangle-=2*PI;
      }
      x= (float) (r*cos(cangle));
      y= (float) (r*sin(cangle));
    }if(mode==3){
      x=getpoint(CX);
      y=getpoint(CY);
      y=getpoint(CH);
    }
    setinmax(&x,45,-45);
    setinmax(&y,45,-45);
    setinmax(&h,-100,-150);
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
    digitalWrite(relaydrip,HIGH);
  }else digitalWrite(relaydrip,LOW);
}
void watchreach(){
  if(!dripstart){//当串口把start reached=false后会读取水位机 即进水  冲煮完就进水
    waterstate=digitalRead(waterpin);
  } else waterstate=LOW;//为了注水时绝对不会进水
  if(waterstate==LOW||reached){//此处只要有一次到达就不再进水,保护(实际项目中把"||reached"删掉)
    digitalWrite(ledpin,HIGH);
    digitalWrite(relaypump,LOW);
    //Serial.println("reached");
    reached=true;
    if(ifpid)//冲煮完进水后并不是马上进行加热的 因此没必要进行pid计算 下次冲煮接收到温度再计算
      myPID.Compute();      
  }else{
    Serial.println("unreached");
    digitalWrite(ledpin,LOW);
    digitalWrite(relaypump,HIGH);
    reached=false;
    myPID.SetIterm(0);//在进水时把积分清零 以免上一次加热用到的积分在下次继续用
  }
}
void pidcontrol(){
  if(reached&&ifpid){//只是进水不能开始控温 要再加个判断 这里暂时添加dripstart 但是不符合的 应该是接收到温度的判断
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
  }else digitalWrite(relaypin,LOW);
}
void gettemp(){
    average_filter(ts.getCelsius(),&Input);
}
void getweight(){
  hx_l.average_filter(hx_l.bias_read(),&weigh_l);
  hx_r.average_filter(hx_r.bias_read(),&weigh_r);
  hx_m.average_filter(hx_m.bias_read(),&weigh_m);
  weigh=weigh_r+weigh_l;
}
void waterpulse()   //measure the quantity of square wave
{
  waterFlow += 0.85034;//5/5.880
}
void waterspeed(){
  float waterml=waterFlow-lastwaterFlow;
  currentSpeed = waterml/time*1000;//
  lastwaterFlow=waterFlow;
}
void serialappout(){
  Serial1.println(String("")+"p"+powdercircle);
  Serial1.println(String("")+"w"+weigh);
  Serial1.println(String("")+"m"+weigh_m);
  Serial1.println(String("")+"f"+waterFlow);
  Serial1.println(String("")+"t"+(int)Input);
}
void serialpiout(){
  Serial.println(String("")+"t"+(int)Input);//当时温度
  if(ifrecognize&&(mode==0||mode==1))
    Serial.println("s");//单目开始识别
}
void serialpiin(){
     while (Serial.available() > 0 )  
    {
      com=char(Serial.read());
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
        case '0':
          mode=0;
        break;
        case '2':
          mode=2;
        break;
        case '3':
          ifrecognize=true;
        break;
        case 'p':
          powdercircle=comdata.substring(1,3).toInt();
        break;
        case 'f':
          foam=comdata.substring(1,3).toInt();
        break;
      }
      comdata = "";
      if(isdrip){
        float zx[5],zy[5],zh[5];
        computedvalue(x-o_point[0],X,zx);
        computedvalue(y-o_point[1],Y,zy);
        computedvalue(h-o_point[2],H,zh);
        addtoc(zx,CX);
        addtoc(zy,CY);
        addtoc(zh,CH);
      }
    }
}
void serialhuin(){
   while (Serial2.available() > 0 )  
    {
      com=char(Serial2.read());
      if(com==' '){
        break;
      }
      comdata += com;
      delay(6);
    }
    if (comdata.length() > 0)
    {
      //Serial.println(comdata);
      switch(comdata[0]){
        case 'o'://确定原点
          o_point[0]=x;
          o_point[1]=y;
          o_point[2]=h;
          stereostart=true; 
        break;//change the "x"and"y" it is possible that the high change as well; 
        case 'd':
          isdrip=true;
        break;
        case '0':
          isdrip=false;
        break;
      }
      comdata = "";
    }      
}
