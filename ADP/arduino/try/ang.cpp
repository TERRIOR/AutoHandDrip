#include<math.h>
#include<stdio.h>
#include <Arduino.h>
#include "ang.h"
const double leg = 6.28*120/360;
const int initangle=30;
//shin:小腿 thigh :大腿  base:上方的 effect:下面的
const float shin=110,thigh=50,basesize=35,effectorsize=22;
bool isdrip=false;
int x,y,h;
struct posi{
  float x;
  float y;
  float z;
};
typedef struct posi posi;
/*float dist(float x1,float x2,float y1,float y2)
{
float distance;
distance=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
return distance;
}*/
//转换坐标
void changeangle(posi* posi_1,posi* posi_2){
        (posi_2->x) = (posi_1->x)*cos(leg) - (posi_1->z)*sin(leg);
        (posi_2->z) = (posi_1->x)*sin(leg) +(posi_1->z)*cos(leg);
        (posi_2->y) = (posi_1->y);
}
//计算坐标  
float Angle(float x,float y,float z){
 float c=sqrt((x+effectorsize-basesize)*(x+effectorsize-basesize)+y*y);//计算舵机旋转中心到小腿下关节(投影)的距离
 float a2=shin*shin-z*z;//算出投影距离
 float alphy = acos((-a2+thigh*thigh+c*c)/(2*thigh*c));//余弦函数
 float beta = -atan2(y,x+effectorsize-basesize);//反正切函数  书中原代码为:-atan2(y,x) 可是按算法来说是错的 
 float i = alphy-beta;
 float angle = i/6.28*360;     
 return angle;
}

void xyztoangle(int x,int y,int z,angle *a){
  posi p2,p3;
  posi p1;
  p1.x=x;
  p1.y=y;
  p1.z=z;
  changeangle(&p1,&p2);
  changeangle(&p2,&p3);
  /*Serial.println(p1.x);
  Serial.println(p1.y);
  Serial.println(p1.z);*/
  a->angle1=Angle(p1.x,p1.y,p1.z);
  a->angle2=Angle(p2.x,p2.y,p2.z);
  a->angle3=Angle(p3.x,p3.y,p3.z);
  //Serial.println(a->angle1);

}
void setinmax(int *j,int max, int min){
  if(*j>max)*j=max;
  if(*j<min)*j=min;
}
