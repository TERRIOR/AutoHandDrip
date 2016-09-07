#include<math.h>
#include<stdio.h>
#include <Arduino.h>
#include "ang.h"
const double leg = 6.28*120/360;
const float shin=30,thigh=40,baseside=0,effectorside=0;
float x,y,z;
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
 float c=sqrt((x+effectorside-baseside)*(x+effectorside-baseside)+y*y+(z+effectorside-baseside)*(z+effectorside-baseside));
 float a2=shin*shin-z*z;
 float alphy = acos((-a2+thigh*thigh+c*c)/(2*thigh*c));
 float beta = -atan2(y,x);
 float i = alphy-beta;
 float angle = i/6.28*360;     
 return angle;
}

void xyztoangle(float x,float y,float z,angle *a){
  posi p2,p3;
  posi p1;
  p1.x=x;
  p1.y=y;
  p1.z=z;
  changeangle(&p1,&p2);
  changeangle(&p2,&p3);
  Serial.println(p1.x);
  Serial.println(p1.y);
  Serial.println(p1.z);
  a->angle1=Angle(p1.x,p1.y,p1.z);
  a->angle2=Angle(p2.x,p2.y,p2.z);
  a->angle3=Angle(p3.x,p3.y,p3.z);
  //Serial.println(a->angle1);

}
