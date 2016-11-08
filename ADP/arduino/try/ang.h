#ifndef ANG_H_INCLUDED
#define ANG_H_INCLUDED

//extern const float shin,thigh,baseside,effectorside;
//extern const double leg;
extern int s,r;
extern const int initangle;
extern bool isdrip,dripstart;
extern float  h,x,y,cangle;
typedef struct ANGLE angle;
struct ANGLE{
float angle1;
float angle2;
float angle3;
};
void setinmax(float *j,int max, int min);
float Angle(float x,float y,float z);
void xyztoangle(int x,int y,int z,angle* a);
void setinmaxint(int *j,int max, int min);
void addtoc(float *dvalue,float *csz);
void computedvalue(float secpoint,float *z,float *dvalue);
float getpoint(float *shuzu);

#endif // ANGLE_H_INCLUDED

