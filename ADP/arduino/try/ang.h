#ifndef ANG_H_INCLUDED
#define ANG_H_INCLUDED

//extern const float shin,thigh,baseside,effectorside;
//extern const double leg;
extern int x,y,h;
extern const int initangle;
extern bool isdrip;
typedef struct ANGLE angle;
struct ANGLE{
float angle1;
float angle2;
float angle3;
};
void setinmax(int *j,int max, int min);
float Angle(float x,float y,float z);
void xyztoangle(int x,int y,int z,angle* a);



#endif // ANGLE_H_INCLUDED

