#ifndef ANG_H_INCLUDED
#define ANG_H_INCLUDED

//extern const float shin,thigh,baseside,effectorside;
//extern const double leg;
extern float x,y,z;
typedef struct ANGLE angle;
struct ANGLE{
float angle1;
float angle2;
float angle3;
};

float Angle(float x,float y,float z);
void xyztoangle(float x,float y,float z,angle* a);



#endif // ANGLE_H_INCLUDED
