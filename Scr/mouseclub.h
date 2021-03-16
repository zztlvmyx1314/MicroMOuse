#ifndef  __MOUSECLUB_H__
#define  ___MOUSECLUB_H__
#include <Mouse_Drive.h>
float disr(float left,float right,float leftbiao,float leftyu,float rightbiao,float rightyu);
float dis(float left,float right,float leftbiao,float leftyu,float rightbiao,float rightyu);

float rightd(float dianya,int bijiaozhi,int quzhenshu);
float flid(float dianya,int bijiaozhi,int quzhenshu);
float frid(float dianya,int bijiaozhi,int quzhenshu);
float leftd(float dianya,int bijiaozhi,int quzhenshu);

 float dis90VR(uint16 DisFlX,uint16 DisFrX);
float  dis45(uint16 DisFlX,uint16 DisFrX);
float dis90VL(uint16 DisFlX,uint16 DisFrX);
float Spurt45(uint16 DisFlX,uint16 DisFrX);
void actiongenerate(); 
float pianchal(float right,float rightd);
float pianchar(float left,float leftd);
#endif 