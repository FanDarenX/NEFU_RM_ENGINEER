#include "bsp_adrc.h"
#include "math.h"
//参数区，这11个就是需要用户整定的参数
/****************TD**********/



void ADRC_Init(ADRC_t *ADRC,
	             float r,float h,//TD
							 float b,float delta,float belta01,float belta02,float belta03,//ESO
							 float alpha1,float alpha2,float belta1,float belta2)//NLSEF
{
	//中间变量等于0
  ADRC->x1=0;ADRC->x2=0;
  ADRC->e =0;ADRC->u=0;
	ADRC->z1=0;ADRC->z2=0;ADRC->z3=0;
	//TD
	ADRC->r=r;           ADRC->h=h;
	//ESO
	ADRC->b=b;           ADRC->delta=delta;   ADRC->belta01=belta01; ADRC->belta02=belta02; ADRC->belta03=belta03;
	//NLSEF
	ADRC->alpha1=alpha1; ADRC->alpha2=alpha2; ADRC->belta1=belta1;   ADRC->belta2=belta2;
}

/*****************************fhan函数*********************************/
float fhan(float x1,float x2,float r,float h)
{
	float deltaa  =0,
		    deltaa0 =0,
	      y       =0,
	      a0      =0,
	      a       =0,
	      fhan    =0;
	
	deltaa = r*h;
	deltaa0 = deltaa*h;
	y=x1+x2*h;
	a0 = sqrtf(deltaa*deltaa+8*r*fabsf(y));
	if(fabsf(y)<=deltaa0)
		a=x2+y/h;
	else
		a=x2+0.5*(a0-deltaa)*sign(y);
	if(fabsf(a)<=deltaa)
		fhan = -r*a/deltaa;
	else
		fhan = -r*sign(a);
	
  return fhan;
}
/************************************sign函数***************************/
float sign_adrc(float x)
{
	if(x>0)
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}
/*******************************fal函数**********************************/
float fal(float e,float alpha,float delta)
{
  float result = 0,fabsf_e = 0;
  
  fabsf_e = fabsf(e);
  
  if(delta>=fabsf_e)
    result = e/powf(delta,1.0-alpha);
  else //if(delta<fabsf_e)
    result = powf(fabsf_e,alpha)*sign(e);
 
 return result;     
}

/********************************ADRC************************************/
float ADRC(ADRC_t *ADRC,float v,float y)  //  参数：v：输入的目标值；  y：反馈值
{
  float u0 = 0,
        e1 = 0,
	    	e2 = 0;
/******************************TD****************************************/
  ADRC->x1 = ADRC->x1 + ADRC->h*ADRC->x2;
  ADRC->x2 = ADRC->x2 + ADRC->h*fhan(ADRC->x1-v,ADRC->x2,ADRC->r,ADRC->h);
/******************************ESO***************************************/
  ADRC->e = ADRC->z1 - y;
  ADRC->z1 = ADRC->z1 + ADRC->h*(ADRC->z2-ADRC->belta01*ADRC->e);
  ADRC->z2 = ADRC->z2 + ADRC->h*(ADRC->z3-ADRC->belta02*fal(ADRC->e,0.5,ADRC->delta)+ADRC->b*ADRC->u);
  ADRC->z3 = ADRC->z3 + ADRC->h*(-ADRC->belta03*fal(ADRC->e,0.25,ADRC->delta));
/******************限幅，ADRC正常的话不会达到限幅条件********************/
  if(ADRC->z1>=30000) ADRC->z1=30000;
  if(ADRC->z1<=-30000) ADRC->z1 = -30000;
  if(ADRC->z2>=30000) ADRC->z2=30000;
  if(ADRC->z2<=-30000) ADRC->z2 = -30000;
  if(ADRC->z3>=30000) ADRC->z3=30000;
  if(ADRC->z3<=-30000) ADRC->z3 = -30000;
/******************************NLSEF*************************************/
  e1 = ADRC->x1 - ADRC->z1;
  e2 = ADRC->x2 - ADRC->z2;
  
  u0 = ADRC->belta1*fal(e1,ADRC->alpha1,ADRC->delta) + ADRC->belta2*fal(e2,ADRC->alpha2,ADRC->delta);//其中0<alpha1<1<alpha2
  
  ADRC->u = u0 - ADRC->z3/ADRC->b;
  
  return ADRC->u;
}