#include "Arduino.h"
#include "mecarun_v2.h"

void Mecarun_v2::cal_mecanum(void)
{
	int16_t x,y,r;
	int16_t RF,LF,RB,LB;
	int16_t xrf,yrf,rrf;//????
	int16_t xlf,ylf,rlf;
	int16_t xrb,yrb,rrb;
	int16_t xlb,ylb,rlb;
	int16_t max;
	float tmp;

	y=this->speed_xyr.y;
	x=this->speed_xyr.x;
	r=this->speed_xyr.r;

	/*????x,y,r??*/
	xrf=-x;
	xlf=-x;
	xrb=x;
	xlb=x;

	yrf=y;
	ylf=-y;
	yrb=y;
	ylb=-y;

	rrf=-r;
	rlf=-r;
	rrb=-r;
	rlb=-r;

	/*????*/
	RF=xrf+yrf+rrf;
	LF=xlf+ylf+rlf;
	RB=xrb+yrb+rrb;
	LB=xlb+ylb+rlb;

 if(RF>=DUTY_MAX||LF>=DUTY_MAX||RB>=DUTY_MAX||LB>=DUTY_MAX)//?????????????
 {
	 max=RF;
	 if(LF>max)max=LF;
	 if(RB>max)max=RB;
	 if(LB>max)max=LB;

	 tmp=(float)DUTY_MAX/(float)max;//??
	 RF=RF*tmp;
	 LF=LF*tmp;
	 RB=RB*tmp;
	 LB=LB*tmp;
 }

	this->motor[0].target=LF;
	this->motor[1].target=RF;
	this->motor[2].target=LB;
	this->motor[3].target=RB;

	this->mt_ctrl.motor_update = 1;
}
